(* An OCaml robot on the core binding supplied by the arena.
   Bearing is earned: a scan contact's bearing is off by up to its
   bearing_error (degrees, a fixed share of the arc), so a wide beam finds a
   robot and a narrow one aims at it. The robot narrows the beam, holds it
   on the target and aims along a line fitted through its recent sightings.
   Contacts within 120 units are exact. Remembered passive cues guide the search.
   Clunkyard robot API 2.1.0: one core under every language, the same
   answers everywhere. Facts (a call shows its arguments):
   match: mode ruleset control robots is_3d tick_rate max_ticks
     robot_id(slot)
   clock: tick seconds remaining_ticks
   arena: size gravity terrain_kind ground_height(x y)
   me: health position velocity heading pitch roll angular_velocity
     attitude grounded altitude wall_contact hurt applied hull_mounts
     to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
   battery: charge capacity regen sustainable_drive compute
     drive_cost(fraction)
   drive: hover_lift
   gun: aim_now cooldown_ticks spread max_power damage_scale
     can_fire(power reserve) shot_speed(power) shot_cost(power)
     reload_ticks(power) shot_damage(power) power_for_speed(speed)
     power_for_flight_time(range seconds) lead(contact power)
     lead_point(position velocity power)
   scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
   sensors: contacts nearest cues contact(slot) position_of(contact)
   tracker: cue nearest_bearing_rate velocity(slot) position(slot)
     history(slot) last_seen(slot)
   nav: zone zone.margin(point) zone.contains(point margin)
     zone.distance_from(point) storm storm.margin(point)
     storm.contains(point margin) storm.distance_from(point) hill
     hill.margin(point) hill.contains(point margin)
     hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
     field_distance(a b)
   score: me best armour
   drone: has ready in_flight launches_left report sightings
   angles: normalize(degrees) angle_diff(target current)
     bearing_to(from to) elevation_to(from to) direction(heading pitch)
     distance(a b) to_chassis(heading pitch attitude)
     from_chassis(heading pitch attitude) clamp(value low high)
   raw: number(path) text(path) has(path) flag(path) count(path) line()
   Controls: drive.set(throttle turn pitch_turn)
     drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
     gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
     scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
     drone.launch(heading pitch param) drone.brief(index value)
     drone.program(language source) status.set(text value) debug.log(line)
     raw.extra(json)
   Every control argument is optional; a second call keeps what the first
   set. Absent facts are your language's none. Names are snake_case here;
   your language cases them its own way. Angles are degrees from the +x
   axis; pitch is positive up.
   Manual: https://clunkyard.com/manual#sdk
*)
open Clunkyard
let search_arc = 40.  (* degrees: sweeps the circle in half a second *)
let hold_width = 60.  (* units across at the target: it stays in the beam from tick to tick *)
let min_arc = 6.      (* degrees: the narrowest the beam holds *)
let window = 12       (* ticks of sightings the aim averages *)
let sightings = 8     (* narrow sightings in the window before a shot *)
let look_every = 40   (* ticks between looks around the target, for a nearer robot *)
let reacquire = 10    (* ticks to look again where a lost target was *)
let free = ref false
(* The last target's slot, for looking again where it was lost. *)
let slot = ref 0
(* Dodges taken, and the tick the current one ends. *)
let dodges = ref 0 and dodge_end = ref 0
let clamp value = Angles.clamp value (-1.) 1.
let steer towards current = clamp (Angles.angle_diff towards current /. 45.)
let on_start robot = free := Match.is_3d robot && (Arena.gravity robot).z = 0.

let sum f xs = List.fold_left (fun total x -> total +. f x) 0. xs

(* A straight line through the narrowest of the last [window] ticks' sightings:
   where the robot is now and how fast it moves. Each sighting is off by its
   bearing_error; the line averages that out. *)
let fit robot slot tick =
  let recent = List.filter (fun (s:sighting) -> s.tick > tick - window) (Tracker.history robot slot) in
  if List.length recent < 2 then None else
  let best = List.fold_left (fun least (s:sighting) -> min least s.bearing_error) infinity recent in
  let seen = List.filter (fun (s:sighting) -> s.bearing_error <= 1.5 *. best) recent in (* the narrow ones *)
  let count = List.length seen in
  if count < 2 then None else
  let n = float count in
  let t0 = sum (fun (s:sighting) -> float s.tick) seen /. n in
  let spread = sum (fun (s:sighting) -> (float s.tick -. t0) ** 2.) seen in
  (* One axis: the line's position now and its slope per tick. *)
  let axis along =
    let mean = sum (fun (s:sighting) -> along s.position) seen /. n in
    let slope = sum (fun (s:sighting) -> (float s.tick -. t0) *. (along s.position -. mean)) seen /. spread in
    mean +. slope *. (float tick -. t0), slope *. float (Match.tick_rate robot) in
  let (px,vx),(py,vy),(pz,vz) = axis (fun (v:vec3) -> v.x), axis (fun (v:vec3) -> v.y), axis (fun (v:vec3) -> v.z) in
  Some ({x=px;y=py;z=pz}, {x=vx;y=vy;z=vz}, count)

let on_turn robot =
  let position=Me.position robot and heading=Me.heading robot and pitch=Me.pitch robot in
  let tick=Clock.tick robot in
  Drive.forces robot ~brake:0.2 ();
  let lost = !slot <> 0 && (match Tracker.last_seen robot !slot with Some seen -> tick-seen <= reacquire | None -> false) in
  (match Sensors.nearest robot,Tracker.cue robot with
  | Some target,_ ->
    let throttle=if target.range>350. then 0.7 else if target.range<200. then -0.4 else 0.2 in
    Drive.set robot ~throttle ~turn:(steer target.bearing heading) ~pitch_turn:(steer target.elevation pitch) ();
    slot := target.slot;
    (* Halve the beam each tick until it is hold_width across at the target:
       narrow enough for a precise bearing, wide enough to keep it in. *)
    let hold=max min_arc (2.*.atan2 (hold_width/.2.) target.range*.180./.Float.pi) in
    (* In a crowd a narrow beam sees one robot. Every look_every ticks,
       widen it for a tick: a nearer robot it finds becomes the target. *)
    let arc=if tick mod look_every=0 then search_arc else max hold (min search_arc ((Scanner.last robot).arc/.2.)) in
    Scanner.set robot ~heading:target.bearing ~arc ~pitch:target.elevation ();
    let power=Gun.power_for_flight_time robot target.range 1. in
    (* An exact contact: Gun.lead is the intercept if the target holds course.
       A noisy one: lead the fitted line, and fire only once it rests on enough sightings. *)
    let aim,ready=
      if target.bearing_error>0. then
        (match fit robot target.slot tick with
         | Some(p,v,count)->Gun.lead_point robot p v power,count>=sightings
         | None->None,false)
      else Gun.lead robot target power,true in
    let h,p=match aim with None->target.bearing,target.elevation|Some (a:solution)->a.heading,a.pitch in
    Gun.aim robot ~heading:h ~pitch:p ();
    let now=Gun.aim_now robot in
    if ready && abs_float(Angles.angle_diff h now.heading)<3. && abs_float(p-.now.pitch)<3. && Gun.can_fire robot power 10. then Gun.fire robot ~power ();
    Status.set robot ~text:(if ready then "TRACK" else "NARROW") ~value:target.range ()
  | None,_ when lost ->
    (* Lost it: look again where it was last seen, widening the beam, before searching. *)
    let where=Option.value(Tracker.position robot !slot)~default:position in
    let bearing=Angles.bearing_to position where in
    let elevation=if !free then Angles.elevation_to position where else 0. in
    Drive.set robot ~throttle:0.5 ~turn:(steer bearing heading) ~pitch_turn:(steer elevation pitch) ();
    Scanner.set robot ~heading:bearing ~arc:(min search_arc ((Scanner.last robot).arc*.2.)) ~pitch:elevation ();
    Status.set robot ~text:"REACQUIRE" ()
  | None,Some cue ->
    let bearing=Angles.bearing_to position cue.position and distance=Angles.distance position cue.position in
    let elevation=if !free then Angles.elevation_to position cue.position else 0. in
    let error=min cue.bearing_error 30. in
    let across=float(tick mod 5-2)/.2.*.error and up=if !free then float((tick/5)mod 3-1)*.error/.2. else 0. in
    Drive.set robot ~throttle:0.7 ~turn:(steer bearing heading) ~pitch_turn:(steer elevation pitch) ();
    Scanner.set robot ~heading:(bearing+.across) ~arc:(Scanner.arc_for_reach robot (distance+.100.)) ~pitch:(elevation+.up) ();
    Status.set robot ~text:"CUE" ~value:distance ()
  | None,None ->
    let up=if !free then float((tick/10)mod 3-1)*.35. else 0. in
    Drive.set robot ~throttle:0.5 ~turn:0.33 ~pitch_turn:(steer 0. pitch) ();
    Scanner.set robot ~heading:(Angles.normalize((Scanner.last robot).heading+.36.)) ~arc:search_arc ~pitch:up ();
    Status.set robot ~text:"SEARCH" ());
  (* A simple dodge: vary it, or predict the shooter. *)
  if tick >= !dodge_end && List.exists (fun (c:contact) -> c.source="muzzle") (Sensors.cues robot) then begin
    incr dodges; dodge_end := tick+10
  end;
  if tick < !dodge_end then begin
    (* Off the line of fire: left, then right next time (turn is positive left, strafe positive right). *)
    let left= !dodges mod 2=1 in
    if Match.control robot="forces" then begin
      Drive.set robot ~throttle:0.5 ();
      Drive.forces robot ~strafe:(if left then -1. else 1.) ()
    end else Drive.set robot ~throttle:1. ~turn:(if left then 1. else -1.) ();
    Status.set robot ~text:(if left then "dodge:left" else "dodge:right") ~value:(float !dodges) ()
  end;
  if Battery.charge robot < Battery.capacity robot /. 4. then Drive.set robot ~throttle:(Option.value(Drive.get_throttle robot)~default:0.*.0.5) ();
  (match Nav.storm robot with
  | Some storm when Option.value(Nav.storm_margin robot position)~default:neg_infinity > -60. ->
    let away=Angles.bearing_to storm.center position in
    Drive.set robot ~throttle:0.8 ~turn:(clamp(Angles.angle_diff away heading/.30.)) ()
  | _ -> ());
  (match Nav.zone robot with
  | Some zone when Option.value(Nav.zone_margin robot position)~default:infinity < 60. ->
    let climb=if zone.shape="sphere" then Angles.elevation_to position zone.center else 0. in
    Drive.set robot ~throttle:0.8 ~turn:(clamp(Angles.angle_diff(Angles.bearing_to position zone.center)heading/.30.)) ~pitch_turn:(clamp((climb-.pitch)/.30.)) ()
  | _ -> ());
  ()
let () = run ~on_start ~on_turn ()
