(* An OCaml robot on the core binding supplied by the arena.
   Precise contacts lead shots; remembered passive cues guide the search.
   Clunkyard robot API 2.0.0: one core under every language, the same
   answers everywhere. Facts (a call shows its arguments):
   match: mode ruleset control robots is_3d tick_rate max_ticks
     robot_id(slot)
   clock: tick seconds remaining_ticks
   arena: size gravity terrain_kind ground_height(x y)
   me: health position velocity heading pitch roll angular_velocity
     attitude grounded altitude applied
   battery: charge capacity regen sustainable_drive compute
     drive_cost(fraction)
   drive: hover_lift
   gun: aim_now cooldown_ticks spread max_power damage_scale
     can_fire(power reserve) shot_speed(power) shot_cost(power)
     reload_ticks(power) shot_damage(power) power_for_speed(speed)
     power_for_flight_time(range seconds) lead(contact power)
     lead_point(position velocity power)
   scanner: last proximity_range reach(arc) arc_for_reach(range)
   sensors: contacts nearest cues contact(slot) position_of(contact)
   tracker: cue nearest_bearing_rate velocity(slot) position(slot)
     last_seen(slot)
   nav: zone zone.margin(point) zone.contains(point margin)
     zone.distance_from(point) storm storm.margin(point)
     storm.contains(point margin) storm.distance_from(point) hill
     hill.margin(point) hill.contains(point margin)
     hill.distance_from(point) pickups nearest_pickup(kind)
     field_distance(a b)
   score: me best armour
   angles: normalize(degrees) angle_diff(target current)
     bearing_to(from to) elevation_to(from to) direction(heading pitch)
     distance(a b) clamp(value low high)
   raw: number(path) text(path) has(path) flag(path) count(path) line()
   Controls: drive.set(throttle turn pitch_turn)
     drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
     gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
     tracker.configure(smoothing forget_ticks cue_ticks)
     status.set(text value) raw.extra(json)
   Every control argument is optional; a second call keeps what the first
   set. Absent facts are your language's none. Names are snake_case here;
   your language cases them its own way. Angles are degrees from the +x
   axis; pitch is positive up.
   Manual: https://clunkyard.com/manual#sdk
*)
open Clunkyard
let free = ref false
let clamp value = Angles.clamp value (-1.) 1.
let steer towards current = clamp (Angles.angle_diff towards current /. 45.)
let on_start robot = free := Match.is_3d robot && (Arena.gravity robot).z = 0.

(* The calibration kernel, xorshift-sqrt v1; safe to delete. *)
let calibration_work work tick =
  let x = ref (Int64.logxor 88172645463325252L (Int64.of_int tick)) and sum = ref 0. in
  for _ = 1 to work do
    x := Int64.logxor !x (Int64.shift_left !x 13);
    x := Int64.logxor !x (Int64.shift_right_logical !x 7);
    x := Int64.logxor !x (Int64.shift_left !x 17);
    sum := !sum +. sqrt (Int64.to_float (Int64.shift_right_logical !x 11))
  done;
  (int_of_float (floor !sum) + Int64.to_int (Int64.shift_right_logical !x 40)) mod 1000000

let on_turn robot =
  let position=Me.position robot and heading=Me.heading robot and pitch=Me.pitch robot in
  Drive.forces robot ~brake:0.2 ();
  (match Sensors.nearest robot,Tracker.cue robot with
  | Some target,_ ->
    let throttle=if target.range>350. then 0.7 else if target.range<200. then -0.4 else 0.2 in
    Drive.set robot ~throttle ~turn:(steer target.bearing heading) ~pitch_turn:(steer target.elevation pitch) ();
    Scanner.set robot ~heading:target.bearing ~arc:20. ~pitch:target.elevation ();
    let power=Gun.power_for_flight_time robot target.range 1. in
    let h,p=match Gun.lead robot target power with None->target.bearing,target.elevation|Some aim->aim.heading,aim.pitch in
    Gun.aim robot ~heading:h ~pitch:p ();
    let now=Gun.aim_now robot in
    if abs_float(Angles.angle_diff h now.heading)<3. && abs_float(p-.now.pitch)<3. && Gun.can_fire robot power 10. then Gun.fire robot ~power ();
    Status.set robot ~text:"TRACK" ~value:target.range ()
  | None,Some cue ->
    let bearing=Angles.bearing_to position cue.position and distance=Angles.distance position cue.position in
    let elevation=if !free then Angles.elevation_to position cue.position else 0. in
    let error=min cue.bearing_error 30. and tick=Clock.tick robot in
    let across=float(tick mod 5-2)/.2.*.error and up=if !free then float((tick/5)mod 3-1)*.error/.2. else 0. in
    Drive.set robot ~throttle:0.7 ~turn:(steer bearing heading) ~pitch_turn:(steer elevation pitch) ();
    Scanner.set robot ~heading:(bearing+.across) ~arc:(Scanner.arc_for_reach robot (distance+.100.)) ~pitch:(elevation+.up) ();
    Status.set robot ~text:"CUE" ~value:distance ()
  | None,None ->
    let up=if !free then float((Clock.tick robot/10)mod 3-1)*.35. else 0. in
    Drive.set robot ~throttle:0.5 ~turn:0.33 ~pitch_turn:(steer 0. pitch) ();
    Scanner.set robot ~heading:(Angles.normalize((Scanner.last robot).heading+.36.)) ~arc:40. ~pitch:up ();
    Status.set robot ~text:"SEARCH" ());
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
  (* Only the arena's calibration sends work. *)
  match Raw.number robot "state.calibration.work" with
  | Some work when Float.is_integer work && work>0. -> Status.set robot ~text:"WORK" ~value:(float(calibration_work(int_of_float work)(Clock.tick robot))) ()
  | _ -> ()
let () = run ~on_start ~on_turn ()
