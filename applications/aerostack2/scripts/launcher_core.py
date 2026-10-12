"""Aerodrone foam dart launcher: fire control (systems/aerodrone/tracking.md,
"Foam dart launcher"). Plain Python and NumPy; aerodrone_tracker.py is the
ROS side and runs it inside the tracker.

The launcher hangs under the hub pointing straight down, its bore the
vertical line through the hub centre and the centre of mass
(hardware/aerodrone.md, "Foam dart launcher"). It is the turrets' mechanism
(systems/racecar_turret/interfaces.md, "Foam dart launcher"): an ST3215
turning the pinion of a rack primes the plunger (``actuator_joint``, metres,
0 forward), a second ST3215 releases the sear (``trigger_joint``, radians, 0
safe), both driven by turret_arduino_bridge.py's ``sts`` transport with no
pan and tilt. Toy power, foam darts only.

Safety: objects only, never people; operator-confirmed only
-------------------------------------------------------------
- One ``{"action": "fire", "target_id": id}`` command starts at most one
  fire cycle, one dart: FireControl._fire is the only caller of
  FireCycle.start, and FireControl.handle, on the tracker's command
  callback, the only caller of _fire. Nothing else primes or fires: there is
  no automatic, timed or on-lock firing, and no option for it.
- A fire command is refused, with the reason (FireControl.gates, in the
  status's ``fire_block``), unless every gate holds: the launcher fitted,
  armed (``launcher_arm``; latched, off at start) and the bridge ready; the
  tracker tracking that target in centered mode; the target's class not a
  person's (PERSON_CLASSES) and known; no person anywhere in the camera's
  view: none in the latest detection frame, which must be recent
  (``fire_detection_timeout_s``), nor in any frame of the last
  ``fire_person_clear_s``; the target placed in the latest frames; the
  aircraft hovering (horizontal speed and tilt) above a still target, its
  height above ground within [``fire_min_agl_m``, ``fire_max_agl_m``], the
  target's estimated ground position within ``fire_max_offset_m`` of the
  bore's ground point; the launcher at rest without a fault, darts left and
  ``fire_min_interval_s`` since the last cycle.
- Those gates but the launcher's own hold from the command to the release
  of the sear, and the target's track must not restart meanwhile (the
  detector giving the target's id to another object of its kind): one
  failing aborts the cycle (a person in a frame aborts it at once, from the
  detections), and FireCycle moves the trigger only while they hold
  (``clear``). An abort before the release leaves the trigger at
  0 and the rack stopped where it is (FireCycle's fault, latched until the
  launcher is disarmed and armed again): disarm, a stop or track command, a
  pre-emption, a lost target, a person, a fault, shutdown, and the bridge's
  500 ms watchdog when the commands stop. The rack is drawn back (primed)
  only inside a fire cycle.

FireCycle below is the turret's (applications/pantiltros/scripts/
pantilt_yolo_tracker.py), its code copied unchanged (the docstrings say what
``clear`` means here): tests/test_aerodrone_launcher.py checks that the two
classes stay identical but for docstrings and runs the turret's FireCycle
tests on this copy. (The turret's module imports OpenCV, Ultralytics and rclpy at the
top and runs in another image; the aerodrone's tracker runs in the DemNav
image.)

The height window
-----------------
A foam dart (about 1 g, 12.7 mm across) falls at about 13-15 m/s at most:
fired down at up to 30 m/s it slows towards that, so a higher drop never
hits harder than the muzzle velocity, which the toy-power rule limits. The
height is limited by where the dart lands and by what the camera can check:

- the dart must land well inside the part of the ground the camera has
  just checked for people. The OAK-1 (69 x 42 deg on its 640 x 352 frame)
  sees 0.38 h to either side across the image. A 5 m/s wind drifts a dart
  about 1.3 m in its fall from 15 m and 2.3 m from 20 m (about 4.9 m from
  30 m), and dispersion of about 3 deg adds 0.05 h: 3.3 m in all from
  20 m, under half of the 7.6 m the camera covers there; from 30 m it
  would be 6.5 m of 11.3 m, and the margin keeps shrinking with height;
- the "no person in view" check must be one the detector can make: a
  person seen from above (shoulders about 0.45 m) spans about 14 px of the
  OAK-1's 640 px at 15 m and 10 px at 20 m, the small end of what the
  VisDrone-trained detector finds; higher, people fall below it and the
  check would pass on a person it cannot see;
- aim: from 20 m the drift and dispersion still keep a calm-air shot on a
  car-sized object.

Hence ``fire_max_agl_m`` 20 m. ``fire_min_agl_m`` 8 m: lower, the camera
checks less than 3 m around the landing point (0.38 x 8 m), too little
warning of someone walking in. The tracker's own minimum is 15 m
(tracking.md, "Limits"), so with the default limits the aircraft fires from
15-20 m above ground.

Other limits: ``fire_max_offset_m`` 1.0 m (the target's estimated ground
position from the bore's ground point: half a car's width, 0.9 m, plus the
estimate's spread); hovering at ``fire_max_speed_mps`` 0.5 m/s or less (a
dart keeps the aircraft's velocity: 0.3 m in a 0.7 s fall), ``fire_max_tilt_deg``
5 deg (a steady hover in a moderate wind; more is a manoeuvre, and the bore
leans 1.3 m off the vertical at 15 m: the offset gate measures the bore's
real ground point anyway), over a target moving at most
``fire_max_target_speed_mps`` 0.5 m/s; detections at most
``fire_detection_timeout_s`` 0.8 s old (four frames at the detector's 5 Hz);
no person for ``fire_person_clear_s`` 3 s (a person the detector finds in
seven frames of ten is missed for 3 s at 5 Hz about once in 10^7);
``fire_min_interval_s`` 2 s, as the turret.

Geometry
--------
Body FLU metres from the odometry's origin, the hub centre (the Navigator's
IMU is 20 mm ahead of it, not corrected): the bore is the body's -z axis
through the hub; the muzzle ``launcher_muzzle_offset_m`` (0, 0, -0.3065)
(hardware/parts/aerodrone/_frame.py, launcher_muzzle_z()); the OAK-1's lens
(0.137, 0, -0.080) (CAMERA_X, CAMERA_LENS_Z there), the tracker's
``camera_offset_m``, so the target is geolocated from the camera, not the
hub. The bore's ground point is where the bore's line from the muzzle meets
the terrain (the DEM, or flat ground at the rangefinder's height).
"""
import json
import math
from dataclasses import dataclass

import numpy as np
import tracking_core as tc
from demnav_pkg.camera_geometry import quaternion_matrix

ACTUATOR_JOINT = "actuator_joint"  # the launcher's priming rack, metres, 0 forward
TRIGGER_JOINT = "trigger_joint"  # the launcher's trigger, radians, 0 safe
# How close the measured rack and trigger must come to a goal to count as there
RACK_TOLERANCE_M = 0.002
TRIGGER_TOLERANCE_RAD = 0.05
# While the launcher is not verified at rest, commands carrying its goals go
# out at least this often (the bridge's watchdog makes it safe after 0.5 s).
LAUNCHER_KEEPALIVE_S = 0.1
# The detectors' classes of people: COCO's person, VisDrone's pedestrian
# and people (the VisDrone YOLOv8n in use maps both to person). A detection
# without a class counts as one too.
PERSON_CLASSES = frozenset({'person', 'pedestrian', 'people'})
# /tracking/command actions of the launcher (the rest are the tracker's).
ACTIONS = ('fire', 'launcher_arm', 'launcher_disarm', 'reload')
# The gates of a fire command, in the order fire_block reports them.
GATES = ('launcher', 'armed', 'ready', 'tracking', 'target', 'not_person', 'no_person',
         'target_seen', 'hover', 'height', 'on_target', 'launcher_rest', 'darts',
         'interval')
# The launcher's own: not required to hold until the release (the cycle
# itself moves the launcher off rest).
LAUNCHER_GATES = frozenset({'launcher_rest', 'darts', 'interval'})


class FireCycle:
    """The dart launcher's fire cycle (no ROS dependencies; the turret's,
    unchanged but for the docstrings): prime (the rack back to the sear,
    ``stroke_m``), chamber (the rack forward again), fire (the trigger to
    ``fire_rad``, held ``dwell_s``, then back to 0), each step verified
    against the measured rack and trigger within ``timeout_s``. The trigger
    moves only while update() is told the release is clear (``clear``: the
    turret's line of fire; here every gate that must hold until the
    release, FireControl.release_clear): chambered with it not clear, or
    not clear before the sear released, is a fault.

    start() is the only way in: goals() asks for the rack back only while
    priming (for at most timeout_s) and for the trigger off 0 only while
    firing (for at most timeout_s plus dwell_s), however late update() runs.
    Anything else asks for the trigger at 0 and the rack forward or held
    where it is (None). A cycle ends in a shot ("fired") or a fault: a step
    not reached in time, no measured position, or an abort (the trigger to
    0, the rack stopped). Faults latch until reset(); a launcher found away
    from rest with no cycle running is returned to rest under supervision.
    """

    READY = "ready"
    PRIMING = "priming"
    CHAMBERING = "chambering"
    FIRING = "firing"  # the trigger out and held
    RESETTING = "resetting"  # the trigger back to 0 after the shot
    FAULT = "fault"
    RETURNING = "returning"  # back to rest after a fault or when found away from it

    def __init__(
        self,
        stroke_m: float = 0.038,
        fire_rad: float = 0.6,
        dwell_s: float = 0.2,
        timeout_s: float = 3.0,
        rack_tolerance_m: float = RACK_TOLERANCE_M,
        trigger_tolerance_rad: float = TRIGGER_TOLERANCE_RAD,
    ) -> None:
        if not 0.0 < rack_tolerance_m < stroke_m / 2.0:
            raise ValueError("the rack tolerance must be positive and under half the stroke")
        if not 0.0 < trigger_tolerance_rad < fire_rad / 2.0:
            raise ValueError("the trigger tolerance must be positive and under half the angle")
        if not (dwell_s >= 0.0 and timeout_s > 0.0):
            raise ValueError("dwell_s must not be negative and timeout_s must be positive")
        self.stroke = float(stroke_m)
        self.fire = float(fire_rad)
        self.dwell_s = float(dwell_s)
        self.timeout_s = float(timeout_s)
        self.rack_tolerance = float(rack_tolerance_m)
        self.trigger_tolerance = float(trigger_tolerance_rad)
        self.phase = self.READY
        self.since = -math.inf  # when the phase began
        self.released = -math.inf  # when the trigger reached the fire angle
        self.target_id = ""  # the running cycle's target, "" without one
        self.reason = ""  # the latched fault

    @property
    def running(self) -> bool:
        return bool(self.target_id)

    def goals(self, now: float) -> tuple[float | None, float]:
        """(rack, trigger) to command; rack None: hold it where it is."""
        if self.phase == self.PRIMING:
            return (self.stroke if now - self.since <= self.timeout_s else None), 0.0
        if self.phase == self.FIRING and now - self.since <= self.timeout_s + self.dwell_s:
            return 0.0, self.fire
        if self.phase == self.FAULT:
            return None, 0.0
        return 0.0, 0.0

    def start(self, now: float, target_id: str) -> None:
        """Begin a cycle on target_id. Only the fire command's handler
        (FireControl._fire) calls this, after checking every gate."""
        self.phase, self.since, self.released = self.PRIMING, float(now), -math.inf
        self.target_id, self.reason = str(target_id), ""

    def abort(self, now: float, reason: str) -> tuple[str, str, str, bool] | None:
        """End a cycle in progress: before the trigger released, a fault
        (the trigger to 0, the rack stopped); after, it completes as fired.
        Returns the fault's event, if any."""
        if self.phase in (self.PRIMING, self.CHAMBERING):
            return self._fault(now, f"aborted while {self.phase} ({reason})")
        if self.phase == self.FIRING:
            if self.released > -math.inf:
                self.phase, self.since = self.RESETTING, float(now)  # the dart is gone
                return None
            return self._fault(now, f"aborted while firing ({reason}): the dart may have gone",
                               shot=True)
        return None

    def reset(self, now: float) -> bool:
        """Clear a latched fault; the launcher returns to rest under supervision."""
        if self.phase != self.FAULT:
            return False
        self.phase, self.since, self.reason = self.RETURNING, float(now), ""
        return True

    def at_rest(self, rack: float | None, trigger: float | None) -> bool:
        return (rack is not None and trigger is not None and rack <= self.rack_tolerance
                and trigger <= self.trigger_tolerance)

    def update(
        self, now: float, rack: float | None, trigger: float | None, clear: bool = True
    ) -> tuple[str, str, str, bool] | None:
        """Advance on the measured rack and trigger (None: no fresh
        measurement) and whether the line of fire is clear now. Returns
        (target_id, result, reason, shot) when a cycle ends ("fired" or
        "fault") or a fault latches (target_id "")."""
        phase = self.phase
        if phase in (self.READY, self.FAULT):
            if phase == self.READY and (
                (rack is not None and rack > 2.0 * self.rack_tolerance)
                or (trigger is not None and trigger > 2.0 * self.trigger_tolerance)
            ):
                # Away from rest with no cycle (moved by hand, or left so at
                # start-up): back to rest, verified as after a fault.
                self.phase, self.since = self.RETURNING, float(now)
            return None
        if rack is None or trigger is None:
            return self._fault(now, "no launcher position", shot=self.released > -math.inf)
        late = now - self.since > self.timeout_s
        if phase == self.PRIMING:
            if rack >= self.stroke - self.rack_tolerance:
                self.phase, self.since = self.CHAMBERING, float(now)
            elif late:
                return self._fault(now, f"priming not reached in {self.timeout_s:g} s")
        elif phase == self.CHAMBERING:
            if rack <= self.rack_tolerance:
                if not clear:
                    # The moment of firing, and the dart would hit the vehicle
                    return self._fault(now, "line of fire blocked")
                self.phase, self.since = self.FIRING, float(now)
            elif late:
                return self._fault(now, f"chambering not reached in {self.timeout_s:g} s")
        elif phase == self.FIRING:
            if self.released == -math.inf and not clear:
                return self._fault(now, "line of fire blocked while firing: the dart may have gone",
                                   shot=True)
            if self.released == -math.inf and trigger >= self.fire - self.trigger_tolerance:
                self.released = float(now)
            if self.released > -math.inf:
                if now - self.released >= self.dwell_s:
                    self.phase, self.since = self.RESETTING, float(now)
            elif late:
                return self._fault(now, f"trigger not reached in {self.timeout_s:g} s")
        elif phase == self.RESETTING:
            if trigger <= self.trigger_tolerance:
                event = (self.target_id, "fired", "", True)
                self.phase, self.target_id = self.READY, ""
                return event
            if late:
                return self._fault(now, f"trigger not back at 0 in {self.timeout_s:g} s",
                                   shot=True)
        elif phase == self.RETURNING:
            if self.at_rest(rack, trigger):
                self.phase = self.READY
            elif late:
                return self._fault(now, f"not back at rest in {self.timeout_s:g} s")
        return None

    def _fault(self, now: float, reason: str, shot: bool = False) -> tuple[str, str, str, bool]:
        event = (self.target_id, "fault", reason, shot)
        self.phase, self.since = self.FAULT, float(now)
        self.target_id, self.reason = "", reason
        return event


# --- detections and geometry --------------------------------------------------
def person_like(cls):
    """Whether a detection of class cls may be a person: a person class, or
    no class at all."""
    name = str(cls or '').strip().lower()
    return not name or name in PERSON_CLASSES


def _finite(*values):
    return all(isinstance(value, (int, float)) and not isinstance(value, bool)
               and math.isfinite(value) for value in values)


def _within(value, high, low=-math.inf):
    """value known, finite and in [low, high] (NaN and None are not)."""
    return value is not None and _finite(value) and low <= value <= high


def target_id_text(value):
    """A command's target_id as the tracker names tracks (a whole number
    is accepted), or '' when it is none."""
    if isinstance(value, bool):
        return ''
    if isinstance(value, (int, float)) and math.isfinite(value) and float(value).is_integer():
        return str(int(value))
    return value.strip() if isinstance(value, str) else ''


def tilt_deg(quaternion):
    """Angle between the body's z axis (the bore) and the vertical, deg."""
    up = float(quaternion_matrix(*quaternion)[2, 2])
    return math.degrees(math.acos(min(1.0, max(-1.0, up))))


def bore_ground_point(pose, muzzle_offset, height_fn, max_range_m=200.0):
    """Where the bore's line, from the muzzle along the body's -z axis,
    meets the ground height_fn(east, north), or None."""
    rotation = quaternion_matrix(*pose.quaternion)
    muzzle = tc.body_point(pose, muzzle_offset)
    direction = rotation @ np.array([0.0, 0.0, -1.0])
    return tc.intersect_terrain(muzzle, direction, height_fn, max_range_m)


def parse_launcher_command(text):
    """The command object of a /tracking/command message whose action is
    the launcher's (ACTIONS), else None (the tracker's, or not JSON)."""
    try:
        command = json.loads(text)
    except (TypeError, ValueError):
        return None
    if isinstance(command, dict) and isinstance(command.get('action'), str) and \
            command['action'] in ACTIONS:
        return command
    return None


@dataclass(frozen=True)
class LauncherConfig:
    """The fire control's parameters (the tracker's ROS parameters of the
    same names; module doc for the values)."""
    launcher_fitted: bool = False
    launcher_stroke_m: float = 0.038
    trigger_fire_rad: float = 0.6
    fire_dwell_s: float = 0.2
    # A step not measured within this long faults (the trigger to 0, the
    # rack stopped; latched until the launcher is disarmed and armed again).
    fire_stroke_timeout_s: float = 3.0
    fire_min_agl_m: float = 8.0
    fire_max_agl_m: float = 20.0
    fire_max_offset_m: float = 1.0
    fire_max_speed_mps: float = 0.5
    fire_max_tilt_deg: float = 5.0
    fire_max_target_speed_mps: float = 0.5
    fire_detection_timeout_s: float = 0.8
    fire_person_clear_s: float = 3.0
    fire_min_interval_s: float = 2.0
    # The clip holds six darts; reload sets the count (0 at start).
    launcher_capacity: int = 6
    launcher_darts: int = 0
    launcher_joint_state_timeout_s: float = 0.5
    launcher_ready_timeout_s: float = 1.0
    launcher_muzzle_offset_m: tuple = (0.0, 0.0, -0.3065)

    def __post_init__(self):
        numbers = {name: value for name, value in vars(self).items()
                   if name not in ('launcher_fitted', 'launcher_muzzle_offset_m')}
        if not _finite(*numbers.values()):
            raise ValueError('the launcher parameters must be finite numbers')
        if not 0.0 < self.fire_min_agl_m < self.fire_max_agl_m:
            raise ValueError('fire_min_agl_m must be positive and under fire_max_agl_m')
        for name in ('fire_max_offset_m', 'fire_max_speed_mps', 'fire_max_tilt_deg',
                     'fire_max_target_speed_mps', 'fire_detection_timeout_s',
                     'launcher_joint_state_timeout_s', 'launcher_ready_timeout_s'):
            if not numbers[name] > 0.0:
                raise ValueError(f'{name} must be positive')
        for name in ('fire_person_clear_s', 'fire_min_interval_s'):
            if numbers[name] < 0.0:
                raise ValueError(f'{name} must not be negative')
        if not 0 <= self.launcher_darts <= self.launcher_capacity:
            raise ValueError('launcher_darts must be within [0, launcher_capacity]')
        if len(self.launcher_muzzle_offset_m) != 3 or not _finite(*self.launcher_muzzle_offset_m):
            raise ValueError('launcher_muzzle_offset_m must be three finite numbers')


@dataclass
class Situation:
    """What the tracker knows now, for the gates (aerodrone_tracker
    builds it at every step; None: not known)."""
    state: str = 'idle'            # the tracker's state
    mode: str = None               # centered or slant
    target_id: str = None          # the tracked target's id
    target_class: str = None       # its kind of object, as first seen
    target_seen_s: float = None    # since it was last placed
    target_speed_mps: float = None
    agl_m: float = None            # the aircraft's height above ground
    speed_mps: float = None        # the aircraft's horizontal speed
    tilt_deg: float = None         # the bore's angle from the vertical
    offset_m: float = None         # the target's estimate from the bore's ground point
    track_started: float = None    # when the target's filter (re)started
    drone: tuple = None            # (east, north, up), navigation frame
    target: tuple = None           # the target's estimate
    bore_ground: tuple = None      # the bore's ground point


class FireControl:
    """The launcher's state, its fire command's gates and its fire cycle
    (module doc). handle() takes the launcher's commands, on_joint_state(),
    on_ready() and on_detections() its inputs; tick() advances the cycle and
    joint_goals() gives the joint command to send; status() the status
    fields. Times are seconds of the tracker's clock."""

    def __init__(self, config=None):
        self.config = config = config or LauncherConfig()
        self.cycle = FireCycle(stroke_m=config.launcher_stroke_m,
                               fire_rad=config.trigger_fire_rad, dwell_s=config.fire_dwell_s,
                               timeout_s=config.fire_stroke_timeout_s)
        self.armed = False
        self.ready = False
        self.ready_time = -math.inf
        self.measured = None           # (time, rack m, trigger rad)
        self.darts = int(config.launcher_darts)
        self.shots = 0
        self.last_cycle_end = -math.inf
        self.last_fire = None          # the last fire command's outcome
        self.last_send = -math.inf
        self.due = False               # a joint command is due now (a cycle started or stopped)
        self.notice = ('', -math.inf)
        self.situation = Situation()   # the latest, for the shot records
        self.cycle_start = None        # the situation when the cycle started
        self.records = []              # shots for the node to publish
        # The camera's view: the latest detection frame's image time and its
        # people (or unclassified objects), and when one was last seen.
        self.frame_time = None
        self.frame_persons = ()
        self.person_time = None
        self.person_label = ''

    # --- inputs ---------------------------------------------------------------
    def on_joint_state(self, stamp, rack, trigger):
        """The bridge's measured rack and trigger (stamp: the tracker's clock)."""
        if rack is not None and trigger is not None and _finite(stamp, rack, trigger):
            self.measured = (float(stamp), float(rack), float(trigger))

    def on_ready(self, now, ready):
        self.ready, self.ready_time = bool(ready), float(now)

    def on_detections(self, now, image_time, items):
        """A detection frame: its image time on the tracker's clock (None
        when the stamp cannot be placed on it: then it does not count as a
        recent frame, but its people do) and its [(track id, class)]. A
        person in it aborts a cycle in progress at once; returns the reason
        when it did."""
        persons = tuple((str(track), str(cls or '')) for track, cls in items
                        if person_like(cls))
        newer = image_time is not None and _finite(image_time) and (
            self.frame_time is None or image_time >= self.frame_time
            or self.frame_time - image_time > 10.0)  # the clock went back
        if newer:
            self.frame_time, self.frame_persons = float(image_time), persons
        if not persons:
            return ''
        seen = float(image_time) if image_time is not None and _finite(image_time) else now
        if self.person_time is None or seen >= self.person_time or \
                self.person_time - now > 10.0:
            self.person_time = seen
            self.person_label = describe(persons)
        if self.cycle.running:
            return self.abort(now, f'a person appeared ({describe(persons)})')
        return ''

    # --- state ------------------------------------------------------------------
    def position(self, now):
        """The measured (rack, trigger), if fresh."""
        if self.measured is None:
            return None
        stamp, rack, trigger = self.measured
        age = now - stamp
        return (rack, trigger) if 0.0 <= age <= self.config.launcher_joint_state_timeout_s \
            else None

    def ready_fresh(self, now):
        return self.ready and 0.0 <= now - self.ready_time <= \
            self.config.launcher_ready_timeout_s

    def state(self, now):
        """The status's ``launcher``, as the turret's."""
        if not self.config.launcher_fitted:
            return 'none'
        phase = self.cycle.phase
        if phase in (FireCycle.FAULT, FireCycle.RETURNING) or self.position(now) is None:
            return 'fault'
        if phase == FireCycle.RESETTING:
            return 'firing'
        if phase == FireCycle.READY:
            return 'ready' if self.darts > 0 else 'empty'
        return phase

    def person_block(self, now):
        """Why the camera's view is not known to be free of people now, or ''."""
        config = self.config
        age = None if self.frame_time is None else now - self.frame_time
        recent = age is not None and -0.5 <= age <= config.fire_detection_timeout_s
        if recent and self.frame_persons:
            return f'a person is in view ({describe(self.frame_persons)})'
        if self.person_time is not None and \
                -10.0 <= now - self.person_time <= config.fire_person_clear_s:  # (a reset)
            return (f'a person was in view {max(0.0, now - self.person_time):.1f} s ago '
                    f'({self.person_label})')
        if not recent:
            return ('no detections' if age is None else
                    f'no detections for {age:.1f} s') + ': cannot check for people'
        return ''

    # --- the gates ----------------------------------------------------------------
    def gates(self, now, target_id, s, launcher=True):
        """[(gate, reason)] of every gate (GATES) that does not hold for
        firing at target_id now, in order; launcher=False leaves out the
        launcher's own (LAUNCHER_GATES): what must hold until the release."""
        config = self.config
        failed = []

        def fail(name, reason):
            failed.append((name, reason))

        if not config.launcher_fitted:
            fail('launcher', 'no launcher')
        if not self.armed:
            fail('armed', 'launcher disarmed')
        if not self.ready_fresh(now):
            fail('ready', 'launcher not ready (no servo bridge, or it reports not ready)')
        if s.state != 'tracking':
            fail('tracking', f'not tracking (tracker {s.state})')
        elif s.mode != 'centered':
            fail('tracking', f'not in centered mode ({s.mode})')
        wanted = target_id_text(target_id)
        if not wanted:
            fail('target', "fire needs the tracked target's target_id")
        elif wanted != (s.target_id or ''):
            fail('target', f'{wanted} is not the tracked target ({s.target_id or "none"})')
        if not str(s.target_class or '').strip():
            fail('not_person', "the target's class is not known")
        elif person_like(s.target_class):
            fail('not_person', f'never at people: the target is a {s.target_class}')
        person = self.person_block(now)
        if person:
            fail('no_person', person)
        if not _within(s.target_seen_s, config.fire_detection_timeout_s, -0.5):
            fail('target_seen', 'the target is not in the latest frames')
        if not (_finite(s.speed_mps) and _finite(s.tilt_deg)):
            fail('hover', 'no recent odometry')
        elif not _within(s.speed_mps, config.fire_max_speed_mps):
            fail('hover', f'not hovering: {s.speed_mps:.1f} m/s '
                          f'(at most {config.fire_max_speed_mps:g})')
        elif not _within(s.tilt_deg, config.fire_max_tilt_deg):
            fail('hover', f'not hovering: tilted {s.tilt_deg:.1f} deg '
                          f'(at most {config.fire_max_tilt_deg:g})')
        elif not _within(s.target_speed_mps, config.fire_max_target_speed_mps):
            fail('hover', 'the target is moving' + (
                f' ({s.target_speed_mps:.1f} m/s)' if _finite(s.target_speed_mps) else ''))
        if not _finite(s.agl_m):
            fail('height', 'height above ground not known (no rangefinder or DEM)')
        elif not _within(s.agl_m, config.fire_max_agl_m, config.fire_min_agl_m):
            fail('height', f'{s.agl_m:.1f} m above ground, outside '
                           f'{config.fire_min_agl_m:g}-{config.fire_max_agl_m:g} m')
        if not _finite(s.offset_m):
            fail('on_target', "the target's or the bore's ground position is not known")
        elif not _within(s.offset_m, config.fire_max_offset_m):
            fail('on_target', f'the target is {s.offset_m:.1f} m from the bore\'s ground '
                              f'point (at most {config.fire_max_offset_m:g} m)')
        if launcher:
            cycle = self.cycle
            measured = self.position(now)
            if cycle.phase == FireCycle.FAULT:
                fail('launcher_rest', f'launcher fault: {cycle.reason}')
            elif cycle.running:
                fail('launcher_rest', 'a fire cycle is running')
            elif measured is None:
                fail('launcher_rest', 'no launcher position')
            elif cycle.phase != FireCycle.READY or not cycle.at_rest(*measured):
                fail('launcher_rest', 'launcher not at rest')
            if self.darts <= 0:
                fail('darts', 'no darts')
            wait = config.fire_min_interval_s - (now - self.last_cycle_end)
            if wait > 0.0:
                fail('interval', f'{wait:.1f} s to the next shot')
        return failed

    def refusal(self, now, target_id, s):
        """Why firing at target_id would be refused now; '' when every gate holds."""
        failed = self.gates(now, target_id, s)
        return failed[0][1] if failed else ''

    def cycle_gates(self, now, s):
        """[(gate, reason)] of what must hold until the release and does not,
        for the cycle in progress: the gates but the launcher's own, and the
        target's track not restarted since the command (the detector gave
        its id to another object of its kind: the tracker started again
        there)."""
        failed = self.gates(now, self.cycle.target_id, s, launcher=False)
        if self.cycle_start is not None and s.track_started != self.cycle_start.track_started:
            failed.append(('target_seen', "the target's track restarted"))
        return failed

    def abort_reason(self, now, s):
        """Why the cycle in progress must stop now, or '': a gate that must
        hold until the release does not."""
        failed = self.cycle_gates(now, s)
        return failed[0][1] if failed else ''

    def release_clear(self, now, s):
        """Whether the sear may be released now (FireCycle's ``clear``)."""
        return self.cycle.running and not self.cycle_gates(now, s)

    # --- commands -------------------------------------------------------------------
    def handle(self, command, now, s):
        """A launcher command (parse_launcher_command); returns what it did."""
        self.situation = s
        action = command.get('action')
        if action == 'fire':
            return self._fire(command, now, s)
        if action == 'launcher_arm':
            return self._arm(now)
        if action == 'launcher_disarm':
            return self.disarm(now, 'launcher disarmed')
        if action == 'reload':
            return self._reload(command, now)
        return self._notify(now, f'unknown launcher action {action!r}')

    def _fire(self, command, now, s):
        """{"action": "fire", "target_id": id}: one dart, if every gate holds."""
        target_id = command.get('target_id')
        refusal = self.refusal(now, target_id, s)
        if refusal:
            self.last_fire = {'target_id': target_id_text(target_id), 'result': 'refused',
                              'reason': refusal, 'stamp': round(now, 3)}
            return self._notify(now, f'fire refused: {refusal}')
        target_id = s.target_id
        self.cycle.start(now, target_id)
        self.due = True
        self.cycle_start = s
        self.last_fire = {'target_id': target_id, 'result': 'running', 'reason': '',
                          'stamp': round(now, 3)}
        return self._notify(now, f'firing at {target_id}')

    def _arm(self, now):
        if not self.config.launcher_fitted:
            return self._notify(now, 'arm refused: no launcher')
        self.armed = True
        if self.cycle.reset(now):
            # Disarming and arming again acknowledges a launcher fault.
            return self._notify(now, 'launcher armed; fault reset: returning it to rest')
        return self._notify(now, 'launcher armed')

    def disarm(self, now, reason):
        """Disarm (the bridge then holds the launcher stopped, the trigger at
        0); a cycle in progress stops."""
        self.abort(now, reason)
        self.armed = False
        self.due = True  # the safe command, whatever the bridge does on its emergency stop
        return self._notify(now, reason)

    def _reload(self, command, now):
        darts = command.get('darts')
        capacity = self.config.launcher_capacity
        if isinstance(darts, bool) or not isinstance(darts, int) or not 0 <= darts <= capacity:
            return self._notify(now, f'reload needs darts: a whole number from 0 to {capacity}')
        if self.cycle.running:
            return self._notify(now, 'reload refused: a fire cycle is running')
        if self.armed:
            return self._notify(now, 'reload refused: disarm the launcher first')
        self.darts = darts
        return self._notify(now, f'reloaded: {darts} darts')

    def abort(self, now, reason):
        """Stop a cycle in progress (FireCycle.abort): the trigger to 0 and
        the rack stopped before the release; returns the reason, or '' when
        none was running."""
        cycle = self.cycle
        target_id = cycle.target_id
        if not cycle.running:
            return ''
        self.due = True  # the trigger to 0 and the rack stopped now, not at the watchdog
        event = cycle.abort(now, reason)
        if event is not None:
            self._ended(now, *event)
        else:
            self._notify(now, f'{reason} after the shot at {target_id}: trigger back to 0')
        return reason

    # --- the cycle ------------------------------------------------------------------
    def tick(self, now, s):
        """Advance the fire cycle on the measured launcher: a gate that must
        hold until the release and does not stops it first."""
        self.situation = s
        cycle = self.cycle
        if cycle.running and cycle.phase in (FireCycle.PRIMING, FireCycle.CHAMBERING,
                                             FireCycle.FIRING):
            reason = self.abort_reason(now, s)
            if reason:
                self.abort(now, reason)
        measured = self.position(now)
        event = cycle.update(now, *(measured if measured is not None else (None, None)),
                             clear=self.release_clear(now, s))
        if event is not None:
            self._ended(now, *event)

    def joint_goals(self, now, s, force=False):
        """(rack, trigger) to command now, or None for no command: while the
        launcher is not verified at rest (none at rest: the bridge holds it
        there), every LAUNCHER_KEEPALIVE_S, and at once when a cycle started
        or stopped (``due``) or force. rack None: stopped where it is. The trigger leaves 0 only
        while FireCycle fires and the release is clear. Disarmed, the bridge
        holds it stopped and drops commands: only the safe command, when
        forced."""
        cycle = self.cycle
        force, self.due = force or self.due, False
        if not self.config.launcher_fitted:
            return None
        if not self.armed:
            return (None, 0.0) if force else None
        measured = self.position(now)
        if cycle.phase == FireCycle.READY or (not force and (
            (cycle.phase == FireCycle.FAULT
             and (measured is None or measured[1] <= cycle.trigger_tolerance))
            or now - self.last_send < LAUNCHER_KEEPALIVE_S
        )):
            return None
        rack, trigger = cycle.goals(now)
        if trigger > 0.0 and not self.release_clear(now, s):
            trigger = 0.0  # blocked between ticks: the next tick stops the cycle
        self.last_send = now
        return rack, trigger

    def _ended(self, now, target_id, result, reason, shot):
        if result == 'fault':
            self.due = True  # stop the rack and return the trigger at once
        if shot:
            self.darts = max(0, self.darts - 1)
            self.shots += 1
            self.records.append(self._record(now, target_id, result, reason))
        if target_id:
            self.last_cycle_end = now
            self.last_fire = {'target_id': target_id, 'result': result, 'reason': reason,
                              'stamp': round(now, 3)}
        if result == 'fault':
            self._notify(now, f'launcher fault: {reason}')
        elif result == 'fired':
            self._notify(now, f'fired at {target_id}; {self.darts} darts left')

    def _record(self, now, target_id, result, reason):
        """A shot (the <prefix>/shots message): the cycle's target, the
        aircraft, the target's estimate and the bore's ground point when the
        command was accepted and now."""
        def place(s):
            if s is None:
                return None
            return {'agl_m': _rounded(s.agl_m), 'offset_m': _rounded(s.offset_m),
                    'speed_mps': _rounded(s.speed_mps), 'tilt_deg': _rounded(s.tilt_deg),
                    'drone': _rounded(s.drone), 'target': _rounded(s.target),
                    'bore_ground': _rounded(s.bore_ground)}
        start = self.cycle_start
        return {'stamp': round(now, 3), 'shot': self.shots, 'target_id': target_id,
                'target_class': start.target_class if start else None,
                'result': result, 'reason': reason, 'darts_left': self.darts,
                'at_command': place(start), 'at_shot': place(self.situation)}

    def _notify(self, now, text):
        self.notice = (text, now)
        return text

    # --- status ---------------------------------------------------------------------
    def status(self, now, s):
        """The launcher's fields of /tracking/status (tracking.md)."""
        failed = self.gates(now, s.target_id, s)
        names = {name for name, _ in failed}
        text, noticed = self.notice
        return {
            'launcher': self.state(now),
            'launcher_armed': self.armed,
            'launcher_ready': self.ready_fresh(now),
            'darts': self.darts,
            'shots': self.shots,
            'target_class': s.target_class,
            'fire_block': failed[0][1] if failed else '',
            'fire_checks': {name: name not in names for name in GATES},
            'last_fire': None if self.last_fire is None else dict(self.last_fire),
            'agl_m': _rounded(s.agl_m),
            'bore_offset_m': _rounded(s.offset_m),
            'launcher_message': text if 0.0 <= now - noticed <= 5.0 else '',
        }


def describe(persons):
    """'#7 person', '#4 (no class)', ...: the people of a frame."""
    return ', '.join(f'#{track or "?"} {cls or "(no class)"}' for track, cls in persons)


def _rounded(value, digits=2):
    if value is None:
        return None
    if isinstance(value, (tuple, list, np.ndarray)):
        return [_rounded(item, digits) for item in value]
    return round(float(value), digits) if _finite(float(value)) else None
