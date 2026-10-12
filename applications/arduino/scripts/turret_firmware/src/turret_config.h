// Turret firmware settings: edit for your mechanics, then rebuild and flash.
//
// The joint convention matches the ST3215 node of pan_tilt_ros
// (applications/pantiltros/pantiltros, steps = 2048 - rad * 4096 / 2pi), so
// servos calibrated for it need no change; the joint limits are the PanTiltROS
// contract used by turret_arduino_bridge.py and the YOLO tracker. The sts
// transport of turret_arduino_bridge.py repeats these settings in
// applications/pantiltros/config/arduino_bridge.yaml.
#pragma once

#include "turret_core.h"

// Servo bus: the STS3215/ST3215 factory baud rate is 1 Mbps.
#ifndef TURRET_SERVO_BAUD
#define TURRET_SERVO_BAUD 1000000
#endif

// Direction (transmit enable) pin of a half-duplex adapter that needs one,
// driven HIGH while transmitting; -1 for adapters that switch direction
// themselves. A one-wire adapter that echoes transmitted bytes back to RX is
// handled either way.
#ifndef TURRET_SERVO_DIR_PIN
#define TURRET_SERVO_DIR_PIN -1
#endif

// Time to wait for a servo's status packet
#ifndef TURRET_SERVO_REPLY_TIMEOUT_US
#define TURRET_SERVO_REPLY_TIMEOUT_US 3000
#endif

// The foam dart launcher (systems/racecar_turret/interfaces.md, "Foam dart
// launcher"): a third ST3215 (ID 14) turning the pinion of the printed
// priming rack, and a fourth servo (ID 15) on the trigger (the sear's
// release). 1 drives both (SET's actuator_joint and trigger_joint, POS
// reports them); 0 for a turret without one, which then ignores them with an
// error.
#ifndef TURRET_ACTUATOR_FITTED
#define TURRET_ACTUATOR_FITTED 1
#endif

static const turret::Config kTurretConfig = {
    {
        // name, servo ID (pan_tilt_ros ctrl_config.yaml), step at 0, direction,
        // lower limit, upper limit (rad), pinion radius (0: revolute), torque
        // limit (0: the servo's own)
        {"pan_joint", 12, 2048, -1, -1.5708f, 1.5708f, 0.0f, 0},
        {"tilt_joint", 13, 2048, -1, -0.65f, 0.75f, 0.0f, 0},
        // The priming rack, in metres of rack travel (hardware/parts/turret/
        // _launcher.py): 0 forward (rest), 0.038 m back primes it (the sear
        // catches from 0.03425 m). Pitch radius 13.5 mm: the design's module
        // 1.5, 18-tooth pinion, so 38 mm is 2.8 rad (1835 steps). The step at
        // 0 and the direction are placeholders: set them at bring-up (rack
        // forward against the sear housing, 0.5 mm back from there). Torque
        // limited to 55 %: priming needs about 0.9-1.05 N m of the servo's
        // 2.9 N m, and the limit also caps the stall current.
        {"actuator_joint", 14, 512, 1, 0.0f, 0.038f, 0.0135f, 550},
        // The trigger: 0 safe (sear engaged), 0.6 rad lifts the sear 5.4 mm
        // (4.45 mm releases it). Torque limited to 30 %: with the safety
        // catch on it stalls against it. Step at 0 and direction:
        // placeholders, set at bring-up.
        {"trigger_joint", 15, 2048, 1, 0.0f, 0.6f, 0.0f, 300},
    },
    2000,   // goal speed, steps/s (about 3.1 rad/s; the tracker limits itself to 1 rad/s)
    50,     // goal acceleration, x100 steps/s^2
    500,    // watchdog: hold (trigger to 0, rack stopped) when no SET arrives for 500 ms
    40,     // POS feedback every 40 ms (25 Hz)
    1000,   // repeat a persisting servo error at most once per second
    250,    // retry a failed torque enable at most every 250 ms
    false,  // STOP/watchdog hold pan/tilt's measured position; true: disable their torque instead
    TURRET_ACTUATOR_FITTED != 0,
};
