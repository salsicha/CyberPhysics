// Turret firmware settings: edit for your mechanics, then rebuild and flash.
//
// The joint convention matches the ST3215 node of pan_tilt_ros
// (applications/pantiltros/pantiltros, steps = 2048 - rad * 4096 / 2pi), so
// servos calibrated for it need no change; the joint limits are the PanTiltROS
// contract used by turret_arduino_bridge.py and the YOLO tracker.
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

static const turret::Config kTurretConfig = {
    {
        // name, servo ID (pan_tilt_ros ctrl_config.yaml), step at 0 rad,
        // direction, min rad, max rad
        {"pan_joint", 12, 2048, -1, -1.5708f, 1.5708f},
        {"tilt_joint", 13, 2048, -1, -0.65f, 0.75f},
    },
    2000,   // goal speed, steps/s (about 3.1 rad/s; the tracker limits itself to 1 rad/s)
    50,     // goal acceleration, x100 steps/s^2
    500,    // watchdog: hold when no SET arrives for 500 ms
    40,     // POS feedback every 40 ms (25 Hz)
    1000,   // repeat a persisting servo error at most once per second
    250,    // retry a failed torque enable at most every 250 ms
    false,  // STOP/watchdog hold the measured position; true: disable torque instead
};
