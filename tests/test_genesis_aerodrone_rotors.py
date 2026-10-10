"""Genesis aerodrone rotor model against ArduPilot's quad X motor mixing.

applications/genesis/scripts/aerodrone_navsim.py turns ArduPilot's motor
PWM into the cf2x body wrench; these checks use ArduPilot's own frame
definition (AP_MotorsMatrix, FRAME_CLASS 1 / FRAME_TYPE 1) and thrust
linearisation (AP_Motors_Thrust_Linearization) as the reference.
"""
import importlib.util
import math
import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARAMS = ROOT / 'applications/genesis/config/aerodrone_sitl.params'

# AP_MotorsMatrix quad X: (angle clockwise from the nose, yaw factor), the
# yaw factor +1 for CCW propellers.
ARDUPILOT_QUAD_X = ((45.0, 1.0), (-135.0, 1.0), (-45.0, -1.0), (135.0, -1.0))


def load_navsim():
    from conftest import install_ros_stubs
    with mock.patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('rosgraph_msgs', 'rosgraph_msgs.msg'):
            sys.modules[name] = mock.MagicMock()
        spec = importlib.util.spec_from_file_location(
            'genesis_aerodrone_rotors', ROOT / 'applications/genesis/scripts/aerodrone_navsim.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module


def read_params(path):
    values = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        line = line.split('#', 1)[0].strip()
        if line:
            name, value = line.split(',')
            values[name] = float(value)
    return values


def ardupilot_pwm(thrust, params, density_ratio=1.0):
    """ArduPilot's PWM for a normalised thrust (thrust_to_actuator with
    altitude compensation, no battery compensation)."""
    expo, spin_min, spin_max = (params[name] for name in (
        'MOT_THST_EXPO', 'MOT_SPIN_MIN', 'MOT_SPIN_MAX'))
    thrust = np.asarray(thrust, dtype=np.float64) / density_ratio
    ratio = ((expo - 1.0) + np.sqrt((1.0 - expo) ** 2 + 4.0 * expo * thrust)) / (2.0 * expo)
    return 1000.0 + 1000.0 * (spin_min + (spin_max - spin_min) * ratio)


class GenesisRotorModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.navsim = load_navsim()
        cls.params = read_params(PARAMS)
        cls.max_thrust = cls.navsim.CF2X_KF * 21700.0 ** 2

    def motor_thrusts(self, motor_thrusts):
        """cf2x propeller thrusts for ArduPilot motor 1-4 thrusts (N)."""
        fraction = np.asarray(motor_thrusts) / self.max_thrust
        return self.navsim.rotor_thrusts(ardupilot_pwm(fraction, self.params), self.max_thrust)

    def test_each_propeller_runs_the_ardupilot_motor_at_its_corner(self):
        self.assertEqual(self.navsim.MOTOR_ORDER, (0, 3, 1, 2))
        for propeller, motor in enumerate(self.navsim.MOTOR_ORDER):
            angle = math.radians(ARDUPILOT_QUAD_X[motor][0])
            ardupilot_flu = np.array([math.cos(angle), -math.sin(angle)])
            with self.subTest(propeller=propeller, motor=motor + 1):
                np.testing.assert_array_equal(
                    np.sign(self.navsim.CF2X_ROTOR_POSITIONS_FLU[propeller]),
                    np.sign(ardupilot_flu))
                # CCW propellers react clockwise: -z in FLU.
                self.assertEqual(self.navsim.ROTOR_YAW_SIGN[propeller],
                                 -ARDUPILOT_QUAD_X[motor][1])
                pwm = np.full(4, 1000.0)
                pwm[motor] = 2000.0
                thrusts = self.navsim.rotor_thrusts(pwm, self.max_thrust)
                self.assertAlmostEqual(thrusts[propeller], self.max_thrust)
                self.assertEqual(np.count_nonzero(thrusts), 1)

    def test_arm_geometry_and_single_rotor_moments(self):
        positions = self.navsim.CF2X_ROTOR_POSITIONS_FLU
        # cf2x.urdf: arm="0.0397", hubs on the body diagonals.
        np.testing.assert_allclose(np.hypot(*positions.T), 0.0397, rtol=0.01)
        ratio = self.navsim.CF2X_KM / self.navsim.CF2X_KF
        for propeller in range(4):
            thrusts = np.zeros(4)
            thrusts[propeller] = 0.1
            force, torque = self.navsim.rotor_wrench(thrusts)
            x, y = positions[propeller]
            with self.subTest(propeller=propeller):
                np.testing.assert_allclose(force, [0.0, 0.0, 0.1])
                np.testing.assert_allclose(torque, [
                    0.1 * y, -0.1 * x, 0.1 * ratio * self.navsim.ROTOR_YAW_SIGN[propeller]])

    def test_ardupilot_roll_pitch_yaw_mixing_turns_the_body_the_right_way(self):
        hover = np.full(4, 0.4 * self.max_thrust)
        delta = 0.05 * self.max_thrust
        for axis, factor, expected in (
                # FRD roll right and pitch up are +x and -y in FLU; yaw
                # right (clockwise seen from above) is -z.
                ('roll', lambda angle, yaw: math.cos(math.radians(angle + 90.0)), (1, 0, 0)),
                ('pitch', lambda angle, yaw: math.cos(math.radians(angle)), (0, -1, 0)),
                ('yaw', lambda angle, yaw: yaw, (0, 0, -1))):
            command = hover + delta * np.array([factor(*motor) for motor in ARDUPILOT_QUAD_X])
            force, torque = self.navsim.rotor_wrench(self.motor_thrusts(command))
            with self.subTest(axis=axis):
                np.testing.assert_allclose(force[2], hover.sum(), rtol=1e-9)
                np.testing.assert_allclose(np.sign(np.round(torque, 9)), expected)

    def test_yaw_moment_opposes_the_ccw_propellers(self):
        # ArduPilot motors 1 and 2 (cf2x propellers 0 and 2) spin CCW.
        _, torque = self.navsim.rotor_wrench([0.1, 0.0, 0.1, 0.0])
        self.assertAlmostEqual(torque[0], 0.0)
        self.assertAlmostEqual(torque[1], 0.0)
        self.assertAlmostEqual(
            torque[2], -0.2 * self.navsim.CF2X_KM / self.navsim.CF2X_KF)

    def test_params_pin_the_simulated_motor_model(self):
        for name in ('MOT_THST_EXPO', 'MOT_SPIN_MIN', 'MOT_SPIN_MAX'):
            with self.subTest(name=name):
                self.assertEqual(self.params[name], getattr(self.navsim, name))
        self.assertEqual(self.params['MOT_BAT_VOLT_MAX'], 0.0)

    def test_hover_thrust_carries_the_mass_at_any_altitude(self):
        weight = self.navsim.CF2X_MASS_KG * self.navsim.GRAVITY
        hover = self.params['MOT_THST_HOVER']
        for altitude in (0.0, 781.0, 1100.0):
            density = self.navsim.air_density_ratio(altitude)
            pwm = ardupilot_pwm(np.full(4, hover), self.params, density)
            thrust = self.navsim.rotor_thrusts(pwm, self.max_thrust, density).sum()
            with self.subTest(altitude=altitude):
                self.assertAlmostEqual(thrust / weight, 1.0, delta=0.005)
        self.assertAlmostEqual(self.navsim.air_density_ratio(1000.0), 0.9075, places=3)

    def test_idle_and_saturated_outputs(self):
        thrusts = self.navsim.rotor_thrusts([1000, 1100, 1150, 2000], self.max_thrust)
        np.testing.assert_allclose(thrusts, [0.0, self.max_thrust, 0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
