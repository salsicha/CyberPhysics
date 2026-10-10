"""applications/isaac/scripts/aerodrone_navsim.py: lock-step, camera frames
and the camera_info that describes them.

The script imports Isaac Sim and ROS only inside its functions, so these
checks load it with plain numpy (ROS stubbed where a node is built).
"""
import math
import re
import sys
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from conftest import REPO, install_ros_stubs, load_script

SCRIPT = REPO / 'applications/isaac/scripts/aerodrone_navsim.py'


@pytest.fixture(scope='module')
def navsim():
    return load_script(SCRIPT, 'isaac_aerodrone_navsim')


class FakeNode:
    def __init__(self, name):
        self.name = name

    def create_publisher(self, msg_type, topic, qos):
        return mock.MagicMock()


def test_camera_info_describes_the_rendered_camera(navsim):
    """RTX maps the horizontal aperture onto the width with square pixels and
    the optical axis through the image centre; in ROS's convention (pixel
    centres at integers) that centre is ((W - 1) / 2, (H - 1) / 2).
    Rendered frames registered against the orthophoto confirm it to within
    0.1 px; Isaac's camera_info helper published (W / 2, H / 2)."""
    fx, fy, cx, cy = navsim.camera_intrinsics()
    assert fx == fy == pytest.approx(640 * 18.2 / 21.0)
    assert (cx, cy) == (319.5, 239.5)
    # About 60 degrees across the width, like the Gazebo cameras.
    assert math.degrees(2 * math.atan(320 / fx)) == pytest.approx(60.0, abs=0.05)


def test_published_camera_info_and_images(navsim):
    with mock.patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['rosgraph_msgs.msg'] = mock.MagicMock()
        args = NS(rgb_topic='/rgb', rgb_info_topic='/rgb_info',
                  depth_topic='/depth', depth_info_topic='/depth_info')
        with mock.patch.object(sys.modules['rclpy.node'], 'Node', FakeNode):
            publisher = navsim.make_publisher(args)
        info = publisher.camera_info('stamp', 640, 480)
        fx, fy, cx, cy = navsim.camera_intrinsics()
        assert list(info.k) == [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        assert list(info.p) == [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        assert (info.width, info.height, info.header.frame_id) == (
            640, 480, 'downward_camera_optical_frame')
        assert info.header.stamp == 'stamp'

        rgba = np.zeros((480, 640, 4), np.uint8)
        rgba[..., 0], rgba[..., 3] = 7, 255
        rgb = navsim.rgb8(rgba)
        depth = navsim.depth32(np.full((480, 640), 250.0))
        assert rgb.shape == (480, 640, 3) and rgb.flags['C_CONTIGUOUS']
        assert depth.dtype == np.float32 and depth.shape == (480, 640)
        # (The stubbed message classes return one shared mock: check each
        # message before building the next.)
        msg = publisher.image('stamp', rgb, 'rgb8', 3)
        assert (msg.step, len(msg.data), msg.data[0], msg.encoding) == (1920, 1920 * 480, 7, 'rgb8')
        assert (msg.width, msg.height, msg.header.frame_id) == (640, 480, info.header.frame_id)
        msg = publisher.image('stamp', depth, '32FC1', 4)
        assert (msg.step, len(msg.data), msg.encoding) == (2560, 2560 * 480, '32FC1')
        assert np.frombuffer(bytes(msg.data), np.float32)[0] == 250.0


def test_one_physics_step_per_new_servo_frame(navsim):
    def frame(count):
        return NS(frame_count=count)

    assert not navsim.is_new_frame(None, None)
    assert navsim.is_new_frame(frame(0), None)
    assert navsim.is_new_frame(frame(8), 7)
    # ArduPilot re-sends an unanswered frame after a second: answer it again,
    # do not step the physics a second time.
    assert not navsim.is_new_frame(frame(8), 8)
    # A restarted ArduPilot counts from 0 again.
    assert navsim.is_new_frame(frame(0), 8)
    assert navsim.render_stride(1200.0, 10.0) == 120
    assert navsim.render_stride(1200.0, 7000.0) == 1


def test_physics_never_advances_inside_a_render():
    """World.step(render=True) runs rendering_dt / physics_dt physics steps
    (120 at 1200/10 Hz) without the motor forces and outside ArduPilot's
    clock. The loop steps physics once per frame and renders with
    World.render(), which does not step it."""
    source = SCRIPT.read_text(encoding='utf-8')
    main = source[source.index('def main():'):]
    assert re.findall(r'world\.step\((.*?)\)', main) == ['render=False']
    capture = source[source.index('def capture('):source.index('def build_world(')]
    assert 'world.render()' in capture and 'world.step' not in capture
    # The camera frame of a physics state is rendered and published before
    # ArduPilot receives that state, with the ground truth of the same stamp.
    loop = main[main.index('world.step(render=False)'):]
    assert loop.index('camera.capture(') < loop.index('transport.send(state)')
    assert 'publish_state(stamp,' in loop and 'publish_camera(stamp,' in loop


def test_capture_waits_for_the_frame_of_the_current_time(navsim):
    """RTX hands the annotators each frame two renders later: capture renders
    until the frame's simulation time is the current one."""
    camera = navsim.DownwardCamera.__new__(navsim.DownwardCamera)
    world = NS(current_time=0.2, renders=0)
    times = iter([0.1, 0.1, 0.2])

    def render():
        world.renders += 1

    world.render = render
    camera.time = NS(get_data=lambda: {'simulationTime': next(times)})
    camera.rgb = NS(get_data=lambda: np.zeros((480, 640, 4), np.uint8))
    camera.depth = NS(get_data=lambda: np.ones((480, 640), np.float32))
    rgb, depth = camera.capture(world, 0.5 / 1200.0)
    assert world.renders == 3
    assert rgb.shape == (480, 640, 3) and depth.shape == (480, 640)
    # A frame that never catches up is dropped, not published as current.
    camera.time = NS(get_data=lambda: {'simulationTime': 0.1})
    assert camera.capture(world, 0.5 / 1200.0) is None
    assert world.renders == 3 + navsim.MAX_RENDERS_PER_FRAME


def test_cameras_far_plane_and_terrain_texture(navsim):
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'Gf.Vec2f(0.3, 1000.0)' in source
    assert '"orthophoto.jpg"' in source and 'terrain_mesh.npz' in source
    # Lit by a sky dome: a DistantLight of a few thousand units renders the
    # terrain black in Isaac Sim 6.
    assert 'UsdLux.DomeLight.Define' in source
    assert not re.search(r'UsdLux\.DistantLight|prim_type="DistantLight"', source)
