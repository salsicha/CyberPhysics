"""RACECAR Turret platform: the frame model against the contract and the
hardware composition (systems/racecar_turret/interfaces.md).

The xacro is evaluated here without xacro (CI has none): properties,
${...} expressions, $(arg ...) with the defaults, xacro:if/unless and the
include of the RACECAR's URDF. The composition is read as YAML with its
`extends` of racecarneo_hardware.yaml resolved the way docker compose does
for the keys used here.
"""
import ast
import math
import operator
import re
import shlex
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from conftest import REPO

SYSTEM = REPO / 'systems/racecar_turret'
XACRO_FILE = SYSTEM / 'urdf/racecar_turret.urdf.xacro'
CONTRACT = (SYSTEM / 'interfaces.md').read_text()
COMPOSITION = REPO / 'compositions/racecar_turret_hardware.yaml'
XACRO = '{http://www.ros.org/wiki/xacro}'
DDS = 'FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304'
JAZZY_IMAGES = {'ros2', 'pantiltros', 'realsense', 'racecarneo', 'depthanything', 'nvblox', 'nicegui'}

# --- xacro ------------------------------------------------------------------

OPERATORS = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
             ast.Div: operator.truediv, ast.USub: operator.neg, ast.UAdd: operator.pos}


def evaluate(expression, names):
    """A ${...} expression of numbers, names and + - * /."""
    def walk(node):
        if isinstance(node, ast.Expression):
            return walk(node.body)
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Name):
            value = names[node.id]
            try:
                return float(value)
            except ValueError:
                return value
        if isinstance(node, ast.BinOp):
            return OPERATORS[type(node.op)](walk(node.left), walk(node.right))
        if isinstance(node, ast.UnaryOp):
            return OPERATORS[type(node.op)](walk(node.operand))
        raise ValueError(f'unsupported xacro expression {expression!r}')
    return walk(ast.parse(expression, mode='eval'))


def substitute(text, properties, args):
    text = re.sub(r'\$\(arg (\w+)\)', lambda m: args[m.group(1)], text)
    return re.sub(r'\$\{([^}]*)\}', lambda m: str(evaluate(m.group(1), {'pi': math.pi, **properties})), text)


def expand(path, properties=None, args=None, macros=None):
    """Joints and links of a xacro file and its includes, after if/unless,
    with xacro:macro definitions and calls (params with := defaults)."""
    root = ET.parse(path).getroot()
    properties = {} if properties is None else properties
    args = {} if args is None else args
    macros = {} if macros is None else macros
    for arg in root.iter(XACRO + 'arg'):
        args.setdefault(arg.get('name'), arg.get('default'))
    joints, links = {}, set()

    def visit(element, base, props):
        for child in element:
            if child.tag == XACRO + 'include':
                included = (base.parent / child.get('filename')).resolve()
                assert included.is_file(), f'{path.name} includes missing {included}'
                sub_joints, sub_links = expand(included, props, args, macros)
                for name in sub_joints:
                    assert name not in joints, f'joint {name} defined twice'
                joints.update(sub_joints)
                links.update(sub_links)
            elif child.tag == XACRO + 'property':
                props[child.get('name')] = substitute(child.get('value'), props, args)
            elif child.tag == XACRO + 'macro':
                macros[child.get('name')] = child
            elif child.tag in (XACRO + 'if', XACRO + 'unless'):
                value = substitute(child.get('value'), props, args).strip().lower()
                if (value in ('true', '1')) == (child.tag == XACRO + 'if'):
                    visit(child, base, props)
            elif child.tag.startswith(XACRO) and child.tag[len(XACRO):] in macros:
                macro = macros[child.tag[len(XACRO):]]
                scope = dict(props)
                for param in macro.get('params').split():
                    name, _, default = param.partition(':=')
                    value = child.get(name, default if _ else None)
                    assert value is not None, f'{name} missing in the call of {macro.get("name")}'
                    scope[name] = substitute(value, props, args)
                visit(macro, base, scope)
            elif child.tag == 'link':
                links.add(child.get('name'))
            elif child.tag == 'joint':
                name = child.get('name')
                assert name not in joints, f'joint {name} defined twice'

                def numbers(tag, attribute, default):
                    found = child.find(tag)
                    text = default if found is None else found.get(attribute, default)
                    return [float(v) for v in substitute(text, props, args).split()]
                limit = child.find('limit')
                joints[name] = {
                    'type': child.get('type'),
                    'parent': substitute(child.find('parent').get('link'), props, args),
                    'child': child.find('child').get('link'),
                    'xyz': numbers('origin', 'xyz', '0 0 0'),
                    'rpy': numbers('origin', 'rpy', '0 0 0'),
                    'axis': numbers('axis', 'xyz', '1 0 0'),
                    'limit': None if limit is None else {
                        key: float(substitute(limit.get(key), props, args))
                        for key in ('lower', 'upper', 'effort', 'velocity')},
                }
    visit(root, path, properties)
    return joints, links


@pytest.fixture(scope='module')
def urdf():
    return expand(XACRO_FILE)


def rotation(rpy):
    roll, pitch, yaw = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis, angle):
    x, y, z = np.asarray(axis) / np.linalg.norm(axis)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def base_to(frame, joints, positions):
    """4x4 base_link -> frame with the given joint positions."""
    by_child = {joint['child']: (name, joint) for name, joint in joints.items()}
    transform = np.eye(4)
    while frame != 'base_link':
        name, joint = by_child[frame]
        step = np.eye(4)
        step[:3, :3] = rotation(joint['rpy'])
        step[:3, 3] = joint['xyz']
        if joint['type'] == 'revolute':
            motion = np.eye(4)
            motion[:3, :3] = axis_angle(joint['axis'], positions.get(name, 0.0))
            step = step @ motion
        elif joint['type'] == 'prismatic':
            motion = np.eye(4)
            motion[:3, 3] = np.asarray(joint['axis']) / np.linalg.norm(joint['axis']) * positions.get(name, 0.0)
            step = step @ motion
        transform = step @ transform
        frame = joint['parent']
    return transform


# Contract (interfaces.md, "Frames and geometry").
TURRET_JOINTS = {
    'turret_mount_joint': ('fixed', 'base_link', 'turret/base_link', [-0.10, 0, 0.24]),
    'pan_joint': ('revolute', 'turret/base_link', 'turret/pan_link', [0, 0, 0.045]),
    'tilt_joint': ('revolute', 'turret/pan_link', 'turret/tilt_link', [0, 0, 0.055]),
    'turret_camera_joint': ('fixed', 'turret/tilt_link', 'turret/camera_link', [0.035, 0, 0]),
    'turret_camera_optical_joint': ('fixed', 'turret/camera_link', 'turret/camera_optical_frame', [0, 0, 0]),
    # Foam dart launcher: from the printed design (hardware/parts/turret/_launcher.py).
    'turret_launcher_joint': ('fixed', 'turret/tilt_link', 'turret/launcher_link', [0, 0, 0]),
    'actuator_joint': ('prismatic', 'turret/tilt_link', 'turret/launcher_rack_link', [-0.1035, 0, 0.058]),
    'trigger_joint': ('revolute', 'turret/tilt_link', 'turret/launcher_trigger_link', [-0.109, 0.021375, 0.059]),
    'turret_muzzle_joint': ('fixed', 'turret/tilt_link', 'turret/muzzle', [0.133, 0, 0.048]),
}
LAUNCHER_LINKS = {'turret/launcher_link', 'turret/launcher_rack_link', 'turret/launcher_trigger_link', 'turret/muzzle'}
STANDALONE_FILE = SYSTEM / 'urdf/turret_printed.urdf.xacro'


def test_contract_states_the_geometry_tested_here():
    for text in ('(-0.10, 0, 0.24)', '(0, 0, 0.045)', '(0, 0, 0.055)', '(0.035, 0, 0)',
                 'axis (0, 0, -1)', 'axis (0, -1, 0)', '±1.5708 rad', '-0.65..0.75 rad',
                 'rpy (-π/2, 0, -π/2)', '`/turret/joint_states`'):
        assert text in CONTRACT, text


@pytest.mark.parametrize('name', TURRET_JOINTS)
def test_turret_joints_match_the_contract(urdf, name):
    joints, links = urdf
    kind, parent, child, xyz = TURRET_JOINTS[name]
    joint = joints[name]
    assert (joint['type'], joint['parent'], joint['child']) == (kind, parent, child)
    assert joint['xyz'] == pytest.approx(xyz)
    assert {parent, child} <= links


def test_joint_axes_and_limits(urdf):
    joints, _links = urdf
    pan, tilt = joints['pan_joint'], joints['tilt_joint']
    assert pan['axis'] == pytest.approx([0, 0, -1]) and tilt['axis'] == pytest.approx([0, -1, 0])
    assert (pan['limit']['lower'], pan['limit']['upper']) == pytest.approx((-1.5708, 1.5708))
    assert (tilt['limit']['lower'], tilt['limit']['upper']) == pytest.approx((-0.65, 0.75))
    # The tracker and the bridge clamp to the same limits.
    bridge = yaml.safe_load((REPO / 'applications/pantiltros/config/arduino_bridge.yaml').read_text())
    params = bridge['turret_arduino_bridge']['ros__parameters']
    assert (params['pan_min_rad'], params['pan_max_rad']) == pytest.approx((-1.5708, 1.5708))
    assert (params['tilt_min_rad'], params['tilt_max_rad']) == pytest.approx((-0.65, 0.75))
    # Only the turret moves: robot_state_publisher needs nothing but /turret/joint_states.
    assert {name for name, joint in joints.items() if joint['type'] != 'fixed'} == {
        'pan_joint', 'tilt_joint', 'actuator_joint', 'trigger_joint'}


def test_launcher_joints(urdf):
    joints, links = urdf
    rack, trigger = joints['actuator_joint'], joints['trigger_joint']
    # 0 = forward (rest), positive draws the rack back; the stroke primes it.
    assert rack['type'] == 'prismatic' and rack['axis'] == pytest.approx([-1, 0, 0])
    assert (rack['limit']['lower'], rack['limit']['upper']) == pytest.approx((0.0, 0.038))
    tilt = joints['tilt_joint']['limit']
    assert rack['limit']['effort'] == pytest.approx(tilt['effort'] / 0.0135)
    assert rack['limit']['velocity'] == pytest.approx(tilt['velocity'] * 0.0135)
    # 0 = safe (sear engaged), positive lifts the sear.
    assert trigger['type'] == 'revolute' and trigger['axis'] == pytest.approx([0, -1, 0])
    assert (trigger['limit']['lower'], trigger['limit']['upper']) == pytest.approx((0.0, 0.6))
    assert LAUNCHER_LINKS <= links
    # The bridge drives the rack through the same pinion.
    bridge = yaml.safe_load((REPO / 'applications/pantiltros/config/arduino_bridge.yaml').read_text())
    assert bridge['turret_arduino_bridge']['ros__parameters']['actuator_pinion_radius_m'] == pytest.approx(0.0135)


def test_the_muzzle_is_parallel_to_the_camera_above_it(urdf):
    joints, _links = urdf
    for pan, tilt in ((0.0, 0.0), (0.4, 0.5), (-1.2, -0.6)):
        positions = {'pan_joint': pan, 'tilt_joint': tilt}
        camera = base_to('turret/camera_link', joints, positions)
        muzzle = base_to('turret/muzzle', joints, positions)
        optical = base_to('turret/camera_optical_frame', joints, positions)[:3, 2]
        # x along the bore = the camera's optical axis
        assert muzzle[:3, 0] == pytest.approx(optical, abs=1e-9)
        # the bore is 48 mm above the camera's axis, on its centre line, ahead of the lens
        offset = camera[:3, :3].T @ (muzzle[:3, 3] - camera[:3, 3])
        assert offset == pytest.approx([0.098, 0.0, 0.048], abs=1e-9)
        # the rack primes backward, along the bore
        rest = base_to('turret/launcher_rack_link', joints, positions)[:3, 3]
        primed = base_to('turret/launcher_rack_link', joints, {**positions, 'actuator_joint': 0.038})[:3, 3]
        assert primed - rest == pytest.approx(-0.038 * optical, abs=1e-9)


def test_a_turret_without_the_launcher_drops_it():
    joints, links = expand(XACRO_FILE, args={'launcher': 'false'})
    assert not ({'actuator_joint', 'trigger_joint', 'turret_muzzle_joint'} & set(joints))
    assert not (LAUNCHER_LINKS & links)
    assert {name for name, joint in joints.items() if joint['type'] != 'fixed'} == {'pan_joint', 'tilt_joint'}


def test_the_standalone_printed_turret_is_the_same_turret(urdf):
    car, _ = urdf
    joints, links = expand(STANDALONE_FILE)
    turret = {name for name in car if name in TURRET_JOINTS}
    assert turret <= set(joints)
    for name in turret - {'turret_mount_joint'}:
        assert joints[name] == car[name], name
    # On the 8 mm base plate: turret/base_link (the 5 mm plate convention) 3 mm up.
    mount = joints['turret_mount_joint']
    assert mount['parent'] == 'base_link' and mount['xyz'] == pytest.approx([0, 0, 0.003])
    assert 'base_link' in links
    without, _ = expand(STANDALONE_FILE, args={'launcher': 'false'})
    assert 'actuator_joint' not in without and 'pan_joint' in without


def test_the_racecar_frames_are_included_unchanged(urdf):
    joints, links = urdf
    assert {'racecarneo/lidar_link', 'racecarneo/camera_link', 'racecarneo/imu_link'} <= links
    assert joints['lidar_joint']['xyz'] == pytest.approx([0.08, 0, 0.18])
    assert joints['camera_joint']['xyz'] == pytest.approx([0.18, 0, 0.22])
    assert joints['imu_joint']['xyz'] == pytest.approx([0, 0, 0.08])


def test_pan_right_and_tilt_up_turn_the_optical_axis_that_way(urdf):
    joints, _links = urdf
    optical = 'turret/camera_optical_frame'

    def view(pan=0.0, tilt=0.0):
        transform = base_to(optical, joints, {'pan_joint': pan, 'tilt_joint': tilt})
        return transform[:3, 2], transform[:3, 0], transform[:3, 1]     # optical z, x, y in base_link

    forward, right, down = view()
    assert forward == pytest.approx([1, 0, 0], abs=1e-9)
    assert right == pytest.approx([0, -1, 0], abs=1e-9) and down == pytest.approx([0, 0, -1], abs=1e-9)
    # +u is image right: positive pan turns the axis towards it (base -y).
    assert view(pan=0.3)[0][1] < -0.25
    # -v is image up: positive tilt turns the axis up.
    assert view(tilt=0.3)[0][2] > 0.25
    # The camera sits above the plate, which is above the lidar's scan plane.
    origin = base_to(optical, joints, {})[:3, 3]
    assert origin == pytest.approx([-0.10 + 0.035, 0, 0.24 + 0.045 + 0.055])
    assert joints['turret_mount_joint']['xyz'][2] > joints['lidar_joint']['xyz'][2] + 0.04


# --- Hardware composition ---------------------------------------------------

def merge_environment(base, local):
    def as_dict(environment):
        if isinstance(environment, dict):
            return dict(environment)
        return dict(item.split('=', 1) if '=' in item else (item, None) for item in environment or [])
    return {**as_dict(base), **as_dict(local)}


def resolve(path):
    services = yaml.safe_load(path.read_text())['services']
    resolved = {}
    for name, service in services.items():
        service = dict(service)
        extends = service.pop('extends', None)
        if extends:
            base_file = path.parent / extends['file']
            base = dict(resolve(base_file)[extends['service']])
            environment = merge_environment(base.get('environment'), service.get('environment'))
            volumes = {volume.split(':')[1]: volume for volume in base.get('volumes', [])}
            volumes.update({volume.split(':')[1]: volume for volume in service.get('volumes', [])})
            base.update(service)
            base['environment'] = environment
            base['volumes'] = list(volumes.values())
            base['extends_from'] = (extends['file'], extends['service'])
            service = base
        else:
            service['environment'] = merge_environment(None, service.get('environment'))
        resolved[name] = service
    return resolved


@pytest.fixture(scope='module')
def stack():
    return resolve(COMPOSITION)


def image_name(service):
    match = re.match(r'salsicha/cyberphysics:([a-z0-9_]+)-', service.get('image', ''))
    return match.group(1) if match else None


def command_args(service):
    return shlex.split(service['command'])


def test_every_car_service_is_reused_from_the_racecar_hardware_stack(stack):
    car = yaml.safe_load((REPO / 'compositions/racecarneo_hardware.yaml').read_text())['services']
    for name in car:
        assert stack[name]['extends_from'] == ('racecarneo_hardware.yaml', name), name
    assert set(stack) - set(car) == {
        'racecar_turret_scan_filter', 'turret_camera', 'turret_servos', 'turret_tracker'}


def test_the_scan_filter_masks_the_plate_posts_into_scan(stack):
    # The lidar driver publishes /scan_raw; SLAM, Nav2 and the collision
    # monitor read the masked /scan (interfaces.md, Topics)
    service = stack['racecar_turret_scan_filter']
    assert image_name(service) == 'racecarneo'
    assert ('../systems/racecar_turret/config/scan_filter.yaml:/workspace/scan_filter.yaml:ro'
            in service['volumes'])
    args = command_args(service)
    assert args[:3] == ['ros2', 'run', 'laser_filters']
    assert '--params-file' in args and '/workspace/scan_filter.yaml' in args
    remaps = {args[i + 1] for i, arg in enumerate(args) if arg == '-r'}
    assert remaps == {'scan:=/scan_raw', 'scan_filtered:=/scan'}
    dockerfile = (REPO / 'applications/racecarneo/Dockerfile').read_text()
    assert 'ros-${ROS_DISTRO}-laser-filters' in dockerfile


def test_jazzy_services_cap_open_files_and_share_dds_settings(stack):
    for name, service in stack.items():
        assert image_name(service) in JAZZY_IMAGES, name
        nofile = service['ulimits']['nofile']
        assert 1024 <= nofile['soft'] <= nofile['hard'] <= 1 << 20, name
        environment = service['environment']
        assert f'FASTDDS_BUILTIN_TRANSPORTS={environment["FASTDDS_BUILTIN_TRANSPORTS"]}' == DDS, name
        assert environment.get('ROS_DOMAIN_ID') == '${ROS_DOMAIN_ID:-0}', name
        assert service['network_mode'] == 'host', name


def test_robot_state_publisher_runs_the_turret_urdf_on_the_turret_joint_states(stack):
    service = stack['racecarneo_robot_state_publisher']
    assert '../systems:/workspace/systems:ro' in service['volumes']
    args = dict(arg.split(':=', 1) for arg in command_args(service) if ':=' in arg)
    launch = next(arg for arg in command_args(service) if arg.endswith('.launch.py'))
    assert launch == '/workspace/systems/racecarneo/launch/racecarneo_description.launch.py'
    assert REPO / args['urdf'].replace('/workspace/', '') == XACRO_FILE
    assert args['joint_states_topic'] == '/turret/joint_states'
    assert args['camera_optical_frames'] == 'false'       # the RealSense driver publishes them
    launch_text = (REPO / 'systems/racecarneo/launch/racecarneo_description.launch.py').read_text()
    assert "'joint_states_topic', default_value='joint_states'" in launch_text
    assert "remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))]" in launch_text


def test_the_ui_runs_with_the_turret_page_and_the_converter_with_the_mux(stack):
    ui = stack['racecarneo_nicegui']
    assert ui['environment']['UI_TURRET'] == '1'
    assert '../applications/nicegui/scripts:/workspace/scripts:ro' in ui['volumes']
    converter = stack['racecarneo_cmd_vel_to_ackermann']
    assert ('../applications/racecarneo/scripts/cmd_vel_to_ackermann.py:/workspace/cmd_vel_to_ackermann.py:ro'
            in converter['volumes'])


def test_turret_camera_publishes_the_contract_topics(stack):
    service = stack['turret_camera']
    args = command_args(service)
    assert args[:3] == ['ros2', 'run', 'usb_cam']
    params = dict(args[i + 1].split(':=', 1) for i, arg in enumerate(args) if arg == '-p')
    remaps = dict(args[i + 1].split(':=', 1) for i, arg in enumerate(args) if arg == '-r')
    # usb_cam publishes image_raw and camera_info in its namespace.
    assert remaps['__ns'] == '/turret/camera'
    assert params['frame_id'] == 'turret/camera_optical_frame'
    assert (params['image_width'], params['image_height']) == ('640', '480')
    assert float(params['framerate']) >= 15.0
    assert params['pixel_format'] == 'mjpeg2rgb'                          # rgb8 out
    assert any(device.endswith(':' + params['video_device']) for device in service['devices'])
    mount = next(v for v in service['volumes'] if v.startswith('../systems/racecar_turret/calibration:'))
    calibration_dir = mount.split(':')[1]
    calibration = params['camera_info_url'].removeprefix('file://')
    assert calibration.startswith(calibration_dir + '/')
    info = yaml.safe_load((SYSTEM / 'calibration' / calibration.rsplit('/', 1)[1]).read_text())
    assert info['camera_name'] == params['camera_name']
    assert (info['image_width'], info['image_height']) == (640, 480)
    dockerfile = (REPO / 'applications/pantiltros/Dockerfile').read_text()
    assert 'ros-${ROS_DISTRO}-usb-cam' in dockerfile


def test_turret_servos_use_the_jetson_uart_without_privileges(stack):
    service = stack['turret_servos']
    assert not service.get('privileged')
    assert not any(volume.startswith('/dev') for volume in service['volumes'])
    [env_file] = service['env_file']
    env = dict(line.split('=', 1) for line in (COMPOSITION.parent / env_file).read_text().splitlines()
               if line and not line.startswith('#'))
    assert env['TURRET_ARDUINO_TRANSPORT'] == 'sts'
    port = env['TURRET_ARDUINO_PORT']
    assert port == '/dev/ttyTHS1'
    # One variable sets the device passed in and the bridge's port.
    assert service['environment']['TURRET_ARDUINO_PORT'] == f'${{TURRET_ARDUINO_PORT:-{port}}}'
    assert service['devices'] == [f'${{TURRET_ARDUINO_PORT:-{port}}}:${{TURRET_ARDUINO_PORT:-{port}}}']
    assert 'turret_arduino_bridge.py' in service['command']
    # No microcontroller: no Arduino IDE or flashing service.
    assert not any(image_name(other) == 'arduino' for other in stack.values())


def test_tracker_runs_the_racecar_turret_profile(stack):
    service = stack['turret_tracker']
    args = command_args(service)
    params_file = args[args.index('--params-file') + 1]
    mount = next(v for v in service['volumes'] if v.startswith('../systems/racecar_turret/config:'))
    assert params_file == mount.split(':')[1] + '/turret_tracker.yaml'
    profile = SYSTEM / 'config/turret_tracker.yaml'
    if profile.exists():
        params = yaml.safe_load(profile.read_text())['pantilt_yolo_tracker']['ros__parameters']
        assert params['camera_topic'] == '/turret/camera/image_raw'
        assert params['joint_state_topic'] == '/turret/joint_states'
        model = params['model_path']
        cache = next(v for v in service['volumes'] if v.startswith('../cache/turret:')).split(':')[1]
        assert model.startswith(cache + '/')
    assert service.get('gpus') == 'all'
