import pytest
from conftest import REPO, load_script

validator = load_script(REPO / 'tools/check_ardupilot_params.py', 'ardupilot_param_validation')


def test_parameter_formats_and_comments(tmp_path):
    path = tmp_path / 'custom.params'
    path.write_text('# comment\nFRAME_CLASS,1\nARMING_CHECK 0\nWP_SPEED, 1.2 # metres/s\n')
    assert validator.param_names(path) == ['FRAME_CLASS', 'ARMING_CHECK', 'WP_SPEED']


@pytest.mark.parametrize('line', ['ARMING_CHECK,', 'ARMING_CHECK', 'BAD-NAME 1', 'WP_SPEED,1,2'])
def test_malformed_parameter_line_fails(tmp_path, line):
    path = tmp_path / 'custom.params'
    path.write_text(line)
    with pytest.raises(ValueError, match='expected NAME'):
        validator.param_names(path)


def test_exact_names_and_vehicle_boundaries(tmp_path, monkeypatch):
    plane = tmp_path / 'ardupilot_plane.params'
    copter = tmp_path / 'arducopter.params'
    plane.write_text('ARSPD_USE 1\n')
    copter.write_text('FRAME_CLASS,1\n')
    calls = []
    def generate(checkout, vehicle):
        calls.append(vehicle)
        return {'ArduPlane': {'ARSPD_USE'}, 'ArduCopter': {'FRAME_CLASS'}}[vehicle]
    monkeypatch.setattr(validator, 'verify_checkout', lambda *args: None)
    monkeypatch.setattr(validator, 'generate_names', generate)
    assert validator.validate(tmp_path, 'pin', [plane, copter]) == 0
    assert set(calls) == {'ArduPlane', 'ArduCopter'}
    for invalid in ['FRAME_CLASS 1\n', 'MISSPELLED_USE 1\n']:
        plane.write_text(invalid)
        assert validator.validate(tmp_path, 'pin', [plane]) == 1


def test_unknown_vehicle_requires_explicit_selection():
    with pytest.raises(ValueError, match='specify --vehicle'):
        validator.vehicle_for('custom.params')
    assert validator.vehicle_for('custom.params', 'Rover') == 'Rover'


def test_checkout_revision_must_match_pin(monkeypatch):
    monkeypatch.setattr(validator.subprocess, 'check_output', lambda args, **kwargs:
                        'old\n' if args[-1] == 'HEAD' else 'pinned\n')
    with pytest.raises(ValueError, match='not at pinned ref'):
        validator.verify_checkout('/checkout', 'pin')


def test_metadata_generation_failure_is_not_partial_validation(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(validator.subprocess, 'run', lambda *args, **kwargs:
                        SimpleNamespace(returncode=1, stdout='', stderr='missing source'))
    with pytest.raises(ValueError, match='metadata generation failed'):
        validator.generate_names(tmp_path, 'ArduCopter')


def test_layered_parameter_files_resolve_to_their_firmware():
    assert validator.vehicle_for('systems/aerodrone/config/arducopter_sitl.params') == 'ArduCopter'
    assert validator.vehicle_for('arducopter_gps_input.params') == 'ArduCopter'
    assert validator.vehicle_for('applications/isaac/config/aerodrone_sitl.params') == 'ArduCopter'
    assert validator.vehicle_for('systems/boat/config/ardurover_sitl.params') == 'Rover'
    assert validator.vehicle_for('systems/submarine/config/ardusub_sitl.params') == 'ArduSub'
    with pytest.raises(ValueError, match='specify --vehicle'):
        validator.vehicle_for('arducopterish.params')


def test_hardware_baselines_keep_safety_checks_and_sitl_relaxations_are_separate():
    """Relaxed arming/failsafes live only in *_sitl.params files."""
    relaxed = {'ARMING_CHECK': '0', 'FS_THR_ENABLE': '0', 'FS_GCS_ENABLE': '0',
               'DISARM_DELAY': '0'}
    for system, firmware in (('aerodrone', 'arducopter'), ('boat', 'ardurover'),
                             ('submarine', 'ardusub')):
        baseline = REPO / f'systems/{system}/config/{firmware}.params'
        values = {}
        for line in baseline.read_text().splitlines():
            line = line.split('#', 1)[0].strip()
            if line:
                name, value = line.replace(',', ' ').split()
                values[name] = value
        assert values.get('ARMING_CHECK') == '1', baseline
        for name, value in relaxed.items():
            assert values.get(name) != value, f'{baseline}: {name}={value}'
        assert (REPO / f'systems/{system}/config/{firmware}_sitl.params').is_file()


# The aerodrone's hardware-only overlay: the printed aircraft's propulsion,
# battery and harmonic notch, loaded on the aircraft between arducopter.params
# and arducopter_gps_input.params and never by the simulators.
AERODRONE_CONFIG = REPO / 'systems/aerodrone/config'
AERODRONE_HARDWARE = AERODRONE_CONFIG / 'arducopter_hardware.params'
HARDWARE_ONLY_PREFIXES = ('BATT_', 'MOT_BAT_', 'INS_HNTCH_', 'MOT_HOVER_LEARN')
SITL_LAYERS = (AERODRONE_CONFIG / 'arducopter.params', AERODRONE_CONFIG / 'arducopter_sitl.params',
               AERODRONE_CONFIG / 'arducopter_gps_input.params', REPO / 'applications/isaac/config/aerodrone_sitl.params')


def read_values(path):
    values = {}
    for line in path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if line:
            name, value = line.replace(',', ' ').split()
            values[name] = float(value)
    return values


def aerodrone_propulsion():
    """The assembly check's propulsion model and the committed take-off masses."""
    import json
    import re
    import sys
    sys.path.insert(0, str(REPO))
    from hardware.parts.aerodrone import assembly
    report = json.loads((REPO / 'hardware/parts/aerodrone/stl/manifest.json').read_text())['assembly']
    launcher = float(re.match(r'\s*(\d+)', report['launcher_takeoff_mass_g']).group(1))
    return assembly, float(report['takeoff_mass_g']), launcher


def test_aerodrone_hardware_overlay_is_valid_and_keeps_safety_checks():
    assert validator.vehicle_for(str(AERODRONE_HARDWARE)) == 'ArduCopter'
    names = validator.param_names(AERODRONE_HARDWARE)
    assert len(names) == len(set(names))
    values = read_values(AERODRONE_HARDWARE)
    for name, value in {'ARMING_CHECK': 0, 'FS_THR_ENABLE': 0, 'FS_GCS_ENABLE': 0, 'DISARM_DELAY': 0}.items():
        assert values.get(name) != value, name
    # battery failsafes armed: RTL at the low threshold, land at the critical one
    assert values['BATT_FS_LOW_ACT'] == 2 and values['BATT_FS_CRT_ACT'] == 1


def test_simulators_never_load_the_aerodrone_hardware_overlay():
    """SITL layers only the baseline, its relaxations, the GPS-denied file and
    the backend file, and none of them sets a hardware-only parameter, so the
    overlay cannot change Gazebo's or Isaac's behaviour."""
    sources = [*REPO.glob('compositions/*.yaml'), *REPO.glob('applications/ardupilot_sitl/**/*'),
               *REPO.glob('applications/isaac/**/*.py'), *REPO.glob('applications/isaac/**/*.yaml'),
               *REPO.glob('applications/isaac/**/*.sh'), *REPO.glob('applications/gazebo/**/*.py'),
               *REPO.glob('applications/gazebo/**/*.sh'), *REPO.glob('applications/gazebo/**/*.yaml')]
    for path in sources:
        if path.is_file():
            assert 'arducopter_hardware' not in path.read_text(errors='ignore'), path
    for path in SITL_LAYERS:
        assert not [n for n in validator.param_names(path) if n.startswith(HARDWARE_ONLY_PREFIXES)], path
    assert 'arducopter_hardware.params' not in (
        REPO / 'applications/ardupilot_sitl/scripts/run_arducopter_sitl.sh').read_text()


def test_aerodrone_hardware_overlay_matches_the_propulsion():
    """Battery compensation and thresholds from the pack, MOT_THST_HOVER and the
    harmonic notch from the motor's test table at the committed take-off masses."""
    assembly, base_g, launcher_g = aerodrone_propulsion()
    v = read_values(AERODRONE_HARDWARE)
    cells = assembly.PACK_CELLS
    assert v['MOT_BAT_VOLT_MAX'] == pytest.approx(cells * assembly.CELL_V_FULL)
    assert v['MOT_BAT_VOLT_MIN'] == pytest.approx(cells * 3.3)
    assert v['BATT_CAPACITY'] == assembly.PACK_MAH
    assert v['MOT_BAT_VOLT_MIN'] <= v['BATT_CRT_VOLT'] < v['BATT_LOW_VOLT'] < v['BATT_ARM_VOLT'] < v['MOT_BAT_VOLT_MAX']
    assert 0 < v['BATT_CRT_MAH'] < v['BATT_LOW_MAH'] == pytest.approx(assembly.PACK_MAH * (1 - assembly.PACK_USABLE))
    base, launcher = assembly.flight_numbers(base_g), assembly.flight_numbers(launcher_g)
    assert launcher['tw_table'] >= assembly.TW_MIN
    assert v['MOT_THST_HOVER'] == pytest.approx(base['thst_hover'], abs=0.03)
    assert v['MOT_HOVER_LEARN'] == 2
    # throttle-based notch on the hover frequency without the launcher, which
    # follows the throttle to the launcher configuration's hover frequency
    assert v['INS_HNTCH_ENABLE'] == 1 and v['INS_HNTCH_MODE'] == 1
    assert v['INS_HNTCH_REF'] == v['MOT_THST_HOVER']
    assert 0.9 * base['hover_hz'] <= v['INS_HNTCH_FREQ'] <= base['hover_hz']
    tracked = v['INS_HNTCH_FREQ'] * (launcher['thst_hover'] / v['INS_HNTCH_REF']) ** 0.5
    assert tracked == pytest.approx(launcher['hover_hz'], rel=0.1)
    assert v['INS_HNTCH_BW'] == pytest.approx(v['INS_HNTCH_FREQ'] / 2, rel=0.1)
    assert v['INS_HNTCH_HMNCS'] == 3
