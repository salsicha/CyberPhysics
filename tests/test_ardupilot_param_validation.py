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
