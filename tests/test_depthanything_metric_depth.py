"""Depth Anything metric depth helpers and the accuracy tool
(applications/depthanything/src/depthanything_pkg/depthanything_pkg/
metric_depth.py and depth_accuracy.py), without ROS or torch; and the
model settings of every composition that runs depth_node."""
import re
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from conftest import REPO, load_script

PKG = REPO / 'applications/depthanything/src/depthanything_pkg/depthanything_pkg'
md = load_script(PKG / 'metric_depth.py', 'metric_depth')
acc = load_script(PKG / 'depth_accuracy.py', 'depth_accuracy')


def test_mask_range_is_what_the_node_publishes():
    depth = md.mask_range([[0.01, 1.0], [np.inf, 90.0]], 0.05, 80.0)
    assert depth.dtype == np.float32 and depth[0, 1] == 1.0
    assert np.isnan(depth[0, 0]) and np.isnan(depth[1, 0]) and np.isnan(depth[1, 1])


def test_depth_images_in_metres_with_nan_for_no_measurement():
    metres = md.depth_image_to_m(np.array([[0.0, 1.5], [np.nan, -1.0]], np.float32), '32FC1')
    assert metres[0, 1] == 1.5 and np.isnan(metres[[0, 1, 1], [0, 0, 1]]).all()
    millimetres = md.depth_image_to_m(np.array([[0, 1500]], np.uint16), '16UC1')
    assert np.isnan(millimetres[0, 0]) and millimetres[0, 1] == pytest.approx(1.5)
    with pytest.raises(ValueError):
        md.depth_image_to_m(np.zeros((1, 1)), 'bgr8')


def test_median_scale_uses_the_valid_sampled_reference_in_range():
    reference = np.full((40, 40), 2.0, np.float32)
    estimate = np.full((40, 40), 16.0, np.float32)
    assert md.median_scale(reference, estimate, 0.3, 4.0) == pytest.approx(0.125)
    # Out-of-range or invalid reference pixels and invalid estimates are not used.
    reference[:20] = 9.0
    estimate[:, :5] = np.nan
    assert md.median_scale(reference, estimate, 0.3, 4.0) == pytest.approx(0.125)
    # Too few samples on the grid.
    assert md.median_scale(reference, estimate, 0.3, 4.0, stride=8, min_samples=100) is None
    # A reference of another resolution is resized to the estimate.
    assert md.median_scale(np.full((20, 20), 3.0), np.full((40, 40), 1.5), 0.3, 4.0) == pytest.approx(2.0)


def test_error_accumulator_reports_scale_error_and_coverage():
    reference = np.tile(np.linspace(0.6, 4.8, 64, dtype=np.float32), (48, 1))
    reference[:, :4] = 8.0  # outside the 0.5-5 m band
    errors = md.ErrorAccumulator((0.5, 5.0), stride=1)
    estimate = reference * 2.0
    estimate[:24] = np.nan  # half the band has no estimate
    errors.add(estimate, reference)
    summary = errors.summary()
    assert summary['coverage'] == pytest.approx(0.5)
    assert summary['ratio']['median'] == pytest.approx(2.0)
    assert summary['rel_error']['median'] == pytest.approx(1.0)
    assert summary['delta_1_25'] == 0.0
    assert summary['frame_ratio'] == {'median': 2.0, 'min': 2.0, 'max': 2.0}
    assert set(summary['ratio_by_distance']) == {'0.5-1', '1-2', '2-3.5', '3.5-5'}
    perfect = md.ErrorAccumulator((0.5, 5.0), stride=1)
    perfect.add(reference, reference)
    assert perfect.summary()['abs_error_m']['p95'] == 0.0 and perfect.summary()['delta_1_25'] == 1.0
    assert md.ErrorAccumulator().summary()['coverage'] is None


def test_model_specs_default_to_the_trained_max_depth():
    assert acc.parse_model('vkitti:vits')['max_depth'] == 80.0
    assert acc.parse_model('hypersim:vits')['max_depth'] == 20.0
    assert acc.parse_model('hypersim:vits:10')['name'] == 'hypersim/vits/10'
    for bad in ('vkitti', 'other:vits', 'a::1'):
        with pytest.raises(Exception):
            acc.parse_model(bad)


def test_scorer_aligns_each_frame_and_fits_one_fixed_scale():
    args = NS(band=(0.5, 5.0), eval_stride=2, align_stride=8, align_range=(0.3, 4.0),
              align_min_samples=10)
    scorer = acc.Scorer(args)
    reference = np.tile(np.linspace(0.6, 4.8, 64, dtype=np.float32), (48, 1))
    for factor in (8.0, 10.0):  # a scale error that differs between frames
        scorer.add(reference * factor, reference)
    summary = scorer.summary()
    assert summary['raw']['ratio']['median'] == pytest.approx(9.0, rel=0.12)
    assert summary['aligned']['abs_error_m']['p95'] == pytest.approx(0.0, abs=1e-5)
    assert summary['aligned']['frames_without_scale'] == 0
    fixed = summary['fixed_scale']
    assert fixed['scale'] == pytest.approx(0.1125) and fixed['rel_error']['p95'] > 0.05


def message(encoding, pixels, bigendian=False):
    pixels = np.ascontiguousarray(pixels)
    data = pixels.astype(pixels.dtype.newbyteorder('>')).tobytes() if bigendian else pixels.tobytes()
    height, width = pixels.shape[:2]
    return NS(encoding=encoding, height=height, width=width, step=len(data) // height,
              is_bigendian=int(bigendian), data=data)


def test_image_messages_decode_to_arrays():
    bgr = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    pixels, encoding = acc.image_array(message('bgr8', bgr))
    assert encoding == 'bgr8' and (pixels == bgr).all()
    pixels, encoding = acc.image_array(message('rgb8', bgr))
    assert encoding == 'bgr8' and (pixels == bgr[..., ::-1]).all()
    depth = np.array([[0.5, np.nan], [2.0, 3.0]], np.float32)
    pixels, encoding = acc.image_array(message('32FC1', depth, bigendian=True))
    assert encoding == '32FC1' and np.array_equal(pixels, depth, equal_nan=True)
    pixels, _ = acc.image_array(message('16UC1', np.array([[1500, 0]], np.uint16)))
    assert pixels.tolist() == [[1500, 0]]


def test_frames_pair_by_stamp_once_per_period():
    pairer = acc.FramePairer(period_ns=1e9, need_estimate=True)
    assert pairer.add('color', 0, 'c0') is None
    assert pairer.add('depth', 0, 'd0') is None
    assert pairer.add('estimate', 0, 'e0') == {'color': 'c0', 'depth': 'd0', 'estimate': 'e0'}
    for kind in ('color', 'depth', 'estimate'):  # within the period
        assert pairer.add(kind, int(0.5e9), kind) is None
    pairer.add('color', int(1.2e9), 'c1')
    assert pairer.add('depth', int(1.2e9), 'd1') is None  # waits for the estimate
    assert pairer.add('estimate', int(1.2e9), 'e1')['color'] == 'c1'
    no_estimate = acc.FramePairer(period_ns=0, need_estimate=False)
    no_estimate.add('depth', 5, 'd')
    assert no_estimate.add('color', 5, 'c') == {'color': 'c', 'depth': 'd'}


def test_frames_are_kept_only_after_the_car_moved():
    assert acc.moved_enough(None, (0.0, 0.0, 0.0), 0.2, 10.0)
    assert not acc.moved_enough((0.0, 0.0, 0.0), (0.1, 0.1, 0.1), 0.2, 10.0)
    assert acc.moved_enough((0.0, 0.0, 0.0), (0.2, 0.0, 0.0), 0.2, 10.0)
    assert acc.moved_enough((0.0, 0.0, 3.1), (0.0, 0.0, -3.1 + np.radians(12)), 0.2, 10.0)


def depth_node_settings():
    """(composition, service, {parameter: value}) of every depth_node."""
    found = []
    for path in sorted((REPO / 'compositions').glob('*.yaml')):
        services = yaml.safe_load(path.read_text()).get('services', {})
        for name, service in services.items():
            command = service.get('command')
            command = ' '.join(command) if isinstance(command, list) else (command or '')
            if 'depthanything_pkg' not in command:
                continue
            params = dict(re.findall(r'(?:-p )?(\w+):=([^\s]+)', command))
            found.append((path.name, name, params))
    return found


def test_every_depth_node_uses_its_checkpoints_trained_max_depth():
    """The model's output is its sigmoid head times max_depth: another
    max_depth than the checkpoint's scales every depth."""
    settings = depth_node_settings()
    assert {c for c, _, _ in settings} >= {'racecarneo.yaml', 'racecarneo_hardware.yaml',
                                            'racecar_turret_isaac.yaml'}
    for composition, service, params in settings:
        dataset = params.get('metric_dataset', 'vkitti')
        if 'max_depth' in params:
            assert float(params['max_depth']) == md.TRAINED_MAX_DEPTH[dataset], (composition, service)


def test_the_image_fetches_every_checkpoint_it_offers():
    dockerfile = (REPO / 'applications/depthanything/Dockerfile').read_text()
    for dataset in md.TRAINED_MAX_DEPTH:
        name = f'depth_anything_v2_metric_{dataset}_vits.pth'
        assert re.search(rf'wget -O \S+/{name}\s', dockerfile), dataset
        assert re.search(rf'sha256sum -c', dockerfile.split(name, 2)[-1]), dataset
    setup = (PKG.parent / 'setup.py').read_text()
    assert 'depth_accuracy = depthanything_pkg.depth_accuracy:main' in setup


def test_scale_aligner_scales_to_the_reference_of_the_same_frame():
    aligner = md.ScaleAligner(min_m=0.3, max_m=4.0, stride=4, min_samples=10,
                              max_age_s=0.05, hold_s=1.0)
    estimate = np.full((48, 64), 16.0, np.float32)
    assert aligner.scale_for(10.0, estimate) is None  # no reference yet
    aligner.add_reference(10.0, np.full((48, 64), 2.0, np.float32))
    aligner.add_reference(10.5, np.full((48, 64), 3.0, np.float32))
    assert aligner.scale_for(10.01, estimate) == pytest.approx(0.125)
    assert aligner.scale_for(10.5, estimate) == pytest.approx(0.1875)
    # No reference near this frame: the last scale holds for hold_s ...
    assert aligner.scale_for(11.2, estimate) == pytest.approx(0.1875)
    # ... then nothing is published rather than an unscaled estimate.
    assert aligner.scale_for(11.6, estimate) is None
    # A reference without valid pixels in range does not reset the hold.
    aligner.add_reference(11.0, np.full((48, 64), np.nan, np.float32))
    assert aligner.scale_for(11.0, estimate) == pytest.approx(0.1875)
    for i in range(20):
        aligner.add_reference(20.0 + i, np.full((4, 4), 1.0, np.float32))
    assert len(aligner.references) == aligner.keep


def test_depth_node_declares_the_alignment_parameters_in_its_launch():
    launch = (PKG.parent / 'launch/depthanything.launch.py').read_text()
    node = (PKG / 'depth_node.py').read_text()
    for name in ('scale_reference_topic', 'scale_reference_min_m', 'scale_reference_max_m',
                 'scale_reference_stride', 'scale_reference_min_samples',
                 'scale_reference_max_age_s', 'scale_hold_s'):
        assert f"'{name}'" in launch and f"declare_parameter('{name}'" in node, name
    assert "'scale_reference_topic': ''" in launch  # off by default

