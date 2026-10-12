"""The OAK-1 DepthAI pipeline (applications/oak1/scripts/oak1_pipeline.py)
built without a device: MegaDepth unchanged beside the YOLO detector, the
ObjectTracker and the MJPEG encoder, and the on-device rate limiter.

The pipeline tests need the depthai package and the Git LFS blobs (they
skip otherwise); the frame-rate and crop tests need neither.
"""
from datetime import timedelta
import json
import sys
from types import SimpleNamespace
from unittest import mock

import pytest

from conftest import REPO

SCRIPTS = REPO / 'applications/oak1/scripts'
MODELS = REPO / 'applications/oak1/models'
MEGADEPTH = MODELS / 'megadepth_192x256_openvino_2021.4_6shave.blob'
sys.path.insert(0, str(SCRIPTS))


def oak1_pipeline_module(depthai=None):
    """oak1_pipeline imported against depthai (a mock when not given)."""
    with mock.patch.dict(sys.modules, {'depthai': depthai or mock.MagicMock()}):
        sys.modules.pop('oak1_pipeline', None)
        import oak1_pipeline
        sys.modules.pop('oak1_pipeline', None)
    return oak1_pipeline


def tracking_model():
    """The tracking model shipped in models/ (export_yolo.py's json)."""
    for path in sorted(MODELS.glob('*.json')):
        meta = json.loads(path.read_text())
        if 'blob' in meta:
            return meta, MODELS / meta['blob']
    pytest.fail('no tracking model json in applications/oak1/models')


class _Stop(Exception):
    pass


def run_rate_limiter(script, stamps, commands=None):
    """Execute the device script on frames with these timestamps (s), the
    host sending commands[i] (an interval) before frame i; return the
    stamps it passes on."""
    frames = iter(stamps)
    commands = dict(commands or {})
    sent = []
    count = {'frames': 0}

    class Input:
        def get(self):
            try:
                stamp = next(frames)
            except StopIteration:
                raise _Stop
            count['frames'] += 1
            return SimpleNamespace(getTimestamp=lambda s=stamp: timedelta(seconds=s))

    class Control:
        def tryGet(self):
            period = commands.pop(count['frames'], None)
            if period is None:
                return None
            data = bytearray(repr(float(period)).encode())
            return SimpleNamespace(getData=lambda: data)

    class Output:
        def send(self, frame):
            sent.append(frame.getTimestamp().total_seconds())

    node = SimpleNamespace(io={'in': Input(), 'out': Output(), 'period': Control()})
    with pytest.raises(_Stop):
        exec(compile(script, 'rate_limit', 'exec'), {'node': node})
    return sent


def test_rate_limiter_passes_every_fourth_frame_of_20_fps_for_5_hz():
    op = oak1_pipeline_module()
    stamps = [100.0 + i / 20.0 + (0.003 if i % 3 else -0.002) for i in range(80)]
    sent = run_rate_limiter(op.rate_limit_script(5.0, 20.0), stamps)
    assert len(sent) == 20
    assert sent == pytest.approx(stamps[::4], abs=1e-6)


def test_rate_limiter_restarts_when_the_device_clock_goes_back():
    op = oak1_pipeline_module()
    stamps = [10.0, 10.05, 10.1, 10.2, 1.0, 1.05, 1.2]
    assert run_rate_limiter(op.rate_limit_script(5.0, 20.0), stamps) == pytest.approx(
        [10.0, 10.2, 1.0, 1.2], abs=1e-6)


def test_host_pauses_and_resumes_the_limiter():
    op = oak1_pipeline_module()
    assert op.limiter_period(None, 20.0) == -1.0
    assert op.limiter_period(0.0, 20.0) == 0.0
    assert op.limiter_period(25.0, 20.0) == 0.0
    assert op.limiter_period(5.0, 20.0) == pytest.approx(0.175)
    stamps = [i / 20.0 for i in range(40)]
    script = op.rate_limit_script(None, 20.0, controlled=True)  # starts paused
    sent = run_rate_limiter(script, stamps, {10: op.limiter_period(5.0, 20.0),
                                             30: op.limiter_period(None, 20.0)})
    # Frames 10..29 at 5 Hz, nothing before or after.
    assert sent == pytest.approx(stamps[10:30:4], abs=1e-6)
    sent = run_rate_limiter(op.rate_limit_script(3.0, 20.0, controlled=True), stamps,
                            {20: op.limiter_period(0.0, 20.0)})
    assert len(sent) == 3 + 20  # capped, then every frame


def test_tracking_image_is_a_centred_crop_of_the_1080p_video():
    op = oak1_pipeline_module()
    # 640x352 is the central 1920x1056 of 1920x1080 scaled by 1/3.
    assert op.tracking_video_crop(640, 352) == (0.0, 12.0, 1920.0, 1068.0)
    assert op.centred_crop((1920, 1080), 4 / 3) == (240.0, 0.0, 1680.0, 1080.0)


def test_unknown_tracker_type_is_refused():
    op = oak1_pipeline_module()
    with pytest.raises(ValueError):
        op.tracker_type('DEEP_SORT')


@pytest.fixture(scope='module')
def dai():
    return pytest.importorskip('depthai')


@pytest.fixture(scope='module')
def built(dai):
    meta, blob = tracking_model()
    for path in (MEGADEPTH, blob):
        if path.stat().st_size < 1_000_000:
            pytest.skip(f'{path.name} is a Git LFS pointer (git lfs pull)')
    op = oak1_pipeline_module(dai)
    config = op.TrackingConfig(
        blob_path=str(blob), num_classes=len(meta['classes']),
        width=meta['input_width'], height=meta['input_height'],
        labels_to_track=[0, 1, 3], rate_hz=5.0)
    pipeline, nodes = op.build_pipeline(
        MEGADEPTH, fps=20.0, megadepth_threads=2, megadepth_queue_size=1,
        megadepth_rate_hz=3.0, tracking=config)
    graph = pipeline.serializeToJson()['pipeline']
    by_id = {node_id: node for node_id, node in graph['nodes']}
    links = {(by_id[c['node1Id']]['name'], c['node1Output'],
              by_id[c['node2Id']]['name'], c['node2Input'])
             for c in graph['connections']}
    return SimpleNamespace(op=op, meta=meta, blob=blob, pipeline=pipeline,
                           nodes=nodes, by_id=by_id, links=links,
                           streams={n['properties']['streamName']
                                    for n in by_id.values()
                                    if n['name'] in ('XLinkOut', 'XLinkIn')})


def props(built, name):
    matches = [n['properties'] for n in built.by_id.values() if n['name'] == name]
    assert len(matches) == 1, name
    return matches[0]


def test_megadepth_branch_is_unchanged(built, dai):
    md = built.nodes['megadepth'].getNumInferenceThreads()
    assert md == 2
    camera = built.nodes['camera']
    assert camera.getPreviewSize() == (256, 192)
    assert camera.getStillSize() == (1280, 720)
    assert camera.getResolution() == dai.ColorCameraProperties.SensorResolution.THE_1080_P
    assert {'camera', 'depth', 'highres', 'camera_control'} <= built.streams
    assert ('NeuralNetwork', 'passthrough', 'XLinkOut', 'in') in built.links
    assert ('NeuralNetwork', 'out', 'XLinkOut', 'in') in built.links
    assert ('ColorCamera', 'still', 'XLinkOut', 'in') in built.links
    # The preview reaches MegaDepth through the rate limiter, one frame queued.
    assert ('ColorCamera', 'preview', 'Script', 'in') in built.links
    assert ('Script', 'out', 'NeuralNetwork', 'in') in built.links
    assert built.nodes['megadepth'].input.getQueueSize() == 1
    assert built.pipeline.getOpenVINOVersion() == dai.OpenVINO.VERSION_2021_4


def test_detector_tracker_and_encoder_are_wired_on_the_video(built, dai):
    assert {'tracklets', 'tracking_jpeg', 'tracking_rate', 'megadepth_rate'} <= built.streams
    assert ('XLinkIn', 'out', 'Script', 'period') in built.links
    for link in [('ColorCamera', 'video', 'Script', 'in'),
                 ('Script', 'out', 'ImageManip', 'inputImage'),
                 ('ImageManip', 'out', 'VideoEncoder', 'in'),
                 ('ImageManip', 'out', 'DetectionNetwork', 'in'),
                 ('DetectionNetwork', 'passthrough', 'ObjectTracker', 'inputTrackerFrame'),
                 ('DetectionNetwork', 'passthrough', 'ObjectTracker', 'inputDetectionFrame'),
                 ('DetectionNetwork', 'out', 'ObjectTracker', 'inputDetections'),
                 ('ObjectTracker', 'out', 'XLinkOut', 'in'),
                 ('VideoEncoder', 'out', 'XLinkOut', 'in')]:
        assert link in built.links, link
    detector = built.nodes['detector']
    assert detector.getNumClasses() == len(built.meta['classes'])
    assert detector.getCoordinateSize() == 4
    assert detector.getAnchors() == []
    assert detector.getConfidenceThreshold() == pytest.approx(0.2)
    assert detector.getNumInferenceThreads() == 1
    tracker = props(built, 'ObjectTracker')
    assert tracker['trackerIdAssignmentPolicy'] == int(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
    assert tracker['trackerType'] == int(dai.TrackerType.SHORT_TERM_IMAGELESS)
    assert tracker['detectionLabelsToTrack'] == [0, 1, 3]
    assert tracker['trackingPerClass'] is False
    encoder = built.nodes['encoder']
    assert encoder.getProfile() == dai.VideoEncoderProperties.Profile.MJPEG
    scale = built.nodes['scale'].initialConfig
    assert (scale.getResizeWidth(), scale.getResizeHeight()) == (
        built.meta['input_width'], built.meta['input_height'])
    assert scale.getCropYMin() == pytest.approx(12 / 1080)
    assert built.nodes['to_bgr'].initialConfig.getFormatConfig().type == dai.ImgFrame.Type.BGR888p


def test_blobs_fit_the_pipeline_openvino_and_the_shave_budget(built, dai):
    pipeline_version = built.pipeline.getOpenVINOVersion()
    total = 0
    for path, threads in ((MEGADEPTH, 2), (built.blob, 1)):
        blob = dai.OpenVINO.Blob(str(path))
        assert dai.OpenVINO.areVersionsBlobCompatible(pipeline_version, blob.version)
        total += blob.numShaves
    # Two networks, each blob compiled for 6 SHAVEs: they share the 12 the
    # device leaves to networks beside the camera, ImageManip and tracker.
    assert total <= 12
    yolo = dai.OpenVINO.Blob(str(built.blob))
    (_name, tensor), = yolo.networkInputs.items()
    assert tensor.dims[:2] == [built.meta['input_width'], built.meta['input_height']]
    assert sorted(yolo.networkOutputs) == sorted(built.meta['outputs'])


def test_without_tracking_only_megadepth_and_stills_are_built(dai):
    if MEGADEPTH.stat().st_size < 1_000_000:
        pytest.skip('MegaDepth blob is a Git LFS pointer')
    op = oak1_pipeline_module(dai)
    pipeline, nodes = op.build_pipeline(MEGADEPTH)
    names = sorted(n['name'] for _, n in pipeline.serializeToJson()['pipeline']['nodes'])
    assert names == ['ColorCamera', 'NeuralNetwork', 'XLinkIn',
                     'XLinkOut', 'XLinkOut', 'XLinkOut']
    assert nodes['megadepth'].getNumInferenceThreads() == 2
