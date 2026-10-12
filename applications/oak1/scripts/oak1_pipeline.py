"""DepthAI pipeline of the nadir OAK-1: MegaDepth, WildNav stills and the
object-tracking detector, all on the camera's Myriad X.

Only one process can hold the device, so megadepth_ros.py builds every
stream here into one pipeline:

- MegaDepth for DemNav: the 256x192 preview into the MegaDepth network
  (one queued frame; capped by a rate limiter while tracking runs);
  streams 'camera' (network passthrough) and 'depth'.
- WildNav stills: 'highres', triggered through 'camera_control'.
- Tracking (optional, systems/aerodrone/tracking.md): the 1080P video,
  cropped to the detector's aspect ratio and scaled to its input size,
  feeds a YOLO detection network and DepthAI's ObjectTracker (stream
  'tracklets'), and the same frames go through the on-device MJPEG encoder
  (stream 'tracking_jpeg') so the host never encodes. Tracklets and JPEG
  frames carry the sequence number of the camera frame they come from.

No ROS imports: tests build the pipeline without a device.
"""
from dataclasses import dataclass, field
from pathlib import Path

import depthai as dai

MEGADEPTH_SIZE = (256, 192)
VIDEO_SIZE = (1920, 1080)  # THE_1080_P

STREAM_CAMERA = 'camera'
STREAM_DEPTH = 'depth'
STREAM_HIGHRES = 'highres'
STREAM_CONTROL = 'camera_control'
STREAM_TRACKLETS = 'tracklets'
STREAM_JPEG = 'tracking_jpeg'
# Rate limiter control (XLinkIn): the detector's and MegaDepth's input.
STREAM_TRACKING_RATE = 'tracking_rate'
STREAM_MEGADEPTH_RATE = 'megadepth_rate'

TRACKER_TYPES = ('SHORT_TERM_IMAGELESS', 'SHORT_TERM_KCF',
                 'ZERO_TERM_IMAGELESS', 'ZERO_TERM_COLOR_HISTOGRAM')


@dataclass
class TrackingConfig:
    """The detector and tracker. Sizes are the network input (width,
    height), which is also the published tracking image."""
    blob_path: str
    num_classes: int
    width: int = 640
    height: int = 352
    confidence_threshold: float = 0.2
    iou_threshold: float = 0.5
    inference_threads: int = 1
    tracker_type: str = 'SHORT_TERM_IMAGELESS'
    labels_to_track: list = field(default_factory=list)  # empty: all
    max_objects: int = 60
    tracklet_max_lifespan: int = 30  # frames a LOST tracklet keeps its ID
    tracklet_birth_threshold: int = 3
    jpeg_quality: int = 80
    # Frames per second into the detector (0: every camera frame). The
    # detector and MegaDepth share the Myriad X's NN resources, so every
    # detector inference slows MegaDepth: cap it.
    rate_hz: float = 5.0
    # Start with the detector paused (no frames) until the host sends a
    # rate on STREAM_TRACKING_RATE.
    start_paused: bool = False


def centred_crop(source_size, aspect):
    """The largest crop of source_size (w, h) with width/height == aspect,
    centred, as (x0, y0, x1, y1) in source pixels."""
    width, height = source_size
    if width / height > aspect:
        crop_w, crop_h = height * aspect, float(height)
    else:
        crop_w, crop_h = float(width), width / aspect
    x0 = (width - crop_w) / 2.0
    y0 = (height - crop_h) / 2.0
    return (x0, y0, x0 + crop_w, y0 + crop_h)


def tracking_video_crop(width, height):
    """Crop of the 1080P video the tracking image shows (video pixels)."""
    return centred_crop(VIDEO_SIZE, width / float(height))


def tracker_type(name):
    if name not in TRACKER_TYPES:
        raise ValueError(f'tracker_type must be one of {TRACKER_TYPES}')
    return getattr(dai.TrackerType, name)


RATE_LIMIT_SCRIPT = '''
period = {period!r}
last = None
while True:
    command = node.io['period'].tryGet() if {controlled!r} else None
    if command is not None:
        period = float(bytes(command.getData()).decode())
        last = None
    frame = node.io['in'].get()
    if period < 0.0:
        continue
    stamp = frame.getTimestamp().total_seconds()
    if last is None or stamp - last >= period or stamp < last:
        last = stamp
        node.io['out'].send(frame)
'''


def limiter_period(rate_hz, camera_fps):
    """The rate limiter's minimum interval between frames for at most
    rate_hz frames per second: 0 passes every frame (rate_hz 0 or not
    below the camera's), -1 none (rate_hz None, paused). Half a camera
    frame of tolerance keeps e.g. 5 Hz from 20 fps at every fourth frame
    despite timestamp jitter."""
    if rate_hz is None:
        return -1.0
    if rate_hz <= 0.0 or rate_hz >= camera_fps:
        return 0.0
    return round(1.0 / float(rate_hz) - 0.5 / float(camera_fps), 6)


def rate_limit_script(rate_hz, camera_fps, controlled=False):
    """Device script passing at most rate_hz frames per second. Controlled,
    it takes a new interval (limiter_period, as text in a Buffer) on its
    'period' input."""
    return RATE_LIMIT_SCRIPT.format(
        period=limiter_period(rate_hz, camera_fps), controlled=bool(controlled))


def period_buffer(period):
    """A Buffer carrying a rate limiter's new interval."""
    buffer = dai.Buffer()
    buffer.setData(list(repr(float(period)).encode()))
    return buffer


def add_rate_limiter(pipeline, source, sink, rate_hz, camera_fps,
                     control_stream=None):
    """A Script node passing at most rate_hz of source's frames to sink;
    with control_stream, an XLinkIn of that name sets its interval."""
    limiter = pipeline.create(dai.node.Script)
    limiter.setProcessor(dai.ProcessorType.LEON_CSS)
    limiter.inputs['in'].setBlocking(False)
    limiter.inputs['in'].setQueueSize(1)
    limiter.setScript(rate_limit_script(rate_hz, camera_fps,
                                        controlled=bool(control_stream)))
    source.link(limiter.inputs['in'])
    limiter.outputs['out'].link(sink)
    if control_stream:
        control = pipeline.create(dai.node.XLinkIn)
        control.setStreamName(control_stream)
        limiter.inputs['period'].setBlocking(False)
        limiter.inputs['period'].setQueueSize(2)
        control.out.link(limiter.inputs['period'])
    return limiter


def add_tracking(pipeline, camera, config, camera_fps=20.0):
    """Add the detector, tracker and JPEG encoder fed by camera.video."""
    blob = Path(config.blob_path)
    if not blob.is_file():
        raise FileNotFoundError(f'Tracking blob not found: {blob}')
    if blob.stat().st_size < 1_000_000:
        raise RuntimeError(
            f'Tracking model is a Git LFS pointer, not a blob: {blob}. '
            'Run git lfs pull before building.')
    width, height = config.width, config.height
    x0, y0, x1, y1 = tracking_video_crop(width, height)

    # 1080P NV12 -> centred crop at the network's aspect -> network size.
    # The encoder takes NV12; the network takes planar BGR.
    scale = pipeline.create(dai.node.ImageManip)
    scale.initialConfig.setCropRect(
        x0 / VIDEO_SIZE[0], y0 / VIDEO_SIZE[1],
        x1 / VIDEO_SIZE[0], y1 / VIDEO_SIZE[1])
    scale.initialConfig.setResize(width, height)
    scale.initialConfig.setKeepAspectRatio(False)
    scale.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
    scale.setMaxOutputFrameSize(width * height * 3 // 2)
    scale.inputImage.setBlocking(False)
    scale.inputImage.setQueueSize(1)

    to_bgr = pipeline.create(dai.node.ImageManip)
    to_bgr.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    to_bgr.setMaxOutputFrameSize(width * height * 3)
    to_bgr.inputImage.setBlocking(False)
    to_bgr.inputImage.setQueueSize(1)

    detector = pipeline.create(dai.node.YoloDetectionNetwork)
    detector.setBlobPath(str(blob))
    detector.setNumClasses(int(config.num_classes))
    detector.setCoordinateSize(4)
    detector.setAnchors([])  # anchor-free (YOLOv8 head)
    detector.setAnchorMasks({})
    detector.setConfidenceThreshold(float(config.confidence_threshold))
    detector.setIouThreshold(float(config.iou_threshold))
    detector.setNumInferenceThreads(int(config.inference_threads))
    detector.setNumPoolFrames(4)
    detector.input.setBlocking(False)
    detector.input.setQueueSize(1)

    tracker = pipeline.create(dai.node.ObjectTracker)
    tracker.setTrackerType(tracker_type(config.tracker_type))
    tracker.setTrackerIdAssignmentPolicy(
        dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
    tracker.setMaxObjectsToTrack(int(config.max_objects))
    tracker.setTrackletMaxLifespan(int(config.tracklet_max_lifespan))
    tracker.setTrackletBirthThreshold(int(config.tracklet_birth_threshold))
    tracker.setTrackerThreshold(float(config.confidence_threshold))
    # One track across class flips (VisDrone's car/van, pedestrian/people).
    tracker.setTrackingPerClass(False)
    if config.labels_to_track:
        tracker.setDetectionLabelsToTrack(
            [int(label) for label in config.labels_to_track])

    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(
        30.0, dai.VideoEncoderProperties.Profile.MJPEG)
    encoder.setQuality(int(config.jpeg_quality))
    encoder.input.setBlocking(False)
    encoder.input.setQueueSize(2)

    tracklets_out = pipeline.create(dai.node.XLinkOut)
    tracklets_out.setStreamName(STREAM_TRACKLETS)
    jpeg_out = pipeline.create(dai.node.XLinkOut)
    jpeg_out.setStreamName(STREAM_JPEG)

    # Frames into the detector: the rate limiter, controlled by the host.
    limiter = add_rate_limiter(
        pipeline, camera.video, scale.inputImage,
        None if config.start_paused else config.rate_hz, camera_fps,
        control_stream=STREAM_TRACKING_RATE)
    scale.out.link(encoder.input)
    scale.out.link(to_bgr.inputImage)
    to_bgr.out.link(detector.input)
    # Track on the frames the detector saw: one tracklets message per
    # detection, with that frame's sequence number.
    detector.passthrough.link(tracker.inputTrackerFrame)
    detector.passthrough.link(tracker.inputDetectionFrame)
    detector.out.link(tracker.inputDetections)
    tracker.out.link(tracklets_out.input)
    encoder.out.link(jpeg_out.input)
    return {'limiter': limiter, 'scale': scale, 'to_bgr': to_bgr,
            'detector': detector, 'tracker': tracker, 'encoder': encoder}


def build_pipeline(megadepth_blob, fps=20.0, highres_size=(1280, 720),
                   megadepth_threads=2, megadepth_queue_size=None,
                   megadepth_rate_hz=0.0, tracking=None):
    """The OAK-1 pipeline: MegaDepth on the preview (megadepth_queue_size
    frames queued, DepthAI's default when None; megadepth_rate_hz caps its
    input, 0 none), the triggered stills and, with tracking (a
    TrackingConfig), the detector branch on the video output."""
    pipeline = dai.Pipeline()
    pipeline.setOpenVINOVersion(dai.OpenVINO.VERSION_2021_4)

    camera = pipeline.create(dai.node.ColorCamera)
    camera.setPreviewSize(*MEGADEPTH_SIZE)
    camera.setInterleaved(False)
    camera.setFps(float(fps))
    camera.setResolution(
        dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camera.setStillSize(int(highres_size[0]), int(highres_size[1]))

    network = pipeline.create(dai.node.NeuralNetwork)
    network.setBlobPath(str(megadepth_blob))
    network.setNumPoolFrames(4)
    network.input.setBlocking(False)
    if megadepth_queue_size:
        network.input.setQueueSize(int(megadepth_queue_size))
    network.setNumInferenceThreads(int(megadepth_threads))

    camera_out = pipeline.create(dai.node.XLinkOut)
    camera_out.setStreamName(STREAM_CAMERA)
    depth_out = pipeline.create(dai.node.XLinkOut)
    depth_out.setStreamName(STREAM_DEPTH)
    highres_out = pipeline.create(dai.node.XLinkOut)
    highres_out.setStreamName(STREAM_HIGHRES)
    control_in = pipeline.create(dai.node.XLinkIn)
    control_in.setStreamName(STREAM_CONTROL)

    if tracking is not None:
        # The host lifts the cap while the detector is paused.
        add_rate_limiter(pipeline, camera.preview, network.input,
                         megadepth_rate_hz, fps,
                         control_stream=STREAM_MEGADEPTH_RATE)
    elif megadepth_rate_hz:
        add_rate_limiter(pipeline, camera.preview, network.input,
                         megadepth_rate_hz, fps)
    else:
        camera.preview.link(network.input)
    network.passthrough.link(camera_out.input)
    network.out.link(depth_out.input)
    camera.still.link(highres_out.input)
    control_in.out.link(camera.inputControl)

    nodes = {'camera': camera, 'megadepth': network}
    if tracking is not None:
        nodes.update(add_tracking(pipeline, camera, tracking, fps))
    return pipeline, nodes
