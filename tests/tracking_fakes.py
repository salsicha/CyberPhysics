"""Plain-Python stand-ins for the ROS messages of the tracking contract
(vision_msgs, sensor_msgs, std_msgs), with the fields and nesting of ROS 2
Humble's (vision_msgs 4.x: bbox.center.position), so message building is
tested without ROS."""
import types


class _Msg:
    _fields = {}

    def __init__(self, **values):
        for name, factory in self._fields.items():
            setattr(self, name, factory())
        for name, value in values.items():
            setattr(self, name, value)


class Time(_Msg):
    _fields = {'sec': int, 'nanosec': int}


class Header(_Msg):
    _fields = {'stamp': Time, 'frame_id': str}


class Point2D(_Msg):
    _fields = {'x': float, 'y': float}


class Pose2D(_Msg):
    _fields = {'position': Point2D, 'theta': float}


class BoundingBox2D(_Msg):
    _fields = {'center': Pose2D, 'size_x': float, 'size_y': float}


class ObjectHypothesis(_Msg):
    _fields = {'class_id': str, 'score': float}


class ObjectHypothesisWithPose(_Msg):
    _fields = {'hypothesis': ObjectHypothesis, 'pose': object}


class Detection2D(_Msg):
    _fields = {'header': Header, 'results': list, 'bbox': BoundingBox2D,
               'id': str}


class Detection2DArray(_Msg):
    _fields = {'header': Header, 'detections': list}


class CompressedImage(_Msg):
    _fields = {'header': Header, 'format': str, 'data': bytes}


class Image(_Msg):
    _fields = {'header': Header, 'height': int, 'width': int,
               'encoding': str, 'data': bytes}


class CameraInfo(_Msg):
    _fields = {'header': Header, 'height': int, 'width': int,
               'distortion_model': str, 'd': list,
               'k': lambda: [0.0] * 9, 'r': lambda: [0.0] * 9,
               'p': lambda: [0.0] * 12}


def message_modules():
    """sys.modules entries for patch.dict."""
    vision = types.ModuleType('vision_msgs')
    vision_msg = types.ModuleType('vision_msgs.msg')
    for cls in (Point2D, Pose2D, BoundingBox2D, ObjectHypothesis,
                ObjectHypothesisWithPose, Detection2D, Detection2DArray):
        setattr(vision_msg, cls.__name__, cls)
    vision.msg = vision_msg
    sensor = types.ModuleType('sensor_msgs')
    sensor_msg = types.ModuleType('sensor_msgs.msg')
    for cls in (CompressedImage, Image, CameraInfo):
        setattr(sensor_msg, cls.__name__, cls)
    sensor.msg = sensor_msg
    std = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    std_msg.Header = Header
    std.msg = std_msg
    return {'vision_msgs': vision, 'vision_msgs.msg': vision_msg,
            'sensor_msgs': sensor, 'sensor_msgs.msg': sensor_msg,
            'std_msgs': std, 'std_msgs.msg': std_msg}
