import image_geometry

from rosbags.image import message_to_cvimage


class ImageMixIn():
    """
    Manipulate images stored in ros messages
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def image_ros2cv(self, image_msg, color_space=None):
        """Convert an image from a ros message to a cv2 image

        :param image_msg: ROS message of type sensor_msgs/Image
        :param color_space: Color space of output image.

        :return: numpy.array: an OpenCV image
        """
        return message_to_cvimage(image_msg, color_space)

    @staticmethod
    def rectify(camera_info_msg, cv_img):
        """
        Rectify a cv2 image using the pinhole model
        and taking the parameters from CameraInfo in the ros image message

        :param camera_info_msg: ROS message of type sensor_msgs/CameraInfo
                            containing calibration info of the camera that shoot the image
        :param cv_img: The cv2 image converted from the ros image message

        :return: numpy.array: colorized cv2 image
        """
        camera_models = image_geometry.PinholeCameraModel()
        camera_models.fromCameraInfo(camera_info_msg)
        camera_models.rectifyImage(cv_img, cv_img)
        return cv_img

    def camera_image(self, image_msg, color_space=None, rectify=False, camera_info_msg=None):
        """
        Extracts the image from a ROS message of type sensor_msgs/Image and transforms it in a cv2 image

        :param image_msg: str: ROS message of type sensor_msgs/Image
        :param color_space: Color space of output image.
        :param rectify: boolean: True if we want to rectify the image. Default False
        :param camera_info_msg: str: ROS message of type sensor_msgs/CameraInfo
                            containing calibration info of the camera that shoot the image.
                            It is only necessary to rectify the image (when rectify=True).

        :return: numpy.array: precessed cv2 image
        """
        cv_img = self.image_ros2cv(image_msg, color_space)
        if rectify:
            cv_img = self.rectify(camera_info_msg, cv_img)
        return cv_img
