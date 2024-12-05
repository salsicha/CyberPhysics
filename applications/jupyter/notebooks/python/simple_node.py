#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
INPUT_TOPIC = "/local_cloud"
OUTPUT_TOPIC = "/local_cloud"
TARGET_FRAME_ID = "body"
SOURCE_FRAME_ID = "0/map"
class SimpleNode:
    def __init__(self):
        self.init_listener = True
        self.sub = rospy.Subscriber(INPUT_TOPIC, PointCloud2, self.pc2_callback)
        self.pub = rospy.Publisher(OUTPUT_TOPIC, PointCloud2, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    def transform_between_frames(self, msg):
        """Example transformation"""
        try:
            trans = self.tf_buffer.lookup_transform(TARGET_FRAME_ID, msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
            cloud_out = do_transform_cloud(msg, trans)
            self.pub.publish(cloud_out)
        except Exception as e:
            print("E: ", str(e))
    def pc2_callback(self, msg):
        """Point cloud callback"""
        self.transform_between_frames(msg)
def main():
    rospy.init_node('take_home', anonymous=True)
    take_home_test = SimpleNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    main()

