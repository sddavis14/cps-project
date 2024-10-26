#!/usr/bin/env python

import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from std_msgs.msg import Header

class SyncedCameraInfoPublisher:
    def __init__(self):
        rospy.init_node('camera_info_publisher', anonymous=True)

        # Get parameters
        self.camera_info_file = rospy.get_param('~camera_info_file', 'camera_info.yaml')
        self.camera_name = rospy.get_param('~camera_name', 'camera')
        self.compressed = rospy.get_param('~compressed', True)

        # Load camera info
        self.camera_info_msg = self.load_camera_info()

        if not self.camera_info_msg:
            rospy.logerr("Failed to load camera calibration. Shutting down.")
            rospy.signal_shutdown("No camera calibration data")
            return

        # Create publisher
        self.info_pub = rospy.Publisher(f'camera_info',
                                        CameraInfo,
                                        queue_size=10)

        # Create subscriber
        topic = f'{"compressed" if self.compressed else "image_raw"}'
        msg_type = CompressedImage if self.compressed else Image
        self.image_sub = rospy.Subscriber(topic,
                                          msg_type,
                                          self.image_callback,
                                          queue_size=10)

        rospy.loginfo(f"Initialized camera info publisher for {self.camera_name}")
        rospy.loginfo(f"Subscribed to {topic}")
        rospy.loginfo(f"Publishing to {self.camera_name}/camera_info")

    def load_camera_info(self):
        try:
            with open(self.camera_info_file, 'r') as f:
                calib_data = yaml.safe_load(f)

            camera_info_msg = CameraInfo()

            # Set image dimensions
            camera_info_msg.height = calib_data['image_height']
            camera_info_msg.width = calib_data['image_width']

            # Set distortion parameters
            camera_info_msg.distortion_model = calib_data['distortion_model']
            camera_info_msg.D = calib_data['distortion_coefficients']['data']

            # Set camera matrix (K)
            camera_info_msg.K = calib_data['camera_matrix']['data']

            # Set rectification matrix (R)
            camera_info_msg.R = calib_data['rectification_matrix']['data']

            # Set projection matrix (P)
            camera_info_msg.P = calib_data['projection_matrix']['data']

            rospy.loginfo(f"Successfully loaded calibration for {self.camera_name}")
            return camera_info_msg

        except Exception as e:
            rospy.logerr(f"Failed to load camera calibration: {str(e)}")
            return None

    def image_callback(self, image_msg):
        """Publish camera info with same timestamp as image"""
        self.camera_info_msg.header.stamp = image_msg.header.stamp
        self.camera_info_msg.header.frame_id = image_msg.header.frame_id
        self.info_pub.publish(self.camera_info_msg)

if __name__ == '__main__':
    try:
        publisher = SyncedCameraInfoPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass