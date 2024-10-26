#!/usr/bin/env python

import rospy
import yaml
import tf2_ros
import geometry_msgs.msg
from math import pi
import tf.transformations

class TransformPublisher:
    def __init__(self):
        rospy.init_node('static_transform_publisher')

        # Get parameters
        self.yaml_file = rospy.get_param('~transform_file', 'transforms.yaml')
        self.publish_rate = rospy.get_param('~rate', 10.0)  # Hz

        # Create broadcaster
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Load and publish transforms
        self.transforms = self.load_transforms()
        self.publish_transforms()

        rospy.loginfo("Transform publisher initialized")

    def load_transforms(self):
        try:
            with open(self.yaml_file, 'r') as f:
                data = yaml.safe_load(f)
                return data
        except Exception as e:
            rospy.logerr(f"Failed to load transforms: {str(e)}")
            return None

    def create_transform_stamped(self, parent_frame, child_frame, pose_data):
        transform = geometry_msgs.msg.TransformStamped()

        # Set header
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        # Set translation
        transform.transform.translation.x = pose_data['pose']['x']
        transform.transform.translation.y = pose_data['pose']['y']
        transform.transform.translation.z = pose_data['pose']['z']

        # Convert RPY to quaternion
        q = tf.transformations.quaternion_from_euler(
            pose_data['pose'].get('roll', 0),
            pose_data['pose'].get('pitch', 0),
            pose_data['pose'].get('yaw', 0)
        )

        # Set rotation
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        return transform

    def publish_transforms(self):
        if not self.transforms:
            rospy.logerr("No transforms to publish")
            return

        transform_msgs = []

        # Create transform messages based on the naming convention in YAML
        transform_mappings = {
            'imu_link_to_lidar': ('imu_link', 'lidar'),
            'lidar_to_imu_sensor_frame': ('lidar', 'imu_sensor_frame'),
            'imu_sensor_frame_to_cam0_sensor_frame': ('imu_sensor_frame', 'cam0_sensor_frame'),
            'imu_sensor_frame_to_cam1_sensor_frame': ('imu_sensor_frame', 'cam1_sensor_frame'),
            'imu_sensor_frame_to_cam2_sensor_frame': ('imu_sensor_frame', 'cam2_sensor_frame'),
            'imu_sensor_frame_to_cam3_sensor_frame': ('imu_sensor_frame', 'cam3_sensor_frame'),
            'imu_sensor_frame_to_cam4_sensor_frame': ('imu_sensor_frame', 'cam4_sensor_frame'),
            'imu_sensor_frame_to_cam5_sensor_frame': ('imu_sensor_frame', 'cam5_sensor_frame'),
            'imu_sensor_frame_to_cam6_sensor_frame': ('imu_sensor_frame', 'cam6_sensor_frame'),
        }

        for transform_name, (parent, child) in transform_mappings.items():
            if transform_name in self.transforms:
                transform_msg = self.create_transform_stamped(
                    parent,
                    child,
                    self.transforms[transform_name]
                )
                transform_msgs.append(transform_msg)
                rospy.loginfo(f"Publishing transform from {parent} to {child}")

        # Publish all transforms
        self.broadcaster.sendTransform(transform_msgs)

if __name__ == '__main__':
    try:
        publisher = TransformPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass