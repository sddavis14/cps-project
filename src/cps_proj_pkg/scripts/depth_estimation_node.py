#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from depth_anything_v2.dpt import DepthAnythingV2
import torch
#from PIL import Image

class DepthEstimator:
    def __init__(self):
        rospy.init_node('depth_estimator', anonymous=True)

        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth')
        self.frame_id = rospy.get_param('~frame_id', 'camera_optical_frame')
        self.model_path = rospy.get_param('~model_path')

        self.frame_count = 0
        self.depth_anything = None
        self.DEVICE = None

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize your depth estimation model here
        self.setup_model()

        # Publishers and Subscribers
        self.depth_pub = rospy.Publisher(self.depth_topic, Image, queue_size=5)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=5)

        rospy.loginfo("Depth estimator initialized")
        rospy.loginfo(f"Subscribing to: {self.image_topic}")
        rospy.loginfo(f"Publishing to: {self.depth_topic}")



    def setup_model(self):
        """
        Initialize your depth estimation model here.
        For example:
        - Load neural network weights
        - Set up any preprocessing parameters
        - Initialize GPU context if needed
        """
        try:
            self.DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

            model_configs = {
                'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
                'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
                'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
                'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
            }

            self.depth_anything = DepthAnythingV2(**{**model_configs['vits'], 'max_depth': 20})
            self.depth_anything.load_state_dict(torch.load(self.model_path, map_location='cpu'))
            self.depth_anything = self.depth_anything.to(self.DEVICE).eval()

            pass
        except Exception as e:
            rospy.logerr(f"Failed to initialize depth model: {str(e)}")
            rospy.signal_shutdown("Model initialization failed")

    def preprocess_image(self, cv_image):
        """
        Preprocess the image before depth estimation.
        Args:
            cv_image: OpenCV image (BGR format)
        Returns:
            preprocessed_image: Image ready for depth estimation
        """
        try:
            # TODO: Add your preprocessing steps
            # Example preprocessing:
            # - Resize image
            # - Normalize values
            # - Convert color space if needed
            # - Create tensor if using deep learning
            return cv_image
        except Exception as e:
            rospy.logerr(f"Preprocessing failed: {str(e)}")
            return None

    def estimate_depth(self, preprocessed_image):
        """
        Estimate depth from preprocessed image.
        Args:
            preprocessed_image: Preprocessed image ready for depth estimation
        Returns:
            depth_map: Depth map as numpy array (float32)
        """
        try:
            # TODO: Add your depth estimation code
            # Example structure:
            # - Run inference on your model
            # - Post-process the output
            # - Convert to metric depth values

            if self.depth_anything is None:
                return None

            pred = self.depth_anything.infer_image(preprocessed_image, 518)

            return pred

        except Exception as e:
            rospy.logerr(f"Depth estimation failed: {str(e)}")
            return None

    def postprocess_depth(self, depth_map):
        """
        Postprocess the depth map.
        Args:
            depth_map: Raw depth map from estimation
        Returns:
            processed_depth: Processed depth map
        """
        try:
            # TODO: Add your postprocessing steps
            # Example processing:
            # - Apply filters
            # - Remove outliers
            # - Scale values
            # - Fill holes
            return depth_map
        except Exception as e:
            rospy.logerr(f"Postprocessing failed: {str(e)}")
            return None

    def image_callback(self, msg):
        """
        Process incoming images and publish depth maps.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.frame_count += 1
            if self.frame_count % 2 == 0:
                return

            # Preprocess image
            preprocessed = self.preprocess_image(cv_image)
            if preprocessed is None:
                return

            # Estimate depth
            depth_map = self.estimate_depth(preprocessed)
            if depth_map is None:
                return

            # Postprocess depth map
            processed_depth = self.postprocess_depth(depth_map)
            if processed_depth is None:
                return

            # Create depth message
            depth_msg = self.bridge.cv2_to_imgmsg(processed_depth, encoding="32FC1")
            depth_msg.header = msg.header
            depth_msg.header.frame_id = self.frame_id

            # Publish depth message
            self.depth_pub.publish(depth_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

if __name__ == '__main__':
    try:
        estimator = DepthEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
