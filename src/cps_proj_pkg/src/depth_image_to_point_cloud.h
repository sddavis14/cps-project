#ifndef DEPTH_TO_POINTCLOUD_HPP
#define DEPTH_TO_POINTCLOUD_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class DepthToPointCloud {
public:
    DepthToPointCloud(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~DepthToPointCloud() = default;

private:
    // ROS handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    
    // Subscribers
    image_transport::Subscriber depth_sub_;
    ros::Subscriber camera_info_sub_;
    
    // Publisher
    ros::Publisher cloud_pub_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Camera parameters
    double fx_, fy_, cx_, cy_;
    bool camera_info_received_;
    
    // Frame IDs
    std::string camera_frame_;
    std::string target_frame_;
    
    // Callbacks
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg);
    
    // Helper functions
    void initializeSubscribers();
    void initializePublishers();
    void loadParameters();
};

#endif // DEPTH_TO_POINTCLOUD_HPP