#include "depth_image_to_point_cloud.h"
#include <opencv2/opencv.hpp>

DepthToPointCloud::DepthToPointCloud(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , it_(nh)
    , tf_listener_(tf_buffer_)
    , camera_info_received_(false)
{
    loadParameters();
    initializeSubscribers();
    initializePublishers();
}

void DepthToPointCloud::loadParameters()
{
    ros::NodeHandle pnh("~");
    camera_frame_ = pnh.param<std::string>("camera_frame", "camera_optical_frame");
    target_frame_ = pnh.param<std::string>("target_frame", "map");
}

void DepthToPointCloud::initializeSubscribers()
{
    depth_sub_ = it_.subscribe("depth_image", 1, 
        &DepthToPointCloud::depthCallback, this);
    camera_info_sub_ = nh_.subscribe("camera_info", 1, 
        &DepthToPointCloud::cameraInfoCallback, this);
}

void DepthToPointCloud::initializePublishers()
{
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
}

void DepthToPointCloud::cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    if (!camera_info_received_) {
        fx_ = info_msg->K[0];
        fy_ = info_msg->K[4];
        cx_ = info_msg->K[2];
        cy_ = info_msg->K[5];
        camera_info_received_ = true;
        camera_info_sub_.shutdown(); // Unsubscribe after receiving camera info
    }
}

void DepthToPointCloud::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    if (!camera_info_received_) {
        ROS_WARN_THROTTLE(1.0, "Waiting for camera info...");
        return;
    }

    try {
        // Convert depth image to OpenCV
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(depth_msg);
        const cv::Mat& depth_img = cv_ptr->image;

        // Create point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->header.frame_id = camera_frame_;
        cloud->header.stamp = pcl_conversions::toPCL(depth_msg->header.stamp);

        // Reserve approximate size (will be filtered later)
        cloud->reserve(depth_img.rows * depth_img.cols);

        //int count = 0;
        // Convert depth image to point cloud
        for (int v = 0; v < depth_img.rows; ++v) {
            for (int u = 0; u < depth_img.cols; ++u) {
                float depth = depth_img.at<float>(v, u);
                depth /= 2.3;
                //count++;
                if (rand() % 10 != 0) {
                    continue;
                }
                // Skip invalid measurements
                if (!std::isfinite(depth) || depth <= 0) {
                    continue;
                }

                // Calculate 3D point from depth
                pcl::PointXYZ point;
                point.z = depth;
                point.x = (u - cx_) * depth / fx_;
                point.y = (v - cy_) * depth / fy_;

                cloud->points.push_back(point);
            }
        }

        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = false;

        // Transform point cloud if needed
        if (camera_frame_ != target_frame_) {
            try {
                geometry_msgs::TransformStamped transform = 
                    tf_buffer_.lookupTransform(target_frame_, camera_frame_,
                                            ros::Time(0), ros::Duration(1.0));

                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud, cloud_msg);

                // Note: In a full implementation, you would apply the transform here
                // This would require additional PCL transformation code

            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(1.0, "Transform failed: %s", ex.what());
            }
        }

        // Convert to ROS message and publish
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header = depth_msg->header;
        output_msg.header.frame_id = camera_frame_;

        cloud_pub_.publish(output_msg);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }
}
