#include "depth_image_to_point_cloud.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_point_cloud");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    DepthToPointCloud node(nh, pnh);
    ros::spin();

    return 0;
}
