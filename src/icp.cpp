#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <sstream>

pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;
// pcl::PointCloud<pcl::PointXYZ> reference_cloud;
int first_cloud= 0;
ros::Publisher aligned;
bool has_prev_cloud = false;
// Callback for the source point cloud
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Convert ROS message to PCL point cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *current_cloud);


    if(has_prev_cloud){
    // Perform ICP registration between current_cloud and prev_cloud
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(current_cloud);
    icp.setInputTarget(prev_cloud);

    pcl::PointCloud<pcl::PointXYZ> registered_cloud;
    icp.align(registered_cloud);

    // Get the relative transform
    Eigen::Matrix4f transform = icp.getFinalTransformation();
    prev_cloud = registered_cloud.makeShared();

    sensor_msgs::PointCloud2 registered_cloud_msg;
    pcl::toROSMsg(registered_cloud, registered_cloud_msg);
    aligned.publish(registered_cloud_msg);
    // std::stringstream string;
    // string << transform;
    // ROS_INFO("Transformation matrix:\n%s", string.str().c_str());
    }
    else{
        prev_cloud = current_cloud;
        has_prev_cloud = true;
    }
    // Print the transformation matrix

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_node");
    ros::NodeHandle nh;
    aligned = nh.advertise<sensor_msgs::PointCloud2>("/aligned", 1);
    ros::Subscriber cloud_sub = nh.subscribe("/distance_filtered", 1, cloudCallback);
    // Subscribe to your point cloud topic

    ros::spin();

    return 0;
}

