#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher filtered_cloud_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
    // Convert PointCloud2 message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*input_cloud, pcl_cloud);

    // Create a filter object to remove points outside the desired distance range
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud.makeShared());
    pass.setFilterFieldName("z");  // Assuming z-axis is the depth dimension
    pass.setFilterLimits(0.8, 1.0);  // Adjust as needed
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    pass.filter(filtered_cloud);

    // Convert the filtered PCL point cloud back to a ROS PointCloud2 message
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(filtered_cloud, filtered_cloud_msg);

    // Publish the filtered point cloud
    filtered_cloud_msg.header = input_cloud->header;
    filtered_cloud_pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_filter_node");
    ros::NodeHandle nh;

    ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("/depth_points", 1, pointCloudCallback);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/distance_filtered", 1);

    ros::spin();

    return 0;
}

