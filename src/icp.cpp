#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;
// pcl::PointCloud<pcl::PointXYZ> reference_cloud;
int first_cloud= 0;
ros::Publisher aligned;
bool has_prev_cloud = false;
Eigen::Matrix4f initial_transform;
// Callback for the source point cloud
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Convert ROS message to PCL point cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *current_cloud);


    // Add the code to look up the local transform here

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    //tf::StampedTransform transform;
    geometry_msgs::TransformStamped transform_bot;
    std::string target_frame = "world";
    std::string source_frame = "camera_color_optical_frame";


    transform_bot = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(2.0)); 

    if(has_prev_cloud){
    // Use the transform between the base and the prev frame

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(current_cloud);
    icp.setInputTarget(prev_cloud);

    pcl::PointCloud<pcl::PointXYZ> registered_cloud;
    icp.align(registered_cloud);

    // Get the relative transform
    Eigen::Matrix4f transform = icp.getFinalTransformation();

    Eigen::Matrix4f verify = initial_transform * transform;

    prev_cloud = registered_cloud.makeShared();

    sensor_msgs::PointCloud2 registered_cloud_msg;
    pcl::toROSMsg(registered_cloud, registered_cloud_msg);
    aligned.publish(registered_cloud_msg);
    // std::stringstream string;
    // string << transform;
    // ROS_INFO("Transformation matrix:\n%s", string.str().c_str());
    }
    else{
        // Create a transform between the base and global point cloud

        Eigen::Vector3f translation(
        transform_bot.transform.translation.x,
        transform_bot.transform.translation.y,
        transform_bot.transform.translation.z);

        Eigen::Quaternionf rotation(
            transform_bot.transform.rotation.w,
            transform_bot.transform.rotation.x,
            transform_bot.transform.rotation.y,
            transform_bot.transform.rotation.z);

        // Create an Affine3f transformation
        Eigen::Affine3f affineTransformation = Eigen::Translation3f(translation) * Eigen::Affine3f(rotation);

        // Get the 4x4 matrix from the Affine3f transformation
        initial_transform = affineTransformation.matrix();

        // Convert Affine3d to Matrix4f
        // Eigen::Matrix4f init = eigenTransform.matrix().cast<float>();


        prev_cloud = current_cloud;
        has_prev_cloud = true;
    }

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

