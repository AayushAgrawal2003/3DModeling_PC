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


ros::Publisher concat_publish;
pcl::PointCloud<pcl::PointXYZ> global_pc;

ros::Time previous;


void tfCallback(const sensor_msgs::PointCloud2::ConstPtr &new_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> transformed;
    pcl::fromROSMsg(*new_cloud, pcl_cloud);
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    //tf::StampedTransform transform;
    geometry_msgs::TransformStamped transform;
    std::string target_frame = "world";
    std::string source_frame = "camera_color_optical_frame";
    if (previous + ros::Duration(1) < ros::Time::now()) {
    
    transform = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(2.0));    
    tf::StampedTransform transform_tf;
    tf::transformStampedMsgToTF(transform, transform_tf);

    pcl_ros::transformPointCloud(pcl_cloud,transformed, transform_tf);

    global_pc += transformed;

    sensor_msgs::PointCloud2 tf_cloud_msg;
    pcl::toROSMsg(global_pc, tf_cloud_msg);

    std_msgs::Header head =  new_cloud->header;
    head.frame_id = "world";
    
    tf_cloud_msg.header = head;
    concat_publish.publish(tf_cloud_msg);

    previous = ros::Time::now();
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "concate_node");
    ros::NodeHandle nh;
    previous = ros::Time::now();
    ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("/distance_filtered", 1, tfCallback);
    concat_publish = nh.advertise<sensor_msgs::PointCloud2>("/global_pc", 1);

    ros::spin();
}
