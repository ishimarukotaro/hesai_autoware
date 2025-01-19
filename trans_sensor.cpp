#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher points_raw_pub;
ros::Publisher velodyne_points_pub;

void sensorCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Create a new PointCloud2 message to modify
    sensor_msgs::PointCloud2 modified_msg = *msg; // Copy the original message

    modified_msg.header.stamp = ros::Time::now(); // Update the timestamp

    // Change the frame_id from "PandarXT-32" to "velodyne"
    modified_msg.header.frame_id = "velodyne";

    // Publish the modified PointCloud2 message to the /points_raw topic
    points_raw_pub.publish(modified_msg);
    
    // Publish the same modified PointCloud2 message to the /velodyne_points topic
    velodyne_points_pub.publish(modified_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "topic_relay");
    ros::NodeHandle nh;

    // Subscribe to the /hesai/pandar topic
    ros::Subscriber sub = nh.subscribe("/hesai/pandar", 1000, sensorCallback);
    
    // Advertise the /points_raw topic
    points_raw_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_raw", 1000);
    
    // Advertise the /velodyne_points topic
    velodyne_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1000);

    // Spin to keep the node running
    ros::spin();
    
    return 0;
}