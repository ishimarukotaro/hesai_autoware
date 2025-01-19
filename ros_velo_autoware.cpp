#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For quaternion operations
#include <tf2/LinearMath/Matrix3x3.h> // For converting quaternion to roll, pitch, yaw

class ShipVelocityNode {
public:
    ShipVelocityNode() {
        // Initialize the publisher and subscriber
        velocity_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("ndt_velo", 10);
        pose_sub = nh.subscribe("/ndt_pose", 10, &ShipVelocityNode::poseCallback, this);
        
        previous_time = ros::Time(0);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ros::Time current_time = msg->header.stamp;

        // Skip the first message to avoid uninitialized previous values
        if (previous_time.isZero()) {
            previous_time = current_time;
            previous_pose = *msg;
            return;
        }

        double dt = (current_time - previous_time).toSec();
        double dx = msg->pose.position.x - previous_pose.pose.position.x;
        double dy = msg->pose.position.y - previous_pose.pose.position.y;
        double dz = msg->pose.position.z - previous_pose.pose.position.z;

        // Calculate linear velocity
        geometry_msgs::TwistWithCovarianceStamped velocity_msg;
        velocity_msg.header.stamp = current_time;
        velocity_msg.twist.twist.linear.x = dx / dt;
        velocity_msg.twist.twist.linear.y = dy / dt;
        velocity_msg.twist.twist.linear.z = dz / dt;

        // Calculate roll, pitch, and yaw angles from quaternions
        double previous_roll, previous_pitch, previous_yaw;
        double current_roll, current_pitch, current_yaw;

        tf2::Matrix3x3(tf2::Quaternion(previous_pose.pose.orientation.x,
                                        previous_pose.pose.orientation.y,
                                        previous_pose.pose.orientation.z,
                                        previous_pose.pose.orientation.w)).getRPY(previous_roll, previous_pitch, previous_yaw);

        tf2::Matrix3x3(tf2::Quaternion(msg->pose.orientation.x,
                                        msg->pose.orientation.y,
                                        msg->pose.orientation.z,
                                        msg->pose.orientation.w)).getRPY(current_roll, current_pitch, current_yaw);

        // Calculate changes in roll, pitch, and yaw angles
        double d_roll = current_roll - previous_roll;
        double d_pitch = current_pitch - previous_pitch;
        double d_yaw = current_yaw - previous_yaw;

        // Normalize angles to be within -pi to pi
        d_roll = normalizeAngle(d_roll);
        d_pitch = normalizeAngle(d_pitch);
        d_yaw = normalizeAngle(d_yaw);

        // Calculate angular velocities
        velocity_msg.twist.twist.angular.x = d_roll / dt;
        velocity_msg.twist.twist.angular.y = d_pitch / dt;
        velocity_msg.twist.twist.angular.z = d_yaw / dt;

        // Publish the velocity message
        velocity_pub.publish(velocity_msg);

        // Update previous pose and time
        previous_pose = *msg;
        previous_time = current_time;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher velocity_pub;
    ros::Subscriber pose_sub;
    geometry_msgs::PoseStamped previous_pose;
    ros::Time previous_time;

    // Function to normalize an angle to the range [-pi, pi]
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ship_velocity_node");

    ShipVelocityNode ship_velocity_node; // Create an instance of the node

    ros::spin(); // Enter the ROS event loop
    return 0;
}