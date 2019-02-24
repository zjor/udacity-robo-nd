#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {

    float lx = (float) req.linear_x;
    float az = (float) req.angular_z;

    ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", lx, az);

    geometry_msgs::Twist msg;
    msg.linear.x = lx;
    msg.angular.z = az;

    motor_command_publisher.publish(msg);

    // Return a response message
    res.msg_feedback = "linear_x: " + std::to_string(lx) + " , angular_z: " + std::to_string(az);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send velocity commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}