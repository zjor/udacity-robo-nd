#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z) {
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget req;
    req.request.linear_x = lin_x;
    req.request.angular_z = ang_z;

    if (!client.call(req)) {
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {
    // ROS_INFO("process_image_callback called; width: %d; height: %d; step: %d", img.width, img.height, img.step);

    int white_pixel = 255;
    int side_width = 2 * img.width / 5;

    int min_x = img.width;
    int max_x = -1;

    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {
            int x = (i % img.step) / 3;
            min_x = x < min_x ? x : min_x;
            max_x = x > max_x ? x : max_x;
        }
    }

    if (max_x != -1) {
        int ball_x = (min_x + max_x) / 2;
        ROS_INFO("Whit ball detected at x = %d", ball_x);
        if (ball_x <= side_width) {
            // Turn left
            drive_robot(0.0, 0.1);
        } else if (ball_x >= img.width - side_width) {
            // Turn right
            drive_robot(0.0, -0.1);
        } else {
            // Drive forward
            drive_robot(0.1, 0.0);
        }
    } else {
        // Stop
        drive_robot(0.0, 0.0);
    }

    

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char** argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}