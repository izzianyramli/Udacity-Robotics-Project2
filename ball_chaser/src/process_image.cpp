#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request service
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    //ROS_INFO_STREAM("Moving robot toward the white ball");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    // Call the command_robot service and pass the requested velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int image_height = img.height;
    int image_width = img.width;

    // Loop through each pixels in the image and check if there is a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass the velocities to it
    // Request a stop when there is no white ball seen by the camera
    for (int i = 0; i < img.height; i++) {
        for (int j = 0; j < img.width; j++) {
            int pixel_index = i * image_width + j;
            
            int r = img.data[3 * pixel_index];
            int g = img.data[3 * pixel_index + 1];
            int b = img.data[3 * pixel_index + 2];
            
            if (r == white_pixel && g == white_pixel && b == white_pixel) {
                if (j < image_width / 3) {
                    ROS_INFO_STREAM("White Ball at the left side");
                    drive_robot(0.0, 0.5);
                }
                else if (j < 2 * image_width / 3) {
                    ROS_INFO_STREAM("White Ball in front");
                    drive_robot(0.5, 0.0);
                }
                else {
                    ROS_INFO_STREAM("White Ball at the right side");
                    drive_robot(0.0, -0.5);
                }
            }
            else {
                ROS_INFO_STREAM("White Ball is not seen");
                drive_robot(0.0, 0.0);
            }
        }
    }
}

int main(int argc, char** argv)
{
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
