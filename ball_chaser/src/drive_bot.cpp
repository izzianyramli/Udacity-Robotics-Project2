#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands
ros::Publisher motor_command_publisher;

// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTarget received : linear_x: %1.2f, angular_z: %1.2f", (float)req.linear_x, (float)req.angular_z);
    
    // Publish the requested linear_x and angular_z to robot wheel joints
    geometry_msgs::Twist wheel_joint;
    
    wheel_joint.linear.x = req.linear_x;
    wheel_joint.angular.z = req.angular_z;

    motor_command_publisher.publish(wheel_joint);
    
    // Wait for 3 seconds for robot to move
    ros::Duration(3).sleep();
    
    // Return a response message
    res.msg_feedback = "Wheel joint velocities set - linear_x: " + std::to_string(wheel_joint.linear.x) + " , angular_z: " + std::to_string(wheel_joint.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a drive_bot node and create a handle to it
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send joint commands");

    ros::spin();

    return 0;
}
