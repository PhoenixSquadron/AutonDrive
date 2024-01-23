#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "servo_controller");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/servo_command", 10); // Replace with the actual topic name for your servo

    ros::Duration(1.0).sleep(); // Give some time for the publisher to initialize

    double angle = 10.0; // Rotation angle in degrees
    double angle_rad = angle * (3.14159 / 180.0); // Convert to radians

    std_msgs::Float64 msg;
    msg.data = angle_rad;

    pub.publish(msg);

    ros::Duration(1.0).sleep(); // Allow time for the servo to move

    return 0;
}
