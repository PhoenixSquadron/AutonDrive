#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <wiringPi.h>

// GPIO pins for H-Bridge control
const int PIN_SERVO1 = 0;  // Replace with the GPIO pin connected to the first servo
const int PIN_SERVO2 = 1;  // Replace with the GPIO pin connected to the second servo

// Servo angle limits
const double ANGLE_MIN = -90.0;  // Minimum servo angle in degrees
const double ANGLE_MAX = 90.0;   // Maximum servo angle in degrees

// Servo angle increment
const double ANGLE_INCREMENT = 10.0;  // Angle change in degrees

// Rest position angles
const double REST_ANGLE_SERVO1 = 0.0;  // Rest position angle for the first servo
const double REST_ANGLE_SERVO2 = 0.0;  // Rest position angle for the second servo

// ROS publisher for servo commands
ros::Publisher servo1_pub;
ros::Publisher servo2_pub;

// Function to control the servo angle
void setServoAngle(int pin, double angle)
{
    // Convert angle to pulse width
    double pulseWidth = (angle - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN) * 2000 + 500;

    // Set the servo pulse width
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(pin, LOW);
}

// Function to move the servo down by the specified angle
void moveServoDown(ros::Publisher& pub, int pin, double& angle)
{
    angle -= ANGLE_INCREMENT;
    if (angle < ANGLE_MIN)
        angle = ANGLE_MIN;

    std_msgs::Float64 msg;
    msg.data = angle;
    pub.publish(msg);

    setServoAngle(pin, angle);
    delay(1000);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "servo_control");
    ros::NodeHandle nh;

    // Setup GPIO pins
    wiringPiSetup();
    pinMode(PIN_SERVO1, OUTPUT);
    pinMode(PIN_SERVO2, OUTPUT);

    // Create ROS publishers for servo commands
    servo1_pub = nh.advertise<std_msgs::Float64>("/servo1_command", 10);  // Replace with the actual topic name for the first servo
    servo2_pub = nh.advertise<std_msgs::Float64>("/servo2_command", 10);  // Replace with the actual topic name for the second servo

    // Initialize servo angles
    double angle_servo1 = REST_ANGLE_SERVO1;
    double angle_servo2 = REST_ANGLE_SERVO2;

    // Move the first servo down by 10 degrees
    moveServoDown(servo1_pub, PIN_SERVO1, angle_servo1);

    // Return the first servo to the rest position
    setServoAngle(PIN_SERVO1, REST_ANGLE_SERVO1);
    delay(1000);

    // Move the second servo down by 10 degrees
    moveServoDown(servo2_pub, PIN_SERVO2, angle_servo2);

    // Return the second servo to the rest position
    setServoAngle(PIN_SERVO2, REST_ANGLE_SERVO2);
    delay(1000);

    return 0;
}
