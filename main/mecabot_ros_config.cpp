#include "mecabot_ros_config.h"

void motorSpeedCallback(const std_msgs::UInt8& motor_speed_msg);
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);