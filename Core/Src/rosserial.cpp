#include <ros.h>
#include <rosserial.h>
#include "geometry_msgs/TwistWithCovariance.h"

void vel_callback(const geometry_msgs::Twist &msg)
{
	linearvelocity_x = msg.linear.x;
	linearvelocity_y = msg.linear.y;
	angularvelocity = msg.angular.z;
}

//----------------------------definition of ros----------------------------
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", vel_callback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->flush();
}

void Rosserial_Init(void)
{
    nh.initNode();
    nh.subscribe(vel_sub);
}

void Rosserial_Spin(void)
{
		nh.spinOnce();
}

bool Rosserial_Checkconfigstate(void)
{
		return nh.config_state();
}

void Rosserial_GetHardware(void)
{
		nh.getHardware()->init();
}
