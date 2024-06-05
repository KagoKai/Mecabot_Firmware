/*
 * mainpp.cpp
 *
 *  Created on: Mar 23, 2024
 *      Author: ADMIN
 */

#include "mainpp.h"
#include "ros.h"
#include "std_msgs/String.h"

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("/chatter", &str_msg);

char data[] = "Hello world!";

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.set_tx_cplt();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.reset_rbuf();
}

void setup()
{
	nh.initNode();
	nh.advertise(chatter);
}

uint32_t millis()
{
	return HAL_GetTick();
}

void loop()
{
	str_msg.data = "hello";
	chatter.publish(&str_msg);

	HAL_Delay(500);

	nh.spinOnce();
}
