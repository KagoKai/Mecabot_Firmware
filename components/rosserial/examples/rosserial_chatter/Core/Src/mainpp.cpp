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
ros::Publisher chatter("chatter", &str_msg);

char data[] = "Hello world!";

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//if (huart->Instance == comm_rosserial.get_handle()->Instance)
	//{
	comm_rosserial.set_tx_cplt();
	//}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.reset_rbuf();
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void setup()
{
	nh.initNode();
	nh.advertise(chatter);
}

void loop()
{
	str_msg.data = "hello";
	chatter.publish(&str_msg);

	nh.spinOnce();

	HAL_Delay(1000);
}
