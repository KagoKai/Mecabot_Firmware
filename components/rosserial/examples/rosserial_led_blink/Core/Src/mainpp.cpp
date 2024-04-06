/*
 * mainpp.cpp
 *
 *  Created on: Mar 23, 2024
 *      Author: ADMIN
 */

#include "mainpp.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.set_tx_cplt();
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.reset_rbuf();
}

void led_blink_callback(const std_msgs::Empty& led_cmd)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::Empty> sub("led_blink", &led_blink_callback);

char data[] = "Hello world!";

void setup()
{
	nh.initNode();
	nh.advertise(chatter);
	nh.subscribe(sub);
}

void loop()
{
	str_msg.data = "hello";
	chatter.publish(&str_msg);

	nh.spinOnce();

	HAL_Delay(1000);
}
