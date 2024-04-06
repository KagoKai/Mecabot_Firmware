/*
 * mainpp2.cpp
 *
 *  Created on: Mar 29, 2024
 *      Author: ADMIN
 */
#ifndef DELTA_T
    #define DELTA_T 0.01f // 100Hz sampling frequency
#endif

#ifndef PI
    #define PI 3.14159265358979f
#endif

#ifndef GYRO_MEAN_ERROR
    #define GYRO_MEAN_ERROR PI * (5.0f / 180.0f) // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#endif

#ifndef BETA
    #define BETA sqrt(3.0f/4.0f) * GYRO_MEAN_ERROR    //*from paper*
#endif

#include "mainpp.h"
#include "mpu6050.h"
#include "madgwick_filter.h"
#include "ros_config.h"

extern I2C_HandleTypeDef hi2c1;

/*** Hardware Struct definitions START ***/
MPU6050_t my_mpu = {
		.address = MPU6050_ADDR_LOW
};

MadgwickFilter_t filter = {
		.beta =0
};
/*** Hardware Struct definitions END ***/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.set_tx_cplt();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.reset_rbuf();
}

void ros_setup()
{
	nh.initNode();

	//nh.subscribe(sub_cmd_vel);

	nh.advertise(pub_imu);
	nh.advertise(pub_odom);
	nh.advertise(pub_joint_states);

	tf_broadcaster.init(nh);
	initOdom();
	initJointStates();
}

void setup()
{
	MPU6050_Handle_t mpu_handle = {
			.rate_div = Rate_1KHz_Div,
			.gyro_range = Gyro_Range_250s,
			.accel_range = Accel_Range_2g
	};
	while (MPU6050_Init(&hi2c1, &my_mpu, mpu_handle, 0) != STATUS_OK);

	MadgwickFilter_Handle_t filter_handle = {
			.beta = BETA,
			.sample_rate = DELTA_T
	};
	while (MadgwickFilter_Init(&filter, filter_handle) != STATUS_OK);

	ros_setup();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void loop()
{

	publishImu();

	nh.spinOnce();

	HAL_Delay(100);
}

void initOdom()
{
	for (int i = 0; i < 3; i++)
	{
		odom_pose[i] = 0.0;
		odom_vel[i] = 0.0;
	}

	odom_msg.pose.pose.position.x = 0.0;
	odom_msg.pose.pose.position.y = 0.0;
	odom_msg.pose.pose.position.z = 0.0;

	odom_msg.pose.pose.orientation.w = 0.0;
	odom_msg.pose.pose.orientation.x = 0.0;
	odom_msg.pose.pose.orientation.y = 0.0;
	odom_msg.pose.pose.orientation.z = 0.0;

	odom_msg.twist.twist.linear.x = 0.0;
	odom_msg.twist.twist.linear.y = 0.0;
	odom_msg.twist.twist.angular.z = 0.0;
}

void initJointStates()
{
	joint_states_msg.header.frame_id = "/base_link";
}

void updateImu()
{
	MPU6050_ReadAccelerometer(&hi2c1, &my_mpu);
	MPU6050_ReadGyroscope(&hi2c1, &my_mpu);

	float gx = my_mpu.gyro_scaled.x, gy = my_mpu.gyro_scaled.y, gz = my_mpu.gyro_scaled.z,
		  ax = my_mpu.accel_scaled.x, ay = my_mpu.accel_scaled.y, az = my_mpu.accel_scaled.z;

	MadgwickFilter_Update_IMU(&filter, gx, gy, gz, ax, ay, az);

	imu_msg.angular_velocity.x = gx;
	imu_msg.angular_velocity.y = gy;
	imu_msg.angular_velocity.z = gz;

	imu_msg.linear_acceleration.x = ax;
	imu_msg.linear_acceleration.y = ay;
	imu_msg.linear_acceleration.z = az;

	imu_msg.orientation.w = filter.q.q0;
	imu_msg.orientation.x = filter.q.q1;
	imu_msg.orientation.y = filter.q.q2;
	imu_msg.orientation.z = filter.q.q3;

	imu_msg.angular_velocity_covariance[1] = 0;
	imu_msg.angular_velocity_covariance[2] = 0;
	imu_msg.angular_velocity_covariance[3] = 0;
	imu_msg.angular_velocity_covariance[4] = 0.02;
	imu_msg.angular_velocity_covariance[5] = 0;
	imu_msg.angular_velocity_covariance[6] = 0;
	imu_msg.angular_velocity_covariance[7] = 0;
	imu_msg.angular_velocity_covariance[8] = 0.02;

	imu_msg.linear_acceleration_covariance[0] = 0.04;
	imu_msg.linear_acceleration_covariance[1] = 0;
	imu_msg.linear_acceleration_covariance[2] = 0;
	imu_msg.linear_acceleration_covariance[3] = 0;
	imu_msg.linear_acceleration_covariance[4] = 0.04;
	imu_msg.linear_acceleration_covariance[5] = 0;
	imu_msg.linear_acceleration_covariance[6] = 0;
	imu_msg.linear_acceleration_covariance[7] = 0;
	imu_msg.linear_acceleration_covariance[8] = 0.04;

	imu_msg.orientation_covariance[0] = 0.0025;
	imu_msg.orientation_covariance[1] = 0;
	imu_msg.orientation_covariance[2] = 0;
	imu_msg.orientation_covariance[3] = 0;
	imu_msg.orientation_covariance[4] = 0.0025;
	imu_msg.orientation_covariance[5] = 0;
	imu_msg.orientation_covariance[6] = 0;
	imu_msg.orientation_covariance[7] = 0;
	imu_msg.orientation_covariance[8] = 0.0025;
}

void updateOdometry()
{
	odom_msg.header.frame_id = odom_header_frame_id;
	odom_msg.child_frame_id = odom_child_frame_id;

	odom_msg.pose.pose.position.x = odom_pose[0];
	odom_msg.pose.pose.position.y = odom_pose[1];
	odom_msg.pose.pose.position.z = 0.0;

	odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

	odom_msg.twist.twist.linear.x = odom_vel[0];
	odom_msg.twist.twist.linear.y = odom_vel[1];
	odom_msg.twist.twist.angular.z = odom_vel[2];
}

void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
	odom_tf.header = odom_msg.header;
	odom_tf.child_frame_id = odom_msg.child_frame_id;
	odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
	odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
	odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
	odom_tf.transform.rotation      = odom_msg.pose.pose.orientation;
}

bool calculateOdometry(float diff_time)
{
	float ax = my_mpu.accel_scaled.x, ay = my_mpu.accel_scaled.y;
	float gz = my_mpu.gyro_scaled.z;

	odom_vel[0] = ax * diff_time;
	odom_vel[1] = ay * diff_time;
	odom_vel[2] = gz * diff_time;

}
