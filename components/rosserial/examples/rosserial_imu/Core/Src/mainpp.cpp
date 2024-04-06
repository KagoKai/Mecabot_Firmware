/*
 * mainpp.cpp
 *
 *  Created on: Mar 27, 2024
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
#include "ros.h"
#include "ros/time.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"

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

ros::NodeHandle nh;

/*
 * Messages
 */
sensor_msgs::Imu imu_msg;

/*
 * Subscribers
 */


/*
 * Publishers
 */
ros::Publisher pub_imu("imu", &imu_msg);

/*
 * TF information
 */
char base_link[] = "/base_link";
char odom[] = "/odom";
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

sensor_msgs::Imu updateImu(void)
{
	MPU6050_ReadAccelerometer(&hi2c1, &my_mpu);
	MPU6050_ReadGyroscope(&hi2c1, &my_mpu);

	float 	gx = my_mpu.gyro_scaled.x,
			gy = my_mpu.gyro_scaled.y,
			gz = my_mpu.gyro_scaled.z,
			ax = my_mpu.accel_scaled.x,
			ay = my_mpu.accel_scaled.y,
			az = my_mpu.accel_scaled.z;

	MadgwickFilter_Update_IMU(&filter, gx, gy, gz, ax, ay, az);
	Quaternion_t quat = filter.q;

	sensor_msgs::Imu tmp_imu_msgs;

    tmp_imu_msgs.angular_velocity.x = gx;
    tmp_imu_msgs.angular_velocity.y = gy;
    tmp_imu_msgs.angular_velocity.z = gz;

    tmp_imu_msgs.linear_acceleration.x = ax;
    tmp_imu_msgs.linear_acceleration.y = ay;
    tmp_imu_msgs.linear_acceleration.z = az;

    tmp_imu_msgs.orientation.w = quat.q0;
    tmp_imu_msgs.orientation.x = quat.q1;
    tmp_imu_msgs.orientation.y = quat.q2;
    tmp_imu_msgs.orientation.z = quat.q3;

    tmp_imu_msgs.angular_velocity_covariance[1] = 0;
    tmp_imu_msgs.angular_velocity_covariance[2] = 0;
    tmp_imu_msgs.angular_velocity_covariance[3] = 0;
    tmp_imu_msgs.angular_velocity_covariance[4] = 0.02;
    tmp_imu_msgs.angular_velocity_covariance[5] = 0;
    tmp_imu_msgs.angular_velocity_covariance[6] = 0;
    tmp_imu_msgs.angular_velocity_covariance[7] = 0;
    tmp_imu_msgs.angular_velocity_covariance[8] = 0.02;

    tmp_imu_msgs.linear_acceleration_covariance[0] = 0.04;
    tmp_imu_msgs.linear_acceleration_covariance[1] = 0;
    tmp_imu_msgs.linear_acceleration_covariance[2] = 0;
    tmp_imu_msgs.linear_acceleration_covariance[3] = 0;
    tmp_imu_msgs.linear_acceleration_covariance[4] = 0.04;
    tmp_imu_msgs.linear_acceleration_covariance[5] = 0;
    tmp_imu_msgs.linear_acceleration_covariance[6] = 0;
    tmp_imu_msgs.linear_acceleration_covariance[7] = 0;
    tmp_imu_msgs.linear_acceleration_covariance[8] = 0.04;

    tmp_imu_msgs.orientation_covariance[0] = 0.0025;
    tmp_imu_msgs.orientation_covariance[1] = 0;
    tmp_imu_msgs.orientation_covariance[2] = 0;
    tmp_imu_msgs.orientation_covariance[3] = 0;
    tmp_imu_msgs.orientation_covariance[4] = 0.0025;
    tmp_imu_msgs.orientation_covariance[5] = 0;
    tmp_imu_msgs.orientation_covariance[6] = 0;
    tmp_imu_msgs.orientation_covariance[7] = 0;
    tmp_imu_msgs.orientation_covariance[8] = 0.0025;

    return tmp_imu_msgs;
}

void publishImu(void)
{
	imu_msg = updateImu();

	imu_msg.header.stamp = nh.now();
	imu_msg.header.frame_id = "/imu";

	/* Publish IMU messages */
	pub_imu.publish(&imu_msg);
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

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	nh.initNode();
	broadcaster.init(nh);
	nh.advertise(pub_imu);
}

void loop()
{

	publishImu();

	nh.spinOnce();

	HAL_Delay(100);
}
