#ifndef _MPU6050_H_
#define _MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>
#include <stm32f1xx.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_i2c.h>
#include "operation_status.h"

#define G 9.81

// Offset values calculated from the calibrate function
#define MPU6050_OFFSET_GX           79
#define MPU6050_OFFSET_GY           106
#define MPU6050_OFFSET_GZ           99
#define MPU6050_OFFSET_AX          -893
#define MPU6050_OFFSET_AY          -129
#define MPU6050_OFFSET_AZ          -19155

#define MPU6050_DEFAULT_TIMEOUT     1000

#define MPU6050_I2C_CLOCK           400000

#define MPU6050_ADDR_LOW            0x68
#define MPU6050_ADDR_HIGH           0x69

#define MPU6050_REG_SMPRT_DIV       0x19
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C

#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40

#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42

#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48

#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

extern I2C_HandleTypeDef hi2c1;

extern const int mpu_calib_buffer_size;
extern const int discard_value;
extern const int gyro_deadzone;
extern const int accel_deadzone;
extern const int accel_offset_divisor;
extern const int gyro_offset_divisor;

extern int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state;
extern int16_t offset_ax, offset_ay, offset_az, offset_gx, offset_gy, offset_gz;

extern int16_t g_accel_val;

typedef enum
{
    Rate_8KHz_Div = 0,
    Rate_4KHz_Div = 1,
    Rate_2KHz_Div = 3,
    Rate_1KHz_Div = 7,
    Rate_500Hz_Div = 15,
    Rate_250Hz_Div = 31,
    Rate_125Hz_Div = 63,
    Rate_100Hz_Div = 79
}MPU6050_SampleRateDiv_t;

typedef enum
{
    Gyro_Range_250s  = 0x00,
    Gyro_Range_500s  = 0x01,
    Gyro_Range_1000s = 0x02,
    Gyro_Range_2000s = 0x03
}MPU6050_GyroscopeRange_t;

typedef enum
{
    Accel_Range_2g  = 0x00,
    Accel_Range_4g  = 0x01,
    Accel_Range_8g  = 0x02,
    Accel_Range_16g = 0x03
}MPU6050_AccelerometerRange_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}MPU6050_DataRaw_t;

typedef struct
{
    float x;
    float y;
    float z;
}MPU6050_DataScaled_t;

typedef struct
{
    I2C_HandleTypeDef *i2c_interface;

    uint8_t address;

    float gyro_scaling_factor;
    float accel_scaling_factor;

    MPU6050_DataRaw_t gyro_raw;
    MPU6050_DataRaw_t accel_raw;

    MPU6050_DataRaw_t gyro_offset;
    MPU6050_DataRaw_t accel_offset;

    float temp_raw;

    MPU6050_DataScaled_t gyro_scaled;
    MPU6050_DataScaled_t accel_scaled;

}MPU6050_t;

typedef struct
{
    MPU6050_SampleRateDiv_t rate_div;
    MPU6050_GyroscopeRange_t gyro_range;
    MPU6050_AccelerometerRange_t accel_range;
}MPU6050_Handle_t;

/** @brief Read the address value within the WHO_AM_I register (at address 0x75).
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 * @param is_high_addr True if using pin AD0 for alternative address.
 * @param buffer The data buffer to save data.
 *
 * @return Operation status.
*/
status_t MPU6050_ReadWhoAmI(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu, uint8_t is_high_addr, uint8_t *buffer);

/** @brief Initialize the MPU device with configuration provided in its handle.
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 * @param handle The handle that hold the device configuration.
 * @param is_high_addr True if using pin AD0 for alternative address.
 *
 * @return Operation status.
*/
status_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu, MPU6050_Handle_t handle , uint8_t is_high_addr);

/** @brief Set the Sample Rate using the SMPRT_DIV register (at address 0x19).
 *         Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV)
 *         (Gyroscope Output Rate is 8 kHz when the DLPF is disabled).
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 * @param rate_div The Sample Rate value.
 *
 * @return Operation status.
*/
status_t MPU6050_ConfigSampleRate(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu, MPU6050_SampleRateDiv_t rate_div);

/** @brief Set the Gyroscope sensitivity range using the GYRO_CONFIG register (at address 0x1B).
 *         -> Range:    +- 250 deg/sec
*                       +- 500 deg/sec
 *                      +- 1000 deg/sec
 *                      +- 2000 deg/sec
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 * @param gyro_range The range value.
 *
 * @return Operation status.
*/
status_t MPU6050_ConfigGyroscope(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu, MPU6050_GyroscopeRange_t gyro_range);

/** @brief Set the Gyroscope sensitivity range using the GYRO_CONFIG register (at address 0x1C).
 *         -> Range:    +- 2  G
 *                      +- 4  G
 *                      +- 8  G
 *                      +- 16 G
 *                  (1 G = 9.81 m/s)
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 * @param accel_range The range value.
 *
 * @return Operation status.
*/
status_t MPU6050_ConfigAccelerometer(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu, MPU6050_AccelerometerRange_t accel_range);

/** @brief Calculate the offset that would be hard coded into the device later
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 *
 * @return Operation status.
*/
status_t MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu);

/** @brief Set the RAW offset for the Gyroscope
 *
 * @param mpu The pointer to the MPU structure.
 * @param offset_x The offset in the velocity around X axis.
 * @param offset_y The offset in the velocity around Y axis.
 * @param offset_z The offset in the velocity around Z axis.
 *
 * @return Operation status.
*/
status_t MPU6050_SetGyroscopeOffset(MPU6050_t *mpu, int16_t offset_x, int16_t offset_y, int16_t offset_z);

/** @brief Set the RAW offset for the Accelerometer
 *
 * @param mpu The pointer to the MPU structure.
 * @param offset_x The offset in the X acceleration.
 * @param offset_y The offset in the Y acceleration.
 * @param offset_z The offset in the Z acceleration.
 *
 * @return Operation status.
*/
status_t MPU6050_SetAccelerometerOffset(MPU6050_t *mpu, int16_t offset_x, int16_t offset_y, int16_t offset_z);

/** @brief Calculate the mean value after 1000 measurements
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 *
 * @return Operation status.
*/
status_t MPU6050_MeanSensor(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu);

/** @brief Calculate the offset value until the mean is within a deadzone range.
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 *
 * @return Operation status.
*/
status_t MPU6050_CalculateOffset(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu);

/** @brief Read the raw gyroscope data and convert it to meaningful unit (deg/s)
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 *
 * @return Operation status.
*/
status_t MPU6050_ReadGyroscope(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);

/** @brief Read the raw accelerometer data and convert it to meaningful unit (m/s^2)
 *
 * @param hi2c The pointer to the I2C structure connected to the device.
 * @param mpu The pointer to the MPU structure.
 *
 * @return Operation status.
*/
status_t MPU6050_ReadAccelerometer(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);

#ifdef __cplusplus
}
#endif

#endif
