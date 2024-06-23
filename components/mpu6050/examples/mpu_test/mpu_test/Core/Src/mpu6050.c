#include "mpu6050.h"

const int buffer_size = 1000;
const int discard_value = 100;
const int gyro_deadzone = 3;
const int accel_deadzone = 3;
const int accel_offset_divisor = 8;
const int gyro_offset_divisor = 4;

int16_t mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0, state = 0;
int16_t offset_ax = 0, offset_ay = 0, offset_az = 0, offset_gx = 0, offset_gy = 0, offset_gz = 0;

int16_t g_accel_val = (int16_t)16384;

status_t MPU6050_ReadWhoAmI(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu, uint8_t is_high_addr, uint8_t *buffer)
{
    HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_WHO_AM_I, 1, buffer, 1, MPU6050_DEFAULT_TIMEOUT);
    return STATUS_OK;
}

status_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu, MPU6050_Handle_t handle , uint8_t is_high_addr)
{
    uint8_t tmp_byte;
    uint8_t check_buffer=0;

    MPU6050_ReadWhoAmI(hi2c, mpu, is_high_addr, &check_buffer);

    /* Check WHO_AM_I value */
    if (check_buffer == (0x68 | is_high_addr))
    {
        // TODO: Implement intialization process

        /* Wake-up routine */
        tmp_byte = 0x00;
        HAL_StatusTypeDef wkup_status = HAL_I2C_Mem_Write(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_PWR_MGMT_1, 1, &tmp_byte, 1, MPU6050_DEFAULT_TIMEOUT);

        //uint8_t debug = 0xFF;
        //HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_PWR_MGMT_1, 1, &debug, 1, MPU6050_DEFAULT_TIMEOUT);

        /* Set the sample rate */
        MPU6050_ConfigSampleRate(hi2c, mpu, handle.rate_div);
        //HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_SMPRT_DIV, 1, &debug, 1, MPU6050_DEFAULT_TIMEOUT);

        /* Set the gyroscope range */
        MPU6050_ConfigGyroscope(hi2c, mpu, handle.gyro_range);
        //HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_GYRO_CONFIG, 1, &debug, 1, MPU6050_DEFAULT_TIMEOUT);

        /* Set the accelerometer range */
        MPU6050_ConfigAccelerometer(hi2c, mpu, handle.accel_range);
        //HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_ACCEL_CONFIG, 1, &debug, 1, MPU6050_DEFAULT_TIMEOUT);

        /* Set the internal Digital Low-Pass Filter bandwidth */
        MPU6050_ConfigDLFP(hi2c, mpu, handle.filter_bandwidth);

        MPU6050_SetAccelerometerOffset(mpu, MPU6050_OFFSET_AX, MPU6050_OFFSET_AY, MPU6050_OFFSET_AZ);
        MPU6050_SetGyroscopeOffset(mpu, MPU6050_OFFSET_GX, MPU6050_OFFSET_GY, MPU6050_OFFSET_GZ);

        return STATUS_OK;
    }
    
    return STATUS_FAIL;
}

status_t MPU6050_ConfigSampleRate(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu, MPU6050_SampleRateDiv_t rate_div)
{
    uint8_t tmp_byte = (uint8_t)rate_div;

    // Write the SMPRT_DIV setting to the register
    HAL_I2C_Mem_Write(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_SMPRT_DIV, 1, &tmp_byte, 1, MPU6050_DEFAULT_TIMEOUT);

    return STATUS_OK;
}

status_t MPU6050_ConfigGyroscope(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu, MPU6050_GyroscopeRange_t gyro_range)
{
    uint8_t tmp_byte;

    // Read the current register value
    HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_GYRO_CONFIG, 1, &tmp_byte, 1, MPU6050_DEFAULT_TIMEOUT);

    // Bit-masking 
    tmp_byte &= !(0x03 << 3);
    tmp_byte |=  (uint8_t)gyro_range << 3;

    // Write the FS_SEL setting to the register
    HAL_I2C_Mem_Write(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_GYRO_CONFIG, 1, &tmp_byte, 1, MPU6050_DEFAULT_TIMEOUT);

    // Set the scaling factor for true data
    switch (gyro_range)
    {
    case Gyro_Range_250s:
        mpu->gyro_scaling_factor = 1.0 / 131.0 * DEG2RAD;
        break;
    case Gyro_Range_500s:
        mpu->gyro_scaling_factor = 1.0 / 65.5 * DEG2RAD;
        break;
    case Gyro_Range_1000s:
        mpu->gyro_scaling_factor = 1.0 / 32.8 * DEG2RAD;
        break;
    case Gyro_Range_2000s:
        mpu->gyro_scaling_factor = 1.0 / 16.4 * DEG2RAD;
        break;
    default:
        break;
    }

    return STATUS_OK;
}

status_t MPU6050_ConfigAccelerometer(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu, MPU6050_AccelerometerRange_t accel_range)
{
    uint8_t tmp_byte;

    // Read the current register value
    HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_ACCEL_CONFIG, 1, &tmp_byte, 1, MPU6050_DEFAULT_TIMEOUT);

    // Bit-masking 
    tmp_byte &= !(0x03 << 3);
    tmp_byte |=  (uint8_t)accel_range << 3;

    // Write the AFS_SEL setting to the register
    HAL_I2C_Mem_Write(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_ACCEL_CONFIG, 1, &tmp_byte, 1, MPU6050_DEFAULT_TIMEOUT);

    // Set the scaling factor for true data
    switch (accel_range)
    {
    case Accel_Range_2g:
        mpu->accel_scaling_factor = G / 16384.0;
        g_accel_val = (int16_t)16384;
        break;
    case Accel_Range_4g:
        mpu->accel_scaling_factor = G / 8192.0;
        g_accel_val = (int16_t)8192;
        break;
    case Accel_Range_8g:
        mpu->accel_scaling_factor = G / 4096.0;
        g_accel_val = (int16_t)4096;
        break;
    case Accel_Range_16g:
        mpu->accel_scaling_factor = G / 2048.0;
        g_accel_val = (int16_t)2048;
        break;
    default:
        break;
    }

    return STATUS_OK;
}

status_t MPU6050_ConfigDLFP(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu, MPU6050_DLFP_t filter_bandwidth)
{
    uint8_t tmp_byte;

    // Read the current register value
    HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_CONFIG, 1, &tmp_byte, 1, MPU6050_DEFAULT_TIMEOUT);

    // Bit-masking 
    tmp_byte &= !(0x07 << 0);
    tmp_byte |=  (uint8_t)filter_bandwidth;

    // Write the DLFP bandwidth to the register
    HAL_I2C_Mem_Write(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_CONFIG, 1, &tmp_byte, 1, MPU6050_DEFAULT_TIMEOUT);

    return STATUS_OK;
}

status_t MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu)
{
    // Measure the initial mean value;
    MPU6050_MeanSensor(hi2c, mpu);
    HAL_Delay(1000);

    // Calibrate the offset until its within the deadzone
    MPU6050_CalculateOffset(hi2c, mpu);
    HAL_Delay(1000);

    return STATUS_OK;
}

status_t MPU6050_SetGyroscopeOffset(MPU6050_t *mpu, int16_t offset_x, int16_t offset_y, int16_t offset_z)
{
    mpu->gyro_offset.x = offset_x;
    mpu->gyro_offset.y = offset_y;
    mpu->gyro_offset.z = offset_z;

    return STATUS_OK;
}

status_t MPU6050_SetAccelerometerOffset(MPU6050_t *mpu, int16_t offset_x, int16_t offset_y, int16_t offset_z)
{
    mpu->accel_offset.x = offset_x;
    mpu->accel_offset.y = offset_y;
    mpu->accel_offset.z = offset_z;

    return STATUS_OK;
}

status_t MPU6050_MeanSensor(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu)
{
    int i = 0;
    int32_t buff_ax = 0, buff_ay = 0, buff_az = 0,
            buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffer_size + discard_value + 1))
    {
        MPU6050_ReadGyroscope(hi2c, mpu);
        MPU6050_ReadAccelerometer(hi2c, mpu);

        // Skip the first 100 measurements
        if (i > discard_value)
        {
            buff_gx += mpu->gyro_raw.x;
            buff_gy += mpu->gyro_raw.y;
            buff_gz += mpu->gyro_raw.z;   

            buff_ax += mpu->accel_raw.x;
            buff_ay += mpu->accel_raw.y;
            buff_az += mpu->accel_raw.z;
        }

        if (i == (buffer_size + discard_value))
        {
            mean_gx = buff_gx / buffer_size;
            mean_gy = buff_gy / buffer_size;
            mean_gz = buff_gz / buffer_size;

            mean_ax = buff_ax / buffer_size;
            mean_ay = buff_ay / buffer_size;
            mean_az = buff_az / buffer_size;
        }
        
        i++;
        HAL_Delay(5);
    }

    return STATUS_OK;
}

status_t MPU6050_CalculateOffset(I2C_HandleTypeDef *hi2c, MPU6050_t* mpu)
{
    offset_ax = -mean_ax / accel_offset_divisor;
    offset_ay = -mean_ay / accel_offset_divisor;
    offset_az = -g_accel_val + (-mean_az / accel_offset_divisor);

    offset_gx = -mean_gx / gyro_offset_divisor;
    offset_gy = -mean_gy / gyro_offset_divisor;
    offset_gz = -mean_gz / gyro_offset_divisor;

    while (1)
    {
        int ready = 0;

        MPU6050_SetGyroscopeOffset(mpu, offset_gx, offset_gy, offset_gz);
        MPU6050_SetAccelerometerOffset(mpu, offset_ax, offset_ay, offset_az);
    
        MPU6050_MeanSensor(hi2c, mpu);

        if (abs(mean_gx) <= gyro_deadzone) ready++;
        else offset_gx = offset_gx - mean_gx / gyro_deadzone;

        if (abs(mean_gy) <= gyro_deadzone) ready++;
        else offset_gy = offset_gy - mean_gy / gyro_deadzone;

        if (abs(mean_gz) <= gyro_deadzone) ready++;
        else offset_gz = offset_gz - mean_gz / gyro_deadzone;

        if (abs(mean_ax) <= accel_deadzone) ready++;
        else offset_ax = offset_ax - mean_ax / accel_deadzone;

        if (abs(mean_ay) <= accel_deadzone) ready++;
        else offset_ay = offset_ay - mean_ay / accel_deadzone;
        
        if (abs(mean_az) <= accel_deadzone) ready++;
        else offset_az = offset_az - mean_az / accel_deadzone;

        if (ready == 6) break;
    }
    
    return 0;
}

status_t MPU6050_ReadGyroscope(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    uint8_t data[6];

    HAL_StatusTypeDef gyro_read_status;
    // Read 6 bytes of RAW data starting from ACCEL_XOUT_H register.
    gyro_read_status = HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_GYRO_XOUT_H, 1, data, 6, MPU6050_DEFAULT_TIMEOUT);

    mpu->gyro_raw.x = (int16_t)(data[0] << 8 | data[1]) + mpu->gyro_offset.x;
    mpu->gyro_raw.y = (int16_t)(data[2] << 8 | data[3]) + mpu->gyro_offset.y;
    mpu->gyro_raw.z = (int16_t)(data[4] << 8 | data[5]) + mpu->gyro_offset.z;

    // Convert the RAW data into angular acceleration in 'deg/s'
    mpu->gyro_scaled.x = mpu->gyro_raw.x * mpu->gyro_scaling_factor;
    mpu->gyro_scaled.y = mpu->gyro_raw.y * mpu->gyro_scaling_factor;
    mpu->gyro_scaled.z = mpu->gyro_raw.z * mpu->gyro_scaling_factor;

    return STATUS_OK;
}

status_t MPU6050_ReadAccelerometer(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    uint8_t data[6];

    HAL_StatusTypeDef accel_read_status;
    // Read 6 bytes of RAW data starting from ACCEL_XOUT_H register.
    accel_read_status = HAL_I2C_Mem_Read(hi2c, (mpu->address << 1), (uint8_t)MPU6050_REG_ACCEL_XOUT_H, 1, data, 6, MPU6050_DEFAULT_TIMEOUT);

    mpu->accel_raw.x = (int16_t)(data[0] << 8 | data[1]) + mpu->accel_offset.x;
    mpu->accel_raw.y = (int16_t)(data[2] << 8 | data[3]) + mpu->accel_offset.y;
    mpu->accel_raw.z = (int16_t)(data[4] << 8 | data[5]) + mpu->accel_offset.z;

    // Convert the RAW data into linear acceleration in 'm/s^2'
    mpu->accel_scaled.x = mpu->accel_raw.x * mpu->accel_scaling_factor;
    mpu->accel_scaled.y = mpu->accel_raw.y * mpu->accel_scaling_factor;
    mpu->accel_scaled.z = mpu->accel_raw.z * mpu->accel_scaling_factor;

    return STATUS_OK;
}
