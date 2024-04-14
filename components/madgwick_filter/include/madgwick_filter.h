#ifndef _MADGWICK_FILTER_H_
#define _MADGWICK_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "operation_status.h"
#include <math.h>

typedef struct
{
    float q0;   // qw
    float q1;   // qx
    float q2;   // qy
    float q3;   // qz
}Quaternion_t;

typedef struct
{
    Quaternion_t q;
    float beta;
    float sample_rate;
}MadgwickFilter_t;

typedef struct 
{
    float beta;
    float sample_rate;
}MadgwickFilter_Handle_t;

status_t MadgwickFilter_Init(MadgwickFilter_t *filter, MadgwickFilter_Handle_t handle);

status_t MadgwickFilter_SetBeta(MadgwickFilter_t *filter, float beta);

status_t MadgwickFilter_SetSampleRate(MadgwickFilter_t *filter, float rate);

/** @brief Update the quaternion according to the IMU (accel & gyro) measurements.
 * 
 * @param gx, gy, gz The gyroscope measurement (rad/s).
 * @param filter The pointer to the Madgwick filter.
 * @param ax, ay, az The acceleration measurement (m/s^2).
 * 
*/
status_t MadgwickFilter_Update_IMU(MadgwickFilter_t *filter, float gx, float gy, float gz, float ax, float ay, float az);

status_t MadgwickFilter_Update_MARG(MadgwickFilter_t *filter, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

// Multiply two quaternions and return a copy of the result, prod = L * R
static Quaternion_t quat_mult(Quaternion_t L, Quaternion_t R)
{
    Quaternion_t product;
    product.q0 = (L.q0 * R.q0) - (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3);
    product.q1 = (L.q0 * R.q1) + (L.q1 * R.q0) + (L.q2 * R.q3) - (L.q3 * R.q2);
    product.q2 = (L.q0 * R.q2) - (L.q1 * R.q3) + (L.q2 * R.q0) + (L.q3 * R.q1);
    product.q3 = (L.q0 * R.q3) + (L.q1 * R.q2) - (L.q2 * R.q1) + (L.q3 * R.q0);
    
    return product;
}

// Multiply a reference of a quaternion by a scalar, q = s*q
static Quaternion_t quat_scalar(Quaternion_t q, float scalar)
{
    q.q0 *= scalar;
    q.q1 *= scalar;
    q.q2 *= scalar;
    q.q3 *= scalar;

    return q;
}

// Adds two quaternions together and the sum is the pointer to another quaternion, Sum = L + R
static Quaternion_t quat_add(Quaternion_t L, Quaternion_t R)
{
    Quaternion_t sum;
    sum.q0 = L.q0 + R.q0;
    sum.q1 = L.q1 + R.q1;
    sum.q2 = L.q2 + R.q2;
    sum.q3 = L.q3 + R.q3;

    return sum;
}

// Subtracts two quaternions together and the sum is the pointer to another quaternion, sum = L - R
static Quaternion_t quat_sub(Quaternion_t L, Quaternion_t R)
{
    Quaternion_t sum;
    sum.q0 = L.q0 - R.q0;
    sum.q1 = L.q1 - R.q1;
    sum.q2 = L.q2 - R.q2;
    sum.q3 = L.q3 - R.q3;

    return sum;
}

// the conjugate of a quaternion is it's imaginary component sign changed  q* = [s, -v] if q = [s, v]
static Quaternion_t quat_conjugate(Quaternion_t q)
{
    q.q1 = -q.q1;
    q.q2 = -q.q2;
    q.q3 = -q.q3;

    return q;
}

static float quat_Norm (Quaternion_t q)
{
    return sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 +q.q3*q.q3);
}

// Normalizes pointer q by calling quat_Norm(q),
static void quat_Normalize(Quaternion_t *q)
{
    float norm = quat_Norm(*q);
    q -> q0 /= norm;
    q -> q1 /= norm;
    q -> q2 /= norm;
    q -> q3 /= norm;
}

static float quat_getYaw(Quaternion_t q)
{
    return atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
}

#ifdef __cplusplus
}
#endif

#endif