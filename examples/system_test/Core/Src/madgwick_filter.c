#include "madgwick_filter.h"

status_t MadgwickFilter_Init(MadgwickFilter_t *filter, MadgwickFilter_Handle_t handle)
{
    filter->q.q0 = 1.0f;
    filter->q.q1 = 0.0f;
    filter->q.q2 = 0.0f;
    filter->q.q3 = 0.0f;

    MadgwickFilter_SetBeta(filter, handle.beta);
    MadgwickFilter_SetSampleRate(filter, handle.sample_rate);

    return STATUS_OK;
}

status_t MadgwickFilter_SetBeta(MadgwickFilter_t *filter, float beta)
{
    filter->beta = beta;

    return STATUS_OK;
}

status_t MadgwickFilter_SetSampleRate(MadgwickFilter_t *filter, float rate)
{
    filter->sample_rate = rate;

    return STATUS_OK;
}

status_t MadgwickFilter_Update_IMU(MadgwickFilter_t *filter, float gx, float gy, float gz, float ax, float ay, float az)
{
    //Variables and constants
    Quaternion_t q_est_prev = filter->q;
    Quaternion_t q_est_dot = {0};            // used as a place holder in equations 42 and 43
    //const struct quaternion q_g_ref = {0, 0, 0, 1};// equation (23), reference to field of gravity for gradient descent optimization (not needed because I used eq 25 instead of eq 21
    Quaternion_t q_a = {0, ax, ay, az};    // equation (24) raw acceleration values, needs to be normalized
    
    float F_g [3] = {0};                        // equation(15/21/25) objective function for gravity
    float J_g [3][4] = {0};                     // jacobian matrix for gravity
    
    Quaternion_t gradient = {0};
    
    /* Integrate angluar velocity to obtain position in angles. */
    Quaternion_t q_w;                   // equation (10), places gyroscope readings in a quaternion
    q_w.q0 = 0;                              // the real component is zero, which the Madgwick uses to simplfy quat. mult.
    q_w.q1 = gx;
    q_w.q2 = gy;
    q_w.q3 = gz;
    
    q_w = quat_mult(q_est_prev, q_w);        // equation (12)
    q_w = quat_scalar(q_w, 0.5);                  // equation (12) dq/dt = (1/2)q*w

    /* NOTE
    * Page 10 states equation (40) substitutes equation (13) into it. This seems false, as he actually
    * substitutes equation (12), q_se_dot_w, not equation (13), q_se_w.
    * 
    * // quat_scalar(&q_w, deltaT);               // equation (13) integrates the angles velocity to position
    * // quat_add(&q_w, q_w, q_est_prev);         // addition part of equation (13)
    */

    /* Compute the gradient by multiplying the jacobian matrix by the objective function. This is equation 20.
     The Jacobian matrix, J, is a 3x4 matrix of partial derivatives for each quaternion component in the x y z axes
     The objective function, F, is a 3x1 matrix for x y and z.
     To multiply these together, the inner dimensions must match, so use J'.
     I calculated "by hand" the transpose of J, so I will be using "hard coordinates" to get those values from J.
     The matrix multiplcation can also be done hard coded to reduce code.
     
     Note: it is possible to compute the objective function with quaternion multiplcation functions, but it does not take into account the many zeros that cancel terms out and is not optimized like the paper shows
     */
    
    quat_Normalize(&q_a);              // normalize the acceleration quaternion to be a unit quaternion
    //Compute the objective function for gravity, equation(15), simplified to equation (25) due to the 0's in the acceleration reference quaternion
    F_g[0] = 2*(q_est_prev.q1 * q_est_prev.q3 - q_est_prev.q0 * q_est_prev.q2) - q_a.q1;
    F_g[1] = 2*(q_est_prev.q0 * q_est_prev.q1 + q_est_prev.q2* q_est_prev.q3) - q_a.q2;
    F_g[2] = 2*(0.5 - q_est_prev.q1 * q_est_prev.q1 - q_est_prev.q2 * q_est_prev.q2) - q_a.q3;
    
    //Compute the Jacobian matrix, equation (26), for gravity
    J_g[0][0] = -2 * q_est_prev.q2;
    J_g[0][1] =  2 * q_est_prev.q3;
    J_g[0][2] = -2 * q_est_prev.q0;
    J_g[0][3] =  2 * q_est_prev.q1;
    
    J_g[1][0] = 2 * q_est_prev.q1;
    J_g[1][1] = 2 * q_est_prev.q0;
    J_g[1][2] = 2 * q_est_prev.q3;
    J_g[1][3] = 2 * q_est_prev.q2;
    
    J_g[2][0] = 0;
    J_g[2][1] = -4 * q_est_prev.q1;
    J_g[2][2] = -4 * q_est_prev.q2;
    J_g[2][3] = 0;
    
    // now computer the gradient, equation (20), gradient = J_g'*F_g
    gradient.q0 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
    gradient.q1 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
    gradient.q2 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
    gradient.q3 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];
    
    // Normalize the gradient, equation (44)
    quat_Normalize(&gradient);
  
    /* This is the sensor fusion part of the algorithm.
     Combining Gyroscope position angles calculated in the beginning, with the quaternion orienting the accelerometer to gravity created above.
     Noticably this new quaternion has not be created yet, I have only calculated the gradient in equation (19).
     Madgwick however uses assumptions with the step size and filter gains to optimize the gradient descent,
        combining it with the sensor fusion in equations (42-44).
     He says the step size has a var alpha, which he assumes to be very large.
     This dominates the previous estimation in equation (19) to the point you can ignore it.
     Eq. 36 has the filter gain Gamma, which is related to the step size and thus alpha. With alpha being very large,
        you can make assumptions to simplify the fusion equatoin of eq.36.
     Combining the simplification of the gradient descent equation with the simplification of the fusion equation gets you eq.
     41 which can be subdivided into eqs 42-44.
    */
    gradient = quat_scalar(gradient, filter->beta);             // multiply normalized gradient by beta
    q_est_dot = quat_sub(q_w, gradient);        // subtract above from q_w, the integrated gyro quaternion
    q_est_dot = quat_scalar(q_est_dot, filter->sample_rate);
    filter->q = quat_add(q_est_prev, q_est_dot);     // Integrate orientation rate to find position
    quat_Normalize(&filter->q);                 // normalize the orientation of the estimate
}

status_t MadgwickFilter_Update_MARG(MadgwickFilter_t *filter, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{

}
