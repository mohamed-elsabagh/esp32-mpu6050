/**********************************************************************
* - Description:		esp32-mpu6050
* - File:				kalman_filter.c
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Mohamed El-Sabagh
* - Target:				ESP32
* - Created:			2017-12-11
* - Last changed:		2017-12-11
*
**********************************************************************/

#include "kalman_filter.h"

/**
  * @brief  Eleminate drift angle of gyroscope using accelerometer.
  * @param  accel: acceleration value
  * @param  gyro: gyroscope value
  * @param  angle_calculations: structure of the angle calculations
  * @retval angle
  */
double filter(double accel, double gyro, angle_calculations *calc)
{
    calc->angle += (gyro - calc->q_bias) * calc->dt;
    calc->angle_err = accel - calc->angle;

    calc->pdot[0] = calc->q_angle - calc->pp[0][1] - calc->pp[1][0];
    calc->pdot[1] = -calc->pp[1][1];
    calc->pdot[2] = -calc->pp[1][1];
    calc->pdot[3] = calc->q_gyro;
    calc->pp[0][0] += calc->pdot[0] * calc->dt;
    calc->pp[0][1] += calc->pdot[1] * calc->dt;
    calc->pp[1][0] += calc->pdot[2] * calc->dt;
    calc->pp[1][1] += calc->pdot[3] * calc->dt;

    calc->pct_0 = calc->c_0 * calc->pp[0][0];
    calc->pct_1 = calc->c_0 * calc->pp[1][0];

    calc->e = calc->r_angle + calc->c_0 * calc->pct_0;

    calc->k_0 = calc->pct_0 / calc->e;
    calc->k_1 = calc->pct_1 / calc->e;

    calc->t_0 = calc->pct_0;
    calc->t_1 = calc->c_0 * calc->pp[0][1];

    calc->pp[0][0] -= calc->k_0 * calc->t_0;
    calc->pp[0][1] -= calc->k_0 * calc->t_1;
    calc->pp[1][0] -= calc->k_1 * calc->t_0;
    calc->pp[1][1] -= calc->k_1 * calc->t_1;

    calc->angle += calc->k_0 * calc->angle_err;
    calc->q_bias += calc->k_1 * calc->angle_err;
    calc->gyro_y = gyro - calc->q_bias;

    return calc->angle;
}
