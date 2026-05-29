//Библиотека честно украдена отсюда: https://github.com/leech001/MPU6050/tree/master
/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "periph/i2c.h"
#include "mpu9x50.h"

// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

void MPU6050_Test(void);

void MPU6050_Init(mpu9x50_t *dev, mpu9x50_params_t *params);

int MPU6050_Read_Accel(MPU6050_t *DataStruct, mpu9x50_t *dev, mpu9x50_results_t *output);

int MPU6050_Read_Gyro(MPU6050_t *DataStruct, mpu9x50_t *dev, mpu9x50_results_t *output);

int MPU6050_Read_Temp(MPU6050_t *DataStruct, mpu9x50_t *dev);

int MPU6050_Read_All(MPU6050_t *DataStruct, mpu9x50_t *dev);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);