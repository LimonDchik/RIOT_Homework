/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */


#include <math.h>
#include "include/mpu6050_Kalman.h"
#include "byteorder.h"
#include "xtimer.h"

#define MPU9X50_ACCEL_START_REG         (0x3B)
#define MPU9X50_GYRO_START_REG          (0x43)
#define MPU9X50_TEMP_START_REG          (0x41)
#define MPU9X50_TEMP_SENSITIVITY    340 //Перепроверить
#define MPU9X50_TEMP_OFFSET         35  //Перепроверить

#define DEV_I2C             (dev->params.i2c)
#define DEV_ADDR            (dev->params.addr)
#define DEV_COMP_ADDR       (dev->params.comp_addr)

#define MAX_VALUE           (0x7FFF)

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};



void MPU6050_Init(mpu9x50_t *dev, mpu9x50_params_t *params) {
    mpu9x50_init(dev, params);
    mpu9x50_set_accel_power(dev, MPU9X50_SENSOR_PWR_ON);
    mpu9x50_set_gyro_power(dev, MPU9X50_SENSOR_PWR_ON);
    mpu9x50_set_compass_power(dev, MPU9X50_SENSOR_PWR_OFF);
    mpu9x50_set_gyro_fsr(dev, MPU9X50_GYRO_FSR_2000DPS);
    mpu9x50_set_accel_fsr(dev, MPU9X50_ACCEL_FSR_16G);
}


int MPU6050_Read_Accel(MPU6050_t *DataStruct, mpu9x50_t *dev, mpu9x50_results_t *output) {
    uint8_t data[6];
    int16_t temp;
    float fsr;

    switch (dev->conf.accel_fsr) {
        case MPU9X50_ACCEL_FSR_2G:
            fsr = 2000.0;
            break;
        case MPU9X50_ACCEL_FSR_4G:
            fsr = 4000.0;
            break;
        case MPU9X50_ACCEL_FSR_8G:
            fsr = 8000.0;
            break;
        case MPU9X50_ACCEL_FSR_16G:
            fsr = 16000.0;
            break;
        default:
            return -2;
    }

    /* Acquire exclusive access */
    i2c_acquire(DEV_I2C);
    /* Read raw data */
    i2c_read_regs(DEV_I2C, DEV_ADDR, MPU9X50_ACCEL_START_REG, data, 6, 0);
    /* Release the bus */
    i2c_release(DEV_I2C);

    DataStruct->Accel_X_RAW = (int16_t) (data[0] << 8 | data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (data[2] << 8 | data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (data[4] << 8 | data[5]);

    /* Normalize data according to configured full scale range */
    temp = (data[0] << 8) | data[1];
    output->x_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[2] << 8) | data[3];
    output->y_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[4] << 8) | data[5];
    output->z_axis = (temp * fsr) / MAX_VALUE;

    DataStruct->Ax = output->x_axis;
    DataStruct->Ay = output->y_axis;
    DataStruct->Az = output->z_axis;

    return 0;
}


int MPU6050_Read_Gyro(MPU6050_t *DataStruct, mpu9x50_t *dev, mpu9x50_results_t *output) {
    uint8_t data[6];
    int16_t temp;
    float fsr;

    switch (dev->conf.gyro_fsr) {
        case MPU9X50_GYRO_FSR_250DPS:
            fsr = 250.0;
            break;
        case MPU9X50_GYRO_FSR_500DPS:
            fsr = 500.0;
            break;
        case MPU9X50_GYRO_FSR_1000DPS:
            fsr = 1000.0;
            break;
        case MPU9X50_GYRO_FSR_2000DPS:
            fsr = 2000.0;
            break;
        default:
            return -2;
    }

    /* Acquire exclusive access */
    i2c_acquire(DEV_I2C);
    /* Read raw data */
    i2c_read_regs(DEV_I2C, DEV_ADDR, MPU9X50_GYRO_START_REG, data, 6, 0);
    /* Release the bus */
    i2c_release(DEV_I2C);

    DataStruct->Gyro_X_RAW = (int16_t) (data[0] << 8 | data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (data[2] << 8 | data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (data[4] << 8 | data[5]);

    /* Normalize data according to configured full scale range */
    temp = (data[0] << 8) | data[1];
    output->x_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[2] << 8) | data[3];
    output->y_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[4] << 8) | data[5];
    output->z_axis = (temp * fsr) / MAX_VALUE;

    DataStruct->Gx = output->x_axis;
    DataStruct->Gy = output->y_axis;
    DataStruct->Gz = output->z_axis;

    return 0;
}

int MPU6050_Read_Temp(MPU6050_t *DataStruct, mpu9x50_t *dev) {
    uint8_t data[2];
    int16_t temp;

    /* Acquire exclusive access */
    i2c_acquire(DEV_I2C);
    /* Read raw temperature value */
    i2c_read_regs(DEV_I2C, DEV_ADDR, MPU9X50_TEMP_START_REG, data, 2, 0);
    /* Release the bus */
    i2c_release(DEV_I2C);

    temp = (int16_t) (data[0] << 8 | data[1]);
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    return 0;
}

int MPU6050_Read_All(MPU6050_t *DataStruct, mpu9x50_t *dev) {
    uint8_t Rec_Data[14];
    int16_t temp;
    float Acc_fsr;
    float Gyro_fsr;

    // Read 14 BYTES of data starting from MPU9X50_ACCEL_START_REG register

    /* Acquire exclusive access */
    i2c_acquire(DEV_I2C);
    /* Read raw data */
    i2c_read_regs(DEV_I2C, DEV_ADDR, MPU9X50_ACCEL_START_REG, Rec_Data, 14, 0);
    /* Release the bus */
    i2c_release(DEV_I2C);

    switch (dev->conf.accel_fsr) {
        case MPU9X50_ACCEL_FSR_2G:
            Acc_fsr = 2000.0;
            break;
        case MPU9X50_ACCEL_FSR_4G:
            Acc_fsr = 4000.0;
            break;
        case MPU9X50_ACCEL_FSR_8G:
            Acc_fsr = 8000.0;
            break;
        case MPU9X50_ACCEL_FSR_16G:
            Acc_fsr = 16000.0;
            break;
        default:
            return -2;
    }

    

    switch (dev->conf.gyro_fsr) {
        case MPU9X50_GYRO_FSR_250DPS:
            Gyro_fsr = 250.0;
            break;
        case MPU9X50_GYRO_FSR_500DPS:
            Gyro_fsr = 500.0;
            break;
        case MPU9X50_GYRO_FSR_1000DPS:
            Gyro_fsr = 1000.0;
            break;
        case MPU9X50_GYRO_FSR_2000DPS:
            Gyro_fsr = 2000.0;
            break;
        default:
            return -2;
    }

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = (DataStruct->Accel_X_RAW * Acc_fsr) / MAX_VALUE;
    DataStruct->Ay = (DataStruct->Accel_Y_RAW * Acc_fsr) / MAX_VALUE;
    DataStruct->Az = (DataStruct->Accel_Z_RAW * Acc_fsr) / MAX_VALUE;
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    DataStruct->Gx = (DataStruct->Gyro_X_RAW * Gyro_fsr) / MAX_VALUE;
    DataStruct->Gy = (DataStruct->Gyro_Y_RAW * Gyro_fsr) / MAX_VALUE;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW * Gyro_fsr) / MAX_VALUE;

    // Kalman angle solve
    double dt = (double)(xtimer_now() - timer) / 1000.0;
    timer = xtimer_now();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);

    return 0;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
