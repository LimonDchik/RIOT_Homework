
/*------------------------
Описание:
    Сбор данных с датчика BMP280;
    Обработка с помощью фильтра Калмана;
    Яркость светодиода повышается вместе с температурой.

Заметки:
    Реализация фильтра почти честно написана самостоятельно.
------------------------*/

#include <stdio.h>

#include "board.h"
#include "periph/gpio.h"
#include "xtimer.h"
#include "periph/pwm.h"
#include "periph/timer.h"
#include "periph/i2c.h"
#include "bmx280.h"

typedef struct
{
    double Q;
    double R;
    double T;
    double P;
} Kalman_t;

void init_my_BMP280(bmx280_params_t* bmp280_params, bmx280_t* bmp280_dev);
uint16_t Kalman_Temp(Kalman_t *Kalman, double newTemp);




Kalman_t KalmanT = {
    .Q = 0.0005f,
    .R = 0.03f,
    .T = 25.0, 
    .P = 1.0};

//double KalmanTemp;
uint32_t timer;

uint16_t Kalman_Temp(Kalman_t *Kalman, double newTemp)
{

    Kalman->P = Kalman->P + Kalman->Q;

    double S = Kalman->P + Kalman->R;
    double K = Kalman->P / S;

    double y = newTemp - Kalman->T;
    Kalman->T += K * y;

    Kalman->P -= K * Kalman->P;
    //Kalman->P = (1.0 - K) * Kalman->P;

    return Kalman->T;
};

void init_my_BMP280(bmx280_params_t* bmp280_params, bmx280_t* bmp280_dev) {
    bmp280_params->i2c_dev = I2C_DEV(0);
    bmp280_params->i2c_addr = 0x76;
    bmp280_params->filter = BMX280_FILTER_OFF; //BMX280_FILTER_16;
    bmp280_params->press_oversample = BMX280_OSRS_SKIPPED; //BMX280_OSRS_X16;
    bmp280_params->temp_oversample = BMX280_OSRS_X1; //BMX280_OSRS_X16;
    bmp280_params->run_mode = BMX280_MODE_NORMAL;
    bmp280_params->t_sb = BMX280_SB_1000; 

    bmx280_init(bmp280_dev, bmp280_params);

    for (uint8_t i = 0; i < 3; i++) {
        bmx280_read_temperature(bmp280_dev);
        xtimer_msleep(100);
    }
}

int main(void) {
    puts("Hello, World!");
    uint16_t temp = 115;
    char buffer[16]; 
    uint16_t min_temp = 5000;
    uint16_t duty_cycle = 1;

    bmx280_params_t my_bmp280_params;
    bmx280_t my_bmp280_dev;
    init_my_BMP280(&my_bmp280_params, &my_bmp280_dev);
    
    pwm_init(PWM_DEV(0), PWM_LEFT, 100, 300);
    pwm_poweron(PWM_DEV(0));
    pwm_set(PWM_DEV(0), 1, duty_cycle);

    while(1) {
        temp = bmx280_read_temperature(&my_bmp280_dev);
        temp = (uint16_t)Kalman_Temp(&KalmanT, (double)temp);
        min_temp = (temp < min_temp) ? temp : min_temp;
        pwm_set(PWM_DEV(0), 1, temp - min_temp);
        sprintf(buffer, "%u.%02u", temp / 100, temp % 100);
        puts(buffer);
        sprintf(buffer, "%u", temp - min_temp);
        puts(buffer);
        puts("------------------------------");
        xtimer_msleep(100);
    }

    return 0;
}
