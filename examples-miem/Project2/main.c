
/*------------------------
Описание:
    Машинка;
    Сбор данных с MPU6050;
    Обработка через ПИД и микшеры;
    управление моторами через ШИМ и аппаратные драйверы.

Заметки:
    ПИД и микшеры - это сильно урезанные версии из BetaFlight;
    ПИД и микшеры хорошо себя показали для собственной реализации полётного контроллера;
    Здесь они пока не работают, скорее всего будут полностью переписаны, причём нейросетью;
    Зато моторы работают;
    Планируется добавить получение команд дистанционно, от джойстика на ArduinoUNO по BLE.
------------------------*/


#include <stdio.h>
#include <math.h>

#include "board.h"
#include "periph/gpio.h"
#include "xtimer.h"
#include "periph/pwm.h"
#include "periph/timer.h"
#include "periph/i2c.h"
#include "bmx280.h"
#include "hd44780.h"
#include "mpu9x50.h"
#include "include/mpu6050_Kalman.h"

#include "include/pid_reg.h"
#include "include/mix.h"

#define ONBOARD_LED GPIO_PIN(PORT_A, 5)

//dev      = TIM2
//pin = GPIO_PIN(PORT_B, 3)  /* D3 */, .cc_chan = 1 //4
//pin = GPIO_PIN(PORT_B, 10) /* D6 */, .cc_chan = 2 //2
//dev      = TIM3
//pin = GPIO_PIN(PORT_B, 4) /* D5 */, .cc_chan = 0 //3
//pin = GPIO_PIN(PORT_C, 7) /* D9 */, .cc_chan = 1 //1
//pin = GPIO_PIN(PORT_C, 8),          .cc_chan = 2 (back)

#define XYZ_AXIS_COUNT 3

int float_buffer(char* str, float float_num, int accuracy);


mpu9x50_params_t mpu6050_params = {
    .i2c = I2C_DEV(0),
    .addr = 0x68,
    .comp_addr = 0x0C,
    .sample_rate = MPU9X50_DEFAULT_SAMPLE_RATE,
};

mpu9x50_t mpu6050;
mpu9x50_results_t mpu6050_res;
MPU6050_t my_mpu6050_data;

int float_buffer(char* str, float float_num, int accuracy) {
    char *tmpSign = (float_num < 0) ? "-" : "";
    float tmpVal = (float_num < 0) ? -float_num : float_num;
    int tmpInt1 = tmpVal;                  // Get the integer (678).
    float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
    if (accuracy > 1000) {
        sprintf (str, "%s%d", tmpSign, tmpInt1);
    } else {
        int tmpInt2 = trunc(tmpFrac * 10000) / accuracy;  // Turn into integer (123).
        sprintf (str, "%s%d.%d", tmpSign, tmpInt1, tmpInt2);
    }
    
    return 0;
}

int main(void) {
    
    char buffer[60];
    float fn;
    
    //char buffer_ax[10]; 
    //char buffer_ay[10];
    //char buffer_az[10];
    //char buffer_gx[10];
    //char buffer_gy[10];
    //char buffer_gz[10];
    

    // Print as parts, note that you need 0-padding for fractional bit.
    //sprintf (buffer, "adc_read = %s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
    //puts(buffer);
    puts("Hello,World!");

    gpio_init(ONBOARD_LED, GPIO_OUT);

    MPU6050_Init(&mpu6050, &mpu6050_params);

	float rawSetpoint[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
	float gyroADCf[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
	float pidSums[3] = {0};
	float motor[4];

	pidBFInitDefault();
    pidBFSetPidFrequencyDt(1);
	mixBfInitDefault();
	mixerSetThrottleAngleCorrection(50);
	rawSetpointUpdate(rawSetpoint[0],rawSetpoint[1], rawSetpoint[2]);
    

    //uint16_t duty_cycle = 1;


    
    pwm_init(PWM_DEV(0), PWM_LEFT, 50, 1000); 
    pwm_poweron(PWM_DEV(0));
    pwm_set(PWM_DEV(0), 1, 0);
    pwm_set(PWM_DEV(0), 2, 0);

    pwm_init(PWM_DEV(1), PWM_LEFT, 50, 1000); 
    pwm_poweron(PWM_DEV(1));
    pwm_set(PWM_DEV(1), 0, 0);
    pwm_set(PWM_DEV(1), 1, 0);
    pwm_set(PWM_DEV(1), 2, 0);


    pwm_set(PWM_DEV(0), 1, 0); //3 4
    pwm_set(PWM_DEV(0), 2, 0); //1 2
    pwm_set(PWM_DEV(1), 0, 0); //2 3
    pwm_set(PWM_DEV(1), 1, 0); //0 1
    while(1) {

        //sprintf(buffer, "%.2f", 20.5);
        //hd44780_set_cursor(&mt16s2h, 0, 0);
        //hd44780_print(&mt16s2h, buffer);

        /*
        MPU6050_Read_All(&my_mpu6050_data, &mpu6050);
        float_buffer(buffer_gx, my_mpu6050_data.Gx, 10000);
        float_buffer(buffer_gy, my_mpu6050_data.Gy, 10000);
        float_buffer(buffer_gz, my_mpu6050_data.Gz, 10000);
        float_buffer(buffer_ax, my_mpu6050_data.Ax, 10000);
        float_buffer(buffer_ay, my_mpu6050_data.Ay, 10000);
        float_buffer(buffer_az, my_mpu6050_data.Az, 10000);
        
        sprintf(buffer, "G %s %s %s", buffer_gx, buffer_gy, buffer_gz);
        hd44780_set_cursor(&mt16s2h, 0, 0);
        hd44780_print(&mt16s2h, buffer);
        puts(buffer);
        sprintf(buffer, "A %s %s %s", buffer_ax, buffer_ay, buffer_az);
        hd44780_set_cursor(&mt16s2h, 0, 1);
        hd44780_print(&mt16s2h, buffer);
        puts(buffer);
        xtimer_msleep(1000);
        hd44780_clear(&mt16s2h);
        */

         
    	pidBFInitDefault();
        pidBFSetPidFrequencyDt(1);
        //MPU6050_Read_All(&my_mpu6050_data, &mpu6050);
        //gyroADCf[0] = my_mpu6050_data.Gx;
        //gyroADCf[1] = my_mpu6050_data.Gy;
        //gyroADCf[2] = my_mpu6050_data.Gz;
        MPU6050_Read_Gyro(&my_mpu6050_data, &mpu6050, &mpu6050_res);
        gyroADCf[0] = mpu6050_res.x_axis;
        gyroADCf[1] = mpu6050_res.y_axis;
        gyroADCf[2] = mpu6050_res.z_axis;

        rawSetpointUpdate(rawSetpoint[0],rawSetpoint[1], rawSetpoint[2]);
        pidController(gyroADCf, pidSums);
        //mixTable(pidSums, pidBFGetPidSumLimit(), pidBFGetPidSumLimitYaw(), motor);
        
        fn = mixTable(pidSums, pidBFGetPidSumLimit(), pidBFGetPidSumLimitYaw(), motor);
        
        float_buffer(buffer, gyroADCf[0], 1);
        puts(buffer);
        float_buffer(buffer, gyroADCf[1], 1);
        puts(buffer);
        float_buffer(buffer, gyroADCf[2], 1);
        puts(buffer);
        puts(" ");
        
        puts("PIDSum!!!");
        float_buffer(buffer, pidSums[2], 1);
        puts(buffer);
        puts("PIDSum!!!");
        //puts("DEBUG!!!");
        float_buffer(buffer, fn, 1);
        //puts(buffer);
        //puts("DEBUG!!!");
        float_buffer(buffer, motor[0], 1);
        puts("MOTORS!!!");
        float_buffer(buffer, motor[0], 1);
        puts(buffer);
        

        float_buffer(buffer, motor[1], 1);
        puts(buffer);
        float_buffer(buffer, motor[2], 1);
        puts(buffer);
        float_buffer(buffer, motor[3], 1);
        puts(buffer);
        puts("MOTORS!!!");
        puts(" ");
        //xtimer_msleep(1000);
        
       
        //pwm_set(PWM_DEV(0), 1, (uint16_t)motor[0]);

        pwm_set(PWM_DEV(0), 1, (uint16_t)motor[1]); //1 2
        pwm_set(PWM_DEV(0), 2, (uint16_t)motor[3]); //3 4 
        pwm_set(PWM_DEV(1), 0, (uint16_t)motor[2]); //2 3
        pwm_set(PWM_DEV(1), 1, (uint16_t)motor[0]); //0 1

        //pwm_set(PWM_DEV(0), 1, 550); //3 4
        //pwm_set(PWM_DEV(0), 2, 550); //1 2
        //pwm_set(PWM_DEV(1), 0, 550); //2 3
        //pwm_set(PWM_DEV(1), 1, 550); //0 1

        xtimer_msleep(50);
        
        

        //xtimer_msleep(1000);
    }

    return 0;
}
