
/*------------------------
Описание:
    Сбор данных с датчика MPU6050;
    Обработка с помощью фильтра Калмана;
    Вывод на дисплей MT-16S2H (Отечественный аналог HD44780).

Заметки:
    Реализация фильтра честно украдена из интернета, ссылка в соответствующей библиотеке.
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

#define ONBOARD_LED GPIO_PIN(PORT_A, 5)

//A0 - PC5
//R/W - PC6
//E - PC8
//DB0 - PA10
//DB1 - PB3
//DB2 - PB6
//DB3 - PB4
//DB4 - PB10
//DB5 - PA8
//DB6 - PA9
//DB7 - PC7

#define LCDA0 GPIO_PIN(PORT_C, 5)
#define LCDRW GPIO_PIN(PORT_C, 6)
#define LCDE GPIO_PIN(PORT_C, 8)
#define LCDDB0 GPIO_PIN(PORT_A, 10)
#define LCDDB1 GPIO_PIN(PORT_B, 3)
#define LCDDB2 GPIO_PIN(PORT_B, 6)
#define LCDDB3 GPIO_PIN(PORT_B, 4)
#define LCDDB4 GPIO_PIN(PORT_B, 10)
#define LCDDB5 GPIO_PIN(PORT_A, 8)
#define LCDDB6 GPIO_PIN(PORT_A, 9)
#define LCDDB7 GPIO_PIN(PORT_C, 7)

int float_buffer(char* str, float float_num, int accuracy);

const hd44780_params_t mt16s2h_params = {
    .cols = 16,                    
    .rows = 2,                      
    .rs = LCDA0,                    
    .rw = LCDRW,                    
    .enable = LCDE,                  
    .data = {
        LCDDB0, LCDDB1, LCDDB2, LCDDB3,  
        LCDDB4, LCDDB5, LCDDB6, LCDDB7  
    }
};

hd44780_t mt16s2h;

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
    char buffer_ax[10]; 
    char buffer_ay[10];
    char buffer_az[10];
    char buffer_gx[10];
    char buffer_gy[10];
    char buffer_gz[10];

    // Print as parts, note that you need 0-padding for fractional bit.
    //sprintf (buffer, "adc_read = %s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
    //puts(buffer);
    puts("Hello,World!");

    gpio_init(ONBOARD_LED, GPIO_OUT);

    hd44780_init(&mt16s2h, &mt16s2h_params);
    hd44780_display(&mt16s2h, HD44780_ON);
    hd44780_cursor(&mt16s2h, HD44780_ON);
    hd44780_blink(&mt16s2h, HD44780_ON);
    hd44780_left2right(&mt16s2h);
    hd44780_clear(&mt16s2h);
    hd44780_set_cursor(&mt16s2h, 0, 0); 
    hd44780_write(&mt16s2h, 0x0B);

    MPU6050_Init(&mpu6050, &mpu6050_params);

    while(1) {
        //sprintf(buffer, "%.2f", 20.5);
        //hd44780_set_cursor(&mt16s2h, 0, 0);
        //hd44780_print(&mt16s2h, buffer);
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
    }

    return 0;
}
