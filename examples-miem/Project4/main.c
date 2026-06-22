#include <stdio.h>
#include <string.h>
#include "arm_math.h"

#include "eeg.h"

int zero_cross_at(int* ar, int start, int length);

q31_t pad[500] = {0};

#define STAGES 6

/* 5 coefficients each stage */
q31_t filter_coeffs[STAGES*5] = {
    1027080468,  // 1 Hz Butterworth high-pass
    -2054160935,
    1027080468,
    2052132225,
    -982447822,
    29794094,    // 60 Hz Butterworth low-pass
    59588187,
    29794094,
    1586460278,
    -631894829,
    1057895428,  // 50 Hz notch
    -2015208065,
    1057895428,
    2017613793,
    -1041084974,
    1055838009,
    -2005272285,
    1055838009,
    2002709966,
    -1039060248,
    1069836749,
    -2036220071,
    1069836749,
    2043955687,
    -1065779498,
    1069095489,
    -2032256517,
    1069095489,
    2024104799,
    -1065041049
};
q31_t filter_state[STAGES*4]; /* 4 ints for each state */
arm_biquad_casd_df1_inst_q31  filter;

#define ARRAY_LENGTH 500
#define ADC_FREQ 500
#define PERIODS 3

int zero_cross_at(int* ar, int start, int length){
    //length = -length;
    //ar - Массив значений отсчётов
    float counts = 0; //1 период альфа-сигнала
    int window = 0; //Окно, в котором ищем закономерность
    int old_num = 0; //Значение для определения направления сигнала
    int new_num = 0;//Значение для определения направления сигнала
    int cross = 0;//Количество пресечений
    
    for (int freq = 5; freq < 20; freq++) {
        counts = ADC_FREQ / freq; 
        window = (int)(counts * PERIODS);
        
        ////Проверяем, что хватает данных от start до конца массива
        //if (start + window > length) {
        //    continue;
        //}
        
        old_num = ar[start];
        cross = 0;
        
        for (int x = start + 1; x < start + window; x++) {
            new_num = ar[x];
            //Отсчитываем пресечение нуля как индикатор половины периода
            if ((old_num > 0 && new_num < 0) || (old_num < 0 && new_num > 0)) { 
                cross++;
            }
            old_num = new_num;
        }
        
        //Проверяем, есть ли требуемое количество периодов
        if ((cross > (PERIODS*2)-2) && cross < (PERIODS*2)+2) {
            return freq; //Сигнал обнаружен, частота определена
        }
    }
    return 0; //Сигнал не обнаружен
}

int main(void) {
    arm_biquad_cascade_df1_init_q31(&filter, STAGES, filter_coeffs, filter_state, 1);  
    for(int i = 0; i < 10; i++){
        arm_biquad_cascade_df1_q31(&filter, (q31_t*) (eeg_data + i*500), pad, 500);
        for(int j = 0; j < 500; j++){
            int alpha_detected = zero_cross_at((int*)pad, j, ARRAY_LENGTH);
            printf("%i\t%li\t%i\n", (i*500 + j)*2, pad[j], alpha_detected);
        }
    }

    return 0;
}