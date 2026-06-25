#include <stdio.h>
#include <string.h>
#include "arm_math.h"

#include "eeg.h"

int zero_cross(int amp);

q31_t pad[500] = {0};

#define STAGES 6

/* 5 coefficients each stage */
q31_t filter_coeffs[STAGES*5] = {
    1027080473,  // 1 Hz Butterworth high-pass
    -2054160934,
    1027080473,
    2052132228,
    -982447817,
    8396887,    // 60 Hz Butterworth low-pass
    16793773,
    8396887,
    1862647612,
    -822493324,
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

#define THRESHOLD 1200
#define COUNTS 100

int zero_cross(int amp){
    if((amp >= THRESHOLD) || (amp <= -THRESHOLD)) {
        return 1;
    } else {
        return 0;
    }
}

int main(void) {
    int alpha_det = 0;
    arm_biquad_cascade_df1_init_q31(&filter, STAGES, filter_coeffs, filter_state, 1);  
    for(int i = 0; i < 10; i++){
        arm_biquad_cascade_df1_q31(&filter, (q31_t*) (eeg_data + i*500), pad, 500);
        for(int j = 0; j < 500; j++){
            if (j % COUNTS == 0) {
                alpha_det = zero_cross(pad[j]);
            }
            printf("%i\t%li\t%i\n", (i*500 + j)*2, pad[j], alpha_det);
        }
    }

    return 0;
}