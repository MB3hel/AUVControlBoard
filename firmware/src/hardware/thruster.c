/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#include <hardware/thruster.h>
#include <framework.h>



static thr_params_t params = {.pwm_period = 0, .pwm_zero = 0, .pwm_range = 0};

#define PULSE_WIDTH_US(speed)       ((int)(params.pwm_range * speed) + params.pwm_zero)


#ifdef CONTROL_BOARD_V1

void thruster_init(void){
    TCC0_PWMStart();
    TCC1_PWMStart();

    // Zero duty cycle for all thruster PWMs
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL2, 0));
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL3, 0));
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL1, 0));
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL0, 0));
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL3, 0));
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL2, 0));
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL1, 0));
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL0, 0));
}

void thruster_config(thr_params_t p){
    params = p;

    // Period in us but clock 3Mhz so times 3
    while(!TCC0_PWM24bitPeriodSet(params.pwm_period * 3));
    while(!TCC1_PWM24bitPeriodSet(params.pwm_period * 3));
}

void thruster_set(float *speeds){
    // Ignore speed sets if thruster params not set
    if(params.pwm_period == 0)
        return;

    // PWM configured on TCC0 and TCC1 for thrusters
    // PWM setup and clocks configured in generated code
    // Timer count rate = 3MHz = 3 counts / us
    #define COUNT_PER_US                3
    
    
    // THR1 = TCC0[2]
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL2, PULSE_WIDTH_US(speeds[0]) * COUNT_PER_US));

    // THR2 = TCC0[3]
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL3, PULSE_WIDTH_US(speeds[1]) * COUNT_PER_US));

    // THR3 = TCC0[1]
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL1, PULSE_WIDTH_US(speeds[2]) * COUNT_PER_US));

    // THR4 = TCC0[0]
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL0, PULSE_WIDTH_US(speeds[3]) * COUNT_PER_US));

    // THR5 = TCC1[3]
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL3, PULSE_WIDTH_US(speeds[4]) * COUNT_PER_US));

    // THR6 = TCC1[2]
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL2, PULSE_WIDTH_US(speeds[5]) * COUNT_PER_US));

    // THR7 = TCC1[1]
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL1, PULSE_WIDTH_US(speeds[6]) * COUNT_PER_US));

    // THR8 = TCC1[0]
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL0, PULSE_WIDTH_US(speeds[7]) * COUNT_PER_US));
}

#endif // CONTROL_BOARD_V1


#ifdef CONTROL_BOARD_V2

// From stm32cubemx_main
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

void thruster_init(void){
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

    // Zero duty cycle for all thruster PWMs
    TIM3->CCR4 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR1 = 0;
    TIM5->CCR4 = 0;
    TIM5->CCR3 = 0;
    TIM5->CCR2 = 0;
    TIM5->CCR1 = 0;
}

void thruster_config(thr_params_t p){
    params = p;

    // Period in us but clock 3Mhz so times 3
    TIM3->ARR = params.pwm_period * 3;
    TIM5->ARR = params.pwm_period * 3;
}

void thruster_set(float *speeds){
    // Ignore speed sets if thruster params not set
    if(params.pwm_period == 0)
        return;

    // PWM configured on TIM3 and TIM5 for thrusters
    // PWM setup and clocks configured in generated code
    // Timer count rate = 3MHz = 3 counts / us
    #define COUNT_PER_US                3
    #define PULSE_WIDTH_US(speed)       ((int)(params.pwm_range * speed) + params.pwm_zero)

    // THR1 = TIM3[4]
    TIM3->CCR4 = PULSE_WIDTH_US(speeds[0]) * COUNT_PER_US;

    // THR2 = TIM3[3]
    TIM3->CCR3 = PULSE_WIDTH_US(speeds[1]) * COUNT_PER_US;

    // THR3 = TIM3[2]
    TIM3->CCR2 = PULSE_WIDTH_US(speeds[2]) * COUNT_PER_US;

    // THR4 = TIM3[1]
    TIM3->CCR1 = PULSE_WIDTH_US(speeds[3]) * COUNT_PER_US;

    // THR5 = TIM5[4]
    TIM5->CCR4 = PULSE_WIDTH_US(speeds[4]) * COUNT_PER_US;

    // THR6 = TIM5[3]
    TIM5->CCR3 = PULSE_WIDTH_US(speeds[5]) * COUNT_PER_US;

    // THR7 = TIM5[2]
    TIM5->CCR2 = PULSE_WIDTH_US(speeds[6]) * COUNT_PER_US;

    // THR8 = TIM5[1]
    TIM5->CCR1 = PULSE_WIDTH_US(speeds[7]) * COUNT_PER_US;
}

#endif // CONTROL_BOARD_V2
