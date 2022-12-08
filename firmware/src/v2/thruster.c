#include <framework.h>
#include <thruster.h>

// From stm32cubemx_main
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

void thruster_init(void){
    float *zero_speeds = (float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    thruster_set(zero_speeds);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

void thruster_set(float *speeds){
    // PWM configured on TIM3 and TIM5 for thrusters
    // PWM setup and clocks configured in generated code
    // Timer count rate = 3MHz = 3 counts / us
    // PWM frequency = 3MHz / 6000 (period in auto reload reg) = 500Hz
    // Supported ESCs use pulses from 1100us to 1900us
    // 1500us pulse = 0% speed
    // 1100us pulse = -100% speed
    // 1900us pulse = 100% speed
    #define COUNT_PER_US                3
    #define PULSE_WIDTH_US(speed)       ((int)(400 * speed) + 1500)

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