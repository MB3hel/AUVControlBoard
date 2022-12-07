#include <framework.h>
#include <thruster.h>

void thruster_init(void){
    float *zero_speeds = (float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    thruster_set(zero_speeds);
    TCC0_PWMStart();
    TCC1_PWMStart();
}

void thruster_set(float *speeds){
    // PWM configured on TCC0 and TCC1 for thrusters
    // PWM setup and clocks configured in generated code
    // Timer count rate = 3MHz = 3 counts / us
    // PWM frequency = 3MHz / 6000 (period reg) = 500Hz
    // Supported ESCs use pulses from 1100us to 1900us
    // 1500us pulse = 0% speed
    // 1100us pulse = -100% speed
    // 1900us pulse = 100% speed
    #define COUNT_PER_US                3
    #define PULSE_WIDTH_US(speed)       ((int)(400 * speed) + 1500)
    
    // THR1 = TCC0[2]
    TCC0_PWM24bitDutySet(TCC0_CHANNEL2, PULSE_WIDTH_US(speeds[0]) * COUNT_PER_US);

    // THR2 = TCC0[3]
    TCC0_PWM24bitDutySet(TCC0_CHANNEL3, PULSE_WIDTH_US(speeds[1]) * COUNT_PER_US);

    // THR3 = TCC0[1]
    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, PULSE_WIDTH_US(speeds[2]) * COUNT_PER_US);

    // THR4 = TCC0[0]
    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, PULSE_WIDTH_US(speeds[3]) * COUNT_PER_US);

    // THR5 = TCC1[3]
    TCC1_PWM24bitDutySet(TCC1_CHANNEL3, PULSE_WIDTH_US(speeds[4]) * COUNT_PER_US);

    // THR6 = TCC1[2]
    TCC1_PWM24bitDutySet(TCC1_CHANNEL2, PULSE_WIDTH_US(speeds[5]) * COUNT_PER_US);

    // THR7 = TCC1[1]
    TCC1_PWM24bitDutySet(TCC1_CHANNEL1, PULSE_WIDTH_US(speeds[6]) * COUNT_PER_US);

    // THR8 = TCC1[0]
    TCC1_PWM24bitDutySet(TCC1_CHANNEL0, PULSE_WIDTH_US(speeds[7]) * COUNT_PER_US);
}
