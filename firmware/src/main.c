
#include <framework.h>
#include <led.h>
#include <delay.h>


#define DELAY_TIME 250

int main(void){
    init_frameworks();
    led_init();
    delay_initialize();
    while(1){
        led_toggle();
        delay_micros(DELAY_TIME * 1000);
    }
}
