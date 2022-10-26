/**
 * @file ports.c
 * @author Marcus Behel
 */

#include <ports.h>
#include <sam.h>

void ports_pinfunc(uint8_t def, int8_t pinfunc){
    uint8_t port = PORT_GETPORT(def);
    uint8_t pin = PORT_GETPIN(def);
    if(pinfunc == PORT_PINFUNC_GPIO){
        PORT->Group[port].PINCFG[pin].bit.PMUXEN = 0;
    }else{
        PORT->Group[port].PINCFG[pin].bit.PMUXEN = 0;
        if(PORT_GETPIN(def) & 0x1){
            // Odd pin (pmux group pin / 2 = pin >> 1)
            PORT->Group[port].PMUX[pin >> 1].bit.PMUXO = pinfunc;
        }else{
            // Even pin (pmux group pin / 2 = pin >> 1)
            PORT->Group[port].PMUX[pin >> 1].bit.PMUXE = pinfunc;
        }
    }
}

void ports_gpio_dir(uint8_t def, uint8_t dir){
    uint8_t port = PORT_GETPORT(def);
    uint8_t pin = PORT_GETPIN(def);
    if(dir == 0){
        // Input pin
        PORT->Group[port].DIRCLR.reg = 0x1 << pin;
    }else{
        // Output pin
        PORT->Group[port].DIRSET.reg = 0x1 << pin;
    }
    // In both modes, enable input (allows reading OUT pin too)
    PORT->Group[port].PINCFG[pin].bit.INEN = 1;
}

void ports_gpio_pull(uint8_t def, uint8_t pull){
    uint8_t port = PORT_GETPORT(def);
    uint8_t pin = PORT_GETPIN(def);
    switch(pull){
    case PORT_GPIO_PULLOFF:
        PORT->Group[port].PINCFG[pin].bit.PULLEN = 0;
        break;
    case PORT_GPIO_PULLUP:
        PORT->Group[port].PINCFG[pin].bit.PULLEN = 1;
        PORT->Group[port].OUTSET.reg = 0x1 << pin;
        break;
    case PORT_GPIO_PULLDOWN:
        PORT->Group[port].PINCFG[pin].bit.PULLEN = 1;
        PORT->Group[port].OUTCLR.reg = 0x1 << pin;
        break;
    }
}

void ports_gpio_write(uint8_t def, uint8_t out){
    uint8_t port = PORT_GETPORT(def);
    uint8_t pin = PORT_GETPIN(def);
    if(out){
        PORT->Group[port].OUTSET.reg = 0x1 << pin;
    }else{
        PORT->Group[port].OUTCLR.reg = 0x1 << pin;
    }
}

void ports_gpio_set(uint8_t def){
    uint8_t port = PORT_GETPORT(def);
    uint8_t pin = PORT_GETPIN(def);
    PORT->Group[port].OUTSET.reg = 0x1 << pin;
}

void ports_gpio_clear(uint8_t def){
    uint8_t port = PORT_GETPORT(def);
    uint8_t pin = PORT_GETPIN(def);
    PORT->Group[port].OUTCLR.reg = 0x1 << pin;
}

void ports_gpio_toggle(uint8_t def){
    uint8_t port = PORT_GETPORT(def);
    uint8_t pin = PORT_GETPIN(def);
    PORT->Group[port].OUTTGL.reg = 0x1 << pin;
}

uint8_t ports_gpio_read(uint8_t def){
    uint8_t port = PORT_GETPORT(def);
    uint8_t pin = PORT_GETPIN(def);
    return (PORT->Group[port].IN.reg & (0x1 << pin)) >> pin;
}

void ports_init(void){
    // Dotstar CLK pin
    ports_pinfunc(P_DS_CLK, PORT_PINFUNC_GPIO);
    ports_gpio_dir(P_DS_CLK, PORT_GPIO_OUT);
    ports_gpio_clear(P_DS_CLK);

    // Dotstar DAT pin
    ports_pinfunc(P_DS_DAT, PORT_PINFUNC_GPIO);
    ports_gpio_dir(P_DS_DAT, PORT_GPIO_OUT);
    ports_gpio_clear(P_DS_DAT);
}
