#include "gpio_bitband.h"

int main(void) {

    // Enable GPIOC clock via bit-banding (atomic, no read-modify-write)
    gpio_clock_enable(GPIO_PORT_C);

    // Configure PC13 as push-pull output, 50 MHz
    gpio_pin_config(GPIO_PORT_C, 13, GPIO_MODE_OUTPUT_50MHZ, GPIO_CNF_OUTPUT_PP);

    // Blink loop using bit-band toggle
    while (1) {
        gpio_pin_toggle(GPIO_PORT_C, 13);
        for (volatile int i = 0; i < 100000; i++);
    }

    return 0;
}
