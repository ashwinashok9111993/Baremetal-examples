#define RCC_APB2ENR   (*(volatile unsigned int *)0x40021018)
#define GPIOC_CRH     (*(volatile unsigned int *)0x40011004)
#define GPIOC_ODR     (*(volatile unsigned int *)0x4001100C)


int main(void) {

    // Enable GPIOC clock
    RCC_APB2ENR |= (1 << 4);
    
    // Configure PC13 as output (50MHz, push-pull)
    GPIOC_CRH &= ~(0xF << 20);
    GPIOC_CRH |= (0x3 << 20);
    
    // Blink loop
    while(1) {
        GPIOC_ODR ^= (1 << 13);
        for(volatile int i = 0; i < 100000; i++);
    }

    return 0;
}
