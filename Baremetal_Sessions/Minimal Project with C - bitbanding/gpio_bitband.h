#ifndef GPIO_BITBAND_H
#define GPIO_BITBAND_H

/*
 * gpio_bitband.h - GPIO HAL using Cortex-M3 Bit-Banding
 *
 * Bit-band regions on STM32F103 (Cortex-M3):
 *   Peripheral bit-band region:  0x40000000 - 0x400FFFFF  (1 MB)
 *   Peripheral bit-band alias:   0x42000000 - 0x43FFFFFF  (32 MB)
 *   SRAM bit-band region:        0x20000000 - 0x200FFFFF  (1 MB)
 *   SRAM bit-band alias:         0x22000000 - 0x23FFFFFF  (32 MB)
 *
 * Alias formula:
 *   alias_addr = alias_base + (byte_offset * 32) + (bit_number * 4)
 *
 * Each bit in the bit-band region maps to a full 32-bit word in the alias
 * region. Writing 1 or 0 to the alias word atomically sets or clears the
 * corresponding bit — no read-modify-write required.
 */

/* ------------------------------------------------------------------ */
/*  Bit-band macros                                                    */
/* ------------------------------------------------------------------ */

#define PERIPH_BASE        0x40000000U
#define PERIPH_BB_BASE     0x42000000U

/* Convert a peripheral register address + bit number to its bit-band alias */
#define BITBAND_PERIPH(reg_addr, bit) \
    (*(volatile unsigned int *)(PERIPH_BB_BASE + \
        (((unsigned int)(reg_addr) - PERIPH_BASE) * 32U) + \
        ((bit) * 4U)))

/* ------------------------------------------------------------------ */
/*  RCC register addresses                                             */
/* ------------------------------------------------------------------ */

#define RCC_BASE           0x40021000U
#define RCC_APB2ENR        (*(volatile unsigned int *)(RCC_BASE + 0x18U))
#define RCC_APB2ENR_ADDR   (RCC_BASE + 0x18U)

/* RCC_APB2ENR clock-enable bit positions */
#define RCC_IOPAEN_BIT     2U
#define RCC_IOPBEN_BIT     3U
#define RCC_IOPCEN_BIT     4U
#define RCC_IOPDEN_BIT     5U

/* ------------------------------------------------------------------ */
/*  GPIO register addresses                                            */
/* ------------------------------------------------------------------ */

#define GPIOA_BASE         0x40010800U
#define GPIOB_BASE         0x40010C00U
#define GPIOC_BASE         0x40011000U
#define GPIOD_BASE         0x40011400U

/* Register offsets from GPIOx_BASE */
#define GPIO_CRL_OFF       0x00U
#define GPIO_CRH_OFF       0x04U
#define GPIO_IDR_OFF       0x08U
#define GPIO_ODR_OFF       0x0CU
#define GPIO_BSRR_OFF      0x10U
#define GPIO_BRR_OFF       0x14U

/* Convenience register accessors */
#define GPIO_REG(base, off)  (*(volatile unsigned int *)((base) + (off)))

/* ------------------------------------------------------------------ */
/*  GPIO port enumeration                                              */
/* ------------------------------------------------------------------ */

typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D
} gpio_port_t;

/* ------------------------------------------------------------------ */
/*  GPIO mode (simplified for output use)                              */
/* ------------------------------------------------------------------ */

typedef enum {
    GPIO_MODE_INPUT       = 0x0,   /* Input mode (reset state)         */
    GPIO_MODE_OUTPUT_10MHZ = 0x1,  /* Output, max speed 10 MHz         */
    GPIO_MODE_OUTPUT_2MHZ  = 0x2,  /* Output, max speed 2 MHz          */
    GPIO_MODE_OUTPUT_50MHZ = 0x3   /* Output, max speed 50 MHz         */
} gpio_mode_t;

typedef enum {
    GPIO_CNF_OUTPUT_PP  = 0x0,     /* General purpose push-pull        */
    GPIO_CNF_OUTPUT_OD  = 0x1,     /* General purpose open-drain       */
    GPIO_CNF_INPUT_ANALOG = 0x0,   /* Analog input                     */
    GPIO_CNF_INPUT_FLOAT  = 0x1,   /* Floating input (reset state)     */
    GPIO_CNF_INPUT_PUPD   = 0x2    /* Input with pull-up / pull-down   */
} gpio_cnf_t;

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static inline unsigned int _gpio_base(gpio_port_t port) {
    switch (port) {
        case GPIO_PORT_A: return GPIOA_BASE;
        case GPIO_PORT_B: return GPIOB_BASE;
        case GPIO_PORT_C: return GPIOC_BASE;
        case GPIO_PORT_D: return GPIOD_BASE;
        default:          return GPIOC_BASE;
    }
}

static inline unsigned int _gpio_rcc_bit(gpio_port_t port) {
    switch (port) {
        case GPIO_PORT_A: return RCC_IOPAEN_BIT;
        case GPIO_PORT_B: return RCC_IOPBEN_BIT;
        case GPIO_PORT_C: return RCC_IOPCEN_BIT;
        case GPIO_PORT_D: return RCC_IOPDEN_BIT;
        default:          return RCC_IOPCEN_BIT;
    }
}

/* ------------------------------------------------------------------ */
/*  HAL functions (all use bit-banding where applicable)               */
/* ------------------------------------------------------------------ */

/*
 * Enable the clock for a GPIO port using bit-banding.
 * Instead of RCC_APB2ENR |= (1 << bit)  (read-modify-write),
 * we write a single 1 to the bit-band alias — atomic, one store.
 */
static inline void gpio_clock_enable(gpio_port_t port) {
    BITBAND_PERIPH(RCC_APB2ENR_ADDR, _gpio_rcc_bit(port)) = 1U;
}

/*
 * Configure a pin's mode and type.
 * Pins 0-7 use CRL, pins 8-15 use CRH.
 * Each pin occupies 4 bits: MODE[1:0] CNF[1:0].
 * This part still uses read-modify-write because we set a 4-bit field.
 */
static inline void gpio_pin_config(gpio_port_t port, unsigned int pin,
                                   gpio_mode_t mode, gpio_cnf_t cnf) {
    unsigned int base = _gpio_base(port);
    unsigned int offset = (pin < 8U) ? GPIO_CRL_OFF : GPIO_CRH_OFF;
    unsigned int pos = (pin % 8U) * 4U;
    volatile unsigned int *cr = (volatile unsigned int *)(base + offset);

    *cr &= ~(0xFU << pos);                       /* clear MODE + CNF  */
    *cr |= (((unsigned int)cnf << 2U) | (unsigned int)mode) << pos;
}

/*
 * Set a single pin HIGH using bit-banding on ODR.
 * Writes 1 to the alias word — no read-modify-write.
 */
static inline void gpio_pin_set(gpio_port_t port, unsigned int pin) {
    BITBAND_PERIPH(_gpio_base(port) + GPIO_ODR_OFF, pin) = 1U;
}

/*
 * Clear a single pin LOW using bit-banding on ODR.
 * Writes 0 to the alias word — no read-modify-write.
 */
static inline void gpio_pin_clear(gpio_port_t port, unsigned int pin) {
    BITBAND_PERIPH(_gpio_base(port) + GPIO_ODR_OFF, pin) = 0U;
}

/*
 * Toggle a pin using bit-banding.
 * Read the current ODR bit via its alias, write back the inverse.
 * Still atomic per-bit — only touches one bit, no masking needed.
 */
static inline void gpio_pin_toggle(gpio_port_t port, unsigned int pin) {
    volatile unsigned int *alias =
        (volatile unsigned int *)(PERIPH_BB_BASE +
            ((_gpio_base(port) + GPIO_ODR_OFF - PERIPH_BASE) * 32U) +
            (pin * 4U));
    *alias ^= 1U;
}

/*
 * Read a single input pin via bit-banding on IDR.
 * Returns 0 or 1.
 */
static inline unsigned int gpio_pin_read(gpio_port_t port, unsigned int pin) {
    return BITBAND_PERIPH(_gpio_base(port) + GPIO_IDR_OFF, pin);
}

#endif /* GPIO_BITBAND_H */
