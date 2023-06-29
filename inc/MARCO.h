#define DELAY_CNT               (3000000)
#define GREEN_LED_PIN           (1 << 4)
#define RED_LED_PIN             (1 << 5)
#define BlUE_LED_PIN            (1 << 1)
#define PORTSHIFT(x)            (1 << x)
#define WRITEreg(addr, value)    *((uint32_t *)(addr)) = value
#define READreg(addr, mask)      *((uint32_t *)(addr)) & (mask)
#define	NVIC_ISER				0xE000E100u
#define PORTD_PCR2                  (*(uint32_t *)0x4004C008)
#define PORTD_PCR3                  (*(uint32_t *)0x4004C00C)
#define GPIOD_PDDR                  (*(uint32_t *)0xF80000D4)