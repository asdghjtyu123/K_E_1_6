

#include "stdint.h"

typedef struct {
  volatile uint32_t CLKCFG[116];                 
}PCC_Type;

typedef struct {
    volatile uint32_t PCR[32];                          
    volatile  uint32_t GPCLR;                             
    volatile uint32_t GPCHR;                            
    volatile uint8_t RESERVED_0[24];
    volatile uint32_t ISFR;                              
    volatile uint8_t RESERVED_1[28];
    volatile  uint32_t DFER;                              
    volatile uint32_t DFCR;                             
    volatile uint32_t DFWR;                             
}PORT_Type;

typedef struct {
    volatile uint32_t PDOR;                              
    volatile  uint32_t PSOR;                            
    volatile  uint32_t PCOR;                           
    volatile  uint32_t PTOR;                           
    volatile  uint32_t PDIR;                            
    volatile uint32_t PDDR;                             
} FGPIO_Type_or_GPIO_Type;


/* INIT Fast-GPIO */
#define FGPIOB  ((FGPIO_Type_or_GPIO_Type *)0xF8000040u)
#define FGPIOD  ((FGPIO_Type_or_GPIO_Type *)0xF80000C0u)

/* INIT GPIO */
#define GPIOB  ((FGPIO_Type_or_GPIO_Type *)0x400FF040u)
#define GPIOD  ((FGPIO_Type_or_GPIO_Type *)0x400FF0C0u)

/* INIT PORT */
#define PORTD  ((PORT_Type *)0x4004C000)
#define PORTB  ((PORT_Type *)0x4004A000)

/* HEX -> DEC -> (DEC/4) = index*/
#define PCC                                      ((PCC_Type *)0x40065000)
#define PCC_PORTA_INDEX                          73
#define PCC_PORTB_INDEX                          74
#define PCC_PORTC_INDEX                          75
#define PCC_PORTD_INDEX                          76
#define PCC_PORTE_INDEX                          77

#define DELAY_CNT               (2000000)
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


#define ENCLK_PORTD          *(uint32_t *)0x40065130       
#define ENCLK_PORTB          *(uint32_t *)0x40065128 
#define CLR_BIT(p,n)          ((p) &= ~((1) << (n)))
