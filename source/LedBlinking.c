

#include "MKE16Z4.h"

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

volatile uint32_t mode;
void initLed()
{
  
     
    /* Enable Clock for PORTB */
    PCC->CLKCFG[74] |= PORTSHIFT(30);
    
    /* Enable Clock for PORTD */
    PCC->CLKCFG[76] |= PORTSHIFT(30);
    
    PORTD->PCR[2]= (1<<8) | (1<<1) | (1<<0) | (1<<19) | (1<<17);  //GPIO input pull up and interupt
    PORTD->PCR[3]= (1<<8) | (1<<1) | (1<<0) | (1<<19) | (1<<17); 
    
    PORTB->PCR[5] |= PORTSHIFT(8);
    PORTB->PCR[4] |= PORTSHIFT(8);
    PORTD->PCR[1] |=  PORTSHIFT(8);
    /* Setup button PIND2 and PIND3 as Input Mode */
    CLR_BIT(GPIOD_PDDR,2); 
    CLR_BIT(GPIOD_PDDR,3);

     
     /* Setup PINB4 as Output Mode */
    FGPIOB->PDDR |= RED_LED_PIN | GREEN_LED_PIN ;
    FGPIOD->PDDR |= BlUE_LED_PIN ;
    
    FGPIOB->PSOR |= RED_LED_PIN;
    FGPIOB->PSOR |= GREEN_LED_PIN;
    FGPIOD->PSOR |= BlUE_LED_PIN;
}

void delay()
{
    uint32_t i;
    for (i = 0; i < DELAY_CNT; i++)
    {
        __asm("nop");
    }
}

void PORTBCD_IRQHandler()
{
  if ( PORTD_PCR2 & (1<<24)){
    mode = 1;
  }
  if ( PORTD_PCR3 & (1<<24)){ 
    mode = 2;
  }


  PORTD->ISFR &= ~PORT_ISFR_ISF_MASK;
  PORTD->ISFR |= PORT_ISFR_ISF(0);
  
}

int main(void)
{
    initLed();
    int temp_reg = 0; 
    temp_reg |= 1 << 26; 
    WRITEreg(NVIC_ISER,temp_reg);
    //NVIC_EnableIRQ(PORTBCD_IRQn);
    while (1)
    {

      if (mode==1){
        GPIOB->PTOR |= RED_LED_PIN;
        GPIOB->PTOR |= GREEN_LED_PIN;
        GPIOD->PTOR |= BlUE_LED_PIN;
        delay();
      }
      if (mode==2){
        GPIOB->PCOR |= RED_LED_PIN;
        GPIOB->PSOR |= GREEN_LED_PIN;
        GPIOD->PSOR |= BlUE_LED_PIN;
        delay();
        GPIOB->PSOR |= RED_LED_PIN;
        GPIOB->PCOR |= GREEN_LED_PIN;
        GPIOD->PSOR |= BlUE_LED_PIN;
        delay();
        GPIOB->PSOR |= RED_LED_PIN;
        GPIOB->PSOR |= GREEN_LED_PIN;
        GPIOD->PCOR |= BlUE_LED_PIN;
        delay();
      }
    }
}