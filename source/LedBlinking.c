

#include "MKE16Z4.h"
#include "MARCO.h"
#define ENCLK_PORTD          *(uint32_t *)0x40065130       
#define ENCLK_PORTB          *(uint32_t *)0x40065128       
void initLed()
{
  
     
    /* Enable Clock for PORTB */
    ENCLK_PORTB |= PORTSHIFT(30);

    ENCLK_PORTD |= PORTSHIFT(30); //PORT D

    PORTD_PCR2 = (1<<8) | (1<<1) | (1<<0) | (1<<19) | (1<<17);  //GPIO input pull up and interupt
    PORTD_PCR3 = (1<<8) | (1<<1) | (1<<0) | (1<<19) | (1<<17); 
    PORTB->PCR[5] |= PORT_PCR_MUX(1);
    PORTB->PCR[4] |= PORT_PCR_MUX(1);
    PORTD->PCR[1] |= PORT_PCR_MUX(1);

    /* Setup button PIND2 and PIND3 as Input Mode */
     GPIOD_PDDR &= ~(1<<2); 
     GPIOD_PDDR &= ~(1<<3);
     
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
        /*__asm("nop");*/
    }
}

void PORTBCD_IRQHandler()
{
  uint8_t mode;

  
  while(1){
  
      if ( PORTD_PCR2 & (1<<24)){
    mode = 1;

  }
  if ( PORTD_PCR3 & (1<<24)){
    mode = 2;

  }
  while (mode==1){

    GPIOB->PTOR |= RED_LED_PIN;
    GPIOB->PTOR |= GREEN_LED_PIN;
    GPIOD->PTOR |= BlUE_LED_PIN;
    delay();
    if ( PORTD_PCR3 & (1<<24)){
      mode = 2;
      GPIOB->PSOR |= GREEN_LED_PIN;
      GPIOB->PSOR |= RED_LED_PIN;
      GPIOD->PSOR |= BlUE_LED_PIN;
      break;
    }
  }
  PORTD->ISFR &= ~PORT_ISFR_ISF_MASK;
  PORTD->ISFR |= PORT_ISFR_ISF(0);
  while (mode==2){
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

 
    if ( PORTD_PCR2 & (1<<24)){
      mode = 1;
      GPIOB->PSOR |= GREEN_LED_PIN;
      GPIOB->PSOR |= RED_LED_PIN;
      GPIOD->PSOR |= BlUE_LED_PIN;
      break;
    }
  }
  PORTD->ISFR &= ~PORT_ISFR_ISF_MASK;
  PORTD->ISFR |= PORT_ISFR_ISF(0);
  }
}

int main(void)
{
    initLed();
    int temp_reg = READreg(NVIC_ISER,~(1 << 26)); 
    temp_reg |= 1 << 26; 
    WRITEreg(NVIC_ISER,temp_reg);
    //NVIC_EnableIRQ(PORTBCD_IRQn);
    while (1)
    {

    }
}