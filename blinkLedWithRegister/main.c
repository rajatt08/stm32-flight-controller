#include "stm32f407xx.h"   // this file contains RCC, GPIO, TIMERS nd all

int main(void)
{
   RCCC->AHB1ENR |= (1<<3);           // BEFORE USING GPIOD, WE MUST ENABLE ITS CLOCK
   RCC->AHB1ENR |= (1<<0);            // // BEFORE USING GPIOA, WE MUST ENABLE ITS CLOCK
  GPIOD->MODER &= ~(3 << (12*2));     // we are clearing first
  GPIOD->MODER |=  (1 << (12*2));     // now for output data register we have to give 01

  GPIOA->MODER &= ~(3 << (0*2));     // we are clearing first for PA0
  GPIOA->MODER |=  (0 << (0*2));     // NOW for input data register we have to give 00 jrurat nhi h wse iss line ki

  while(1){
	  if(GPIOA->IDR & (1 << 0))  // if getting button pressed then led ON
	  {
		  GPIOD->ODR |= (1<<12);
	  }
	  else{                       // otherwise led off
		  GPIOD->ODR &= ~(1<<12);
	  }
  }
}
