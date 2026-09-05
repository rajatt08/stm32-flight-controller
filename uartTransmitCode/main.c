#include "stm32f407xx.h"

void UART_SendChar(char ch)
{
    while(!(USART2->SR & (1 << 7)));   // Wait until TXE = 1.....here SR is status register...this while loop means jbtk 1 nhi hai tb tk chalate rho

    USART2->DR = ch;    // jse hi 1 ho jaye ch bhej do
}

void UART_SendString(char *str)
{
    while(*str)
    {
        UART_SendChar(*str);
        str++;
    }
}

int main(void)
{
    //Enable clocks
	RCC->AHB1ENR |= (1<<0);   // GPIOA
	RCC->APB1ENR |= (1<<17);  // USART2

   // PA2 Alternate function
	GPIOA->MODER &= ~(3 << (2*2));
	GPIOA->MODER |=  (2 << (2*2));

    //AF7 -> USART2
	GPIOA->AFR[0] &= ~(0xf << 8);
	GPIOA->AFR[0] |=  (7 << 8);


    // Baud Rate = 9600
    USART2->BRR = 0x0683;

    // Enable TX
    USART2->CR1 |= (1 << 3);

    //Enable USART
    USART2->CR1 |= (1 << 13);

    while(1)
    {
       UART_SendString("Hello World\r\n");
       for(int i = 0;i<500000;i++);
    }
}
