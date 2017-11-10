#ifndef ATMEGA64UART_H_
#define ATMEGA64UART_H_

void USART_Init(void);
void USART_SendByte(uint8_t);
void USART0_SendChar(unsigned char);
void USART0_SendString(unsigned char*);
uint8_t USART_ReceiveByte();

#endif /* ATMEGA64UART_H_ */