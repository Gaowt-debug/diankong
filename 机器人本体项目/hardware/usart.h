#ifndef _USART_
#define _USART_


void USART_INIT(void);
void USART_sendbit(uint8_t data);
void USART_sendstring(char* data);
void USART_pack_get(void);
char USART_pack_staytest(void);
char USART_pack_command(char* command);
void USART_pack_clear(void);
void USART2_INIT(void);

#endif
