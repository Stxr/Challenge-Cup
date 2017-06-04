#ifndef __USART4_WIFI_H
#define __USART4_WIFI_H
#include "stdio.h"	
#include "delay.h" 

//////////////////////////////////////////////////////////////////////////////////	 

extern u8 APP_mode;

#define USART3_REC_NUM  			200  	//定义最大接收字节数 200
extern u8 uart_byte_count;          //uart_byte_count要小于USART_REC_LEN
extern u8 receive_str[USART3_REC_NUM]; 

void ESP8266_init(void);
void USART3_init(u32 bound);
void WIFI_Server_Init(void);
void USART3SendChars(u8 *str, u16 strlen);

void uart4SendChars(u8 *str, u16 strlen);

#endif


