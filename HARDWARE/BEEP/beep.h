#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h" 
#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////	 

//LED�˿ڶ���
#define BEEP PGout(7)	// ����������IO 

//��������
void BEEP_Init(void); //��ʼ��		 				    
void BEEP_ms(u16 time);
#endif

















