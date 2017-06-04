#include "usart4_wifi.h"
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "lcd.h"
#include "beep.h" 

/*********************************************************************************
*********************MCU���� STM32F407Ӧ�ÿ�����(�����)**************************
**********************************************************************************
* �ļ�����: lte.c                                                                *
* �ļ�������WIFIʹ��                            ��                                *
* �������ڣ�2015.10.06                                                           *
* ��    ����V1.0                                                                 *
* ��    �ߣ�Clever                                                               *
* ˵    ���������ֻ�APP���ƿ�����                                                * 
**********************************************************************************
*********************************************************************************/	

u8 receive_str[USART3_REC_NUM];     //���ջ�������,���USART_REC_LEN���ֽ� 
u8 uart_byte_count=0;
u8 APP_mode=0;          //APP����ģʽ  0�����������  1�����շ�����

//unsigned char MODE[]="AT+CWMODE=3\r\n";
//unsigned char Router[]="AT+CWSAP=\"qiming_wifi\",\"0123456789\",11,4\r\n";  //���ó�·���� ����Ϊqiming_wifi ����0123456789
//unsigned char RST[]="AT+RST\r\n";
//unsigned char M_Connection[]="AT+CIPMUX=1\r\n";
//unsigned char SERVER[]="AT+CIPSERVER=1,5000\r\n";  //�˿ں�5000
//unsigned char SEND[]="AT+CIPSEND=\r\n";  //AT+CIPSEND= ��������
unsigned char MODE[]="AT+CWMODE=3\r\n";
unsigned char Router[]="AT+CWJAP=\"stxr\",\"sunshee123\"\r\n";  //���ó�·���� ����Ϊqiming_wifi ����0123456789
unsigned char RST[]="AT+RST\r\n";
unsigned char M_Connection[]="AT+CIPMUX=1\r\n";
unsigned char SERVER[]="AT+CIPSTART=\"TCP\",\"192.168.31.204\",8088\r\n";  //�˿ں�5000
unsigned char SEND[]="AT+CIPSEND=\r\n";  //AT+CIPSEND= ��������

/****************************************************************************
* ��    ��: void USART3_init(u32 bound)
* ��    �ܣ�LTE_USART3��ʼ��
* ��ڲ�����bound��������   
* ���ز�������
* ˵    ���� 
****************************************************************************/
void USART3_init(u32 bound)
{   
/****************************** ����4��ʼ��*********************************/
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART4ʱ�� 
	 	USART_DeInit(USART3);  //��λ����4
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB0����ΪUSART4
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB1����ΪUSART4
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB0��GPIOB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);          //��ʼ��PA9��PA10
   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������1	
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���4 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);         //��������ж�
	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;      //����4�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		   //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			   //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	  //����ָ���Ĳ�����ʼ��VIC�Ĵ�����
/****************************** ����4��ʼ��**********************************/  
}

//ʹ��ESP8266 ������CH_PDΪ��
void ESP8266_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   //100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);          // 
  GPIO_SetBits(GPIOC, GPIO_Pin_1); 
}

//����1����һ���ַ�
void USART3SendChar(u8 ch)
{      
	while((USART3->SR&0x40)==0);  
    USART3->DR = (u8) ch;      
}
/****************************************************************************
* ��    ��: void uart1SendChars(u8 *str, u16 strlen)
* ��    �ܣ�����1����һ�ַ���
* ��ڲ�����*str�����͵��ַ���
            strlen���ַ�������
* ���ز�������
* ˵    ���� 
****************************************************************************/
void USART3SendChars(u8 *str, u16 strlen)
{ 
	  u16 k= 0 ; 
   do { USART3SendChar(*(str + k)); k++; }   //ѭ������,ֱ���������   
    while (k < strlen); 
} 

void WIFI_Server_Init(void)
{
	//USART3_init(115200);	    //���ڳ�ʼ��������Ϊ115200  wifiģ��������õĲ�����Ϊ115200
	
	USART3SendChars(MODE,sizeof(MODE));   
	delay_ms(1000);
	
	USART3SendChars(RST,sizeof(RST));     //����ģ��
	delay_ms(1000);
	delay_ms(1000);
	
	USART3SendChars(Router,sizeof(Router));   //����wifiģ���·���� ��Ӧ��·�����ָ�����
	delay_ms(1000);                          //���ⲽ�ֻ����ߵ��ԾͿ����ѵ�����Ϊqiming_wifi��wifi
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);

//	USART3SendChars(M_Connection,sizeof(M_Connection));  //����������
//	delay_ms(1000);
//	
 
	USART3SendChars(SERVER,sizeof(SERVER));  //���óɷ����� �� ���ö˿ں�5000  
	delay_ms(1000);                         //���ⲽ���ֻ�APP�������õ�IP���˿ںžͿ���������  wifiģ���IPһ��̶�Ϊ192.168.4.1
	
}
//����1�жϷ������



