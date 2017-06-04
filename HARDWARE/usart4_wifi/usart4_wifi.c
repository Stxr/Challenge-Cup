#include "usart4_wifi.h"
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "lcd.h"
#include "beep.h" 

/*********************************************************************************
*********************MCU启明 STM32F407应用开发板(高配版)**************************
**********************************************************************************
* 文件名称: lte.c                                                                *
* 文件简述：WIFI使用                            ·                                *
* 创建日期：2015.10.06                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：利用手机APP控制开发板                                                * 
**********************************************************************************
*********************************************************************************/	

u8 receive_str[USART3_REC_NUM];     //接收缓存数组,最大USART_REC_LEN个字节 
u8 uart_byte_count=0;
u8 APP_mode=0;          //APP控制模式  0：命令控制区  1：接收发送区

//unsigned char MODE[]="AT+CWMODE=3\r\n";
//unsigned char Router[]="AT+CWSAP=\"qiming_wifi\",\"0123456789\",11,4\r\n";  //配置成路由器 名字为qiming_wifi 密码0123456789
//unsigned char RST[]="AT+RST\r\n";
//unsigned char M_Connection[]="AT+CIPMUX=1\r\n";
//unsigned char SERVER[]="AT+CIPSERVER=1,5000\r\n";  //端口号5000
//unsigned char SEND[]="AT+CIPSEND=\r\n";  //AT+CIPSEND= 发送数据
unsigned char MODE[]="AT+CWMODE=3\r\n";
unsigned char Router[]="AT+CWJAP=\"stxr\",\"sunshee123\"\r\n";  //配置成路由器 名字为qiming_wifi 密码0123456789
unsigned char RST[]="AT+RST\r\n";
unsigned char M_Connection[]="AT+CIPMUX=1\r\n";
unsigned char SERVER[]="AT+CIPSTART=\"TCP\",\"192.168.31.204\",8088\r\n";  //端口号5000
unsigned char SEND[]="AT+CIPSEND=\r\n";  //AT+CIPSEND= 发送数据

/****************************************************************************
* 名    称: void USART3_init(u32 bound)
* 功    能：LTE_USART3初始化
* 入口参数：bound：波特率   
* 返回参数：无
* 说    明： 
****************************************************************************/
void USART3_init(u32 bound)
{   
/****************************** 串口4初始化*********************************/
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART4时钟 
	 	USART_DeInit(USART3);  //复位串口4
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB0复用为USART4
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB1复用为USART4
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB0与GPIOB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);          //初始化PA9，PA10
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1	
  USART_Cmd(USART3, ENABLE);  //使能串口4 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);         //开启相关中断
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;      //串口4中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		   //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			   //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	  //根据指定的参数初始化VIC寄存器、
/****************************** 串口4初始化**********************************/  
}

//使能ESP8266 就是置CH_PD为高
void ESP8266_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   //100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);          // 
  GPIO_SetBits(GPIOC, GPIO_Pin_1); 
}

//串口1发送一个字符
void USART3SendChar(u8 ch)
{      
	while((USART3->SR&0x40)==0);  
    USART3->DR = (u8) ch;      
}
/****************************************************************************
* 名    称: void uart1SendChars(u8 *str, u16 strlen)
* 功    能：串口1发送一字符串
* 入口参数：*str：发送的字符串
            strlen：字符串长度
* 返回参数：无
* 说    明： 
****************************************************************************/
void USART3SendChars(u8 *str, u16 strlen)
{ 
	  u16 k= 0 ; 
   do { USART3SendChar(*(str + k)); k++; }   //循环发送,直到发送完毕   
    while (k < strlen); 
} 

void WIFI_Server_Init(void)
{
	//USART3_init(115200);	    //串口初始化波特率为115200  wifi模块出厂配置的波特率为115200
	
	USART3SendChars(MODE,sizeof(MODE));   
	delay_ms(1000);
	
	USART3SendChars(RST,sizeof(RST));     //重启模块
	delay_ms(1000);
	delay_ms(1000);
	
	USART3SendChars(Router,sizeof(Router));   //配置wifi模块成路由器 相应的路由名字跟密码
	delay_ms(1000);                          //到这步手机或者电脑就可以搜到名字为qiming_wifi的wifi
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);

//	USART3SendChars(M_Connection,sizeof(M_Connection));  //开启多连接
//	delay_ms(1000);
//	
 
	USART3SendChars(SERVER,sizeof(SERVER));  //配置成服务器 与 设置端口号5000  
	delay_ms(1000);                         //到这步打开手机APP输入设置的IP跟端口号就可以连接了  wifi模块的IP一般固定为192.168.4.1
	
}
//串口1中断服务程序



