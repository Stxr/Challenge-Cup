#include "sys.h"
#include "delay.h"
#include "stdlib.h"
#include "usart.h"
#include "sram.h"
#include "malloc.h"
#include "ILI93xx.h"
#include "led.h"
#include "key.h"
#include "timer.h"
#include "touch.h"
#include "sdio_sdcard.h"
#include "GUI.h"
#include "ff.h"
#include "exfuns.h"
#include "w25qxx.h"
#include "fontupd.h"
#include "includes.h"
#include "EmWinHZFont.h"
#include "WM.h"
#include "BUTTON.h"
#include "mypage.h"
#include "usart4_wifi.h"
/************************************************
�����ⲿio�ڣ�

PA 2  3 4 6 
PB 6 7	
PC 6(U6_TX) 7 11 12 13 
PD 6 7 11 12 13
PE 5 6
PF 0 1 2 3 4 12 13 14 15
PG 0 1 3 4 5 15

PA 2  3 4 
PB 6 
PC 0 2 3  11 12 13
PD  11 12 13
PF  3 4 13 
PG 3  

IO ���䣺
           dir      enable     step
motor1     PG4    	PG5					PD12
motor2		 PD6      PC6         PD7
motor3     PC7      PG15         PA4
motor4     PE5      PE6         PA6

					 exti     beep
speed1     PF12     PF15
speed2 		 PF14     PF0
speed3		 PG0      PF1
speed4		 PG1	    PF2
************************************************/

//�������ȼ�
#define START_TASK_PRIO				3
//�����ջ��С	
#define START_STK_SIZE 				1024
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

//TOUCH����
//�����������ȼ�
#define TOUCH_TASK_PRIO				4
//�����ջ��С
#define TOUCH_STK_SIZE				128
//������ƿ�
OS_TCB TouchTaskTCB;
//�����ջ
CPU_STK TOUCH_TASK_STK[TOUCH_STK_SIZE];
//touch����
void touch_task(void *p_arg);

//LED0����
//�����������ȼ�
#define LED0_TASK_PRIO 				5
//�����ջ��С
#define LED0_STK_SIZE				128
//������ƿ�
OS_TCB Led0TaskTCB;
//�����ջ
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
//led0����
void led0_task(void *p_arg);

//EMWINDEMO����
//�����������ȼ�
#define EMWINDEMO_TASK_PRIO			7
//�����ջ��С
#define EMWINDEMO_STK_SIZE			2048
//������ƿ�
OS_TCB EmwindemoTaskTCB;
//�����ջ
CPU_STK EMWINDEMO_TASK_STK[EMWINDEMO_STK_SIZE];
//emwindemo_task����
void emwindemo_task(void *p_arg);
void tmr1_callback(void *p_tmr, void *p_arg); 	//��ʱ��1�ص�����

OS_TMR 	tmr1;		//��ʱ��1
int testSpeed;
u8 flag_wifi =0;
u8 flag_page=0;
u8 flag_pagechange = 0;
u8 flag_speed;
u8 speedBuffer=0;
extern float dataDisplay[4][5];
int main(void)
 {
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);       	//��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	//�жϷ�������
	uart_init(115200);    	//���ڲ���������
	//ESP8266_init();
	USART3_init(115200);	    //���ڳ�ʼ��������Ϊ115200  wifiģ��������õĲ�����Ϊ115200
	TFTLCD_Init();			//��ʼ��LCD
	stepMotor_Init();
	W25QXX_Init();			//��ʼ��W25Q128
	LED_Init();   			//LED��ʼ��
	 
BEEP_Init();
	jumpSign_Init();
	//FSMC_SRAM_Init(); 		//SRAM��ʼ��	
	mem_init(SRAMIN); 		//�ڲ�RAM��ʼ��
	//mem_init(SRAMEX); 		//�ⲿRAM��ʼ��
	//mem_init(SRAMCCM);		//CCM��ʼ��
	
	
	exfuns_init();			//Ϊfatfs�ļ�ϵͳ�����ڴ�
	f_mount(fs[0],"0:",1);	//����SD��
	f_mount(fs[1],"1:",1);	//����FLASH
	font_init();
	
//	while(font_init())		//��ʼ���ֿ�
//	{
//		LCD_ShowString(30,70,200,16,16,"Font Error!");
//		while(SD_Init())	//���SD��
//		{
//			LCD_ShowString(30,90,200,16,16,"SD Card Failed!");
//			delay_ms(200);
//			LCD_Fill(30,90,200+30,70+16,WHITE);
//			delay_ms(200);		    
//		}
//		update_font(30,90,16,"0:");	//����ֿⲻ���ھ͸����ֿ�
//		delay_ms(2000);
//		LCD_Clear(WHITE);	//����
//		break;
//	}
	TP_Init();			//��ʼ��������
	
	OSInit(&err);		//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();//�����ٽ���
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);  //����UCOSIII
	while(1);
}

//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC,ENABLE);//����CRCʱ��
	GUI_Init();  			//STemWin��ʼ��
	
		//������ʱ��1
	OSTmrCreate((OS_TMR		*)&tmr1,		//��ʱ��1
                (CPU_CHAR	*)"tmr1",		//��ʱ������
                (OS_TICK	 )0,			//20*10=200ms
                (OS_TICK	 )10,          //100*10=1000ms  OS_CFG_TMR_TASK_RATE_HZ=100
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //����ģʽ
                (OS_TMR_CALLBACK_PTR)tmr1_callback,//��ʱ��1�ص�����
                (void	    *)0,			//����Ϊ0
                (OS_ERR	    *)&err);		//���صĴ�����
								
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	//STemWin Demo����	
	OSTaskCreate((OS_TCB*     )&EmwindemoTaskTCB,		
				 (CPU_CHAR*   )"Emwindemo task", 		
                 (OS_TASK_PTR )emwindemo_task, 			
                 (void*       )0,					
                 (OS_PRIO	  )EMWINDEMO_TASK_PRIO,     
                 (CPU_STK*    )&EMWINDEMO_TASK_STK[0],	
                 (CPU_STK_SIZE)EMWINDEMO_STK_SIZE/10,	
                 (CPU_STK_SIZE)EMWINDEMO_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void*       )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR*     )&err);
	//����������
	OSTaskCreate((OS_TCB*     )&TouchTaskTCB,		
				 (CPU_CHAR*   )"Touch task", 		
                 (OS_TASK_PTR )touch_task, 			
                 (void*       )0,					
                 (OS_PRIO	  )TOUCH_TASK_PRIO,     
                 (CPU_STK*    )&TOUCH_TASK_STK[0],	
                 (CPU_STK_SIZE)TOUCH_STK_SIZE/10,	
                 (CPU_STK_SIZE)TOUCH_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void*       )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR*     )&err);			 
	//LED0����
	OSTaskCreate((OS_TCB*     )&Led0TaskTCB,		
				 (CPU_CHAR*   )"Led0 task", 		
                 (OS_TASK_PTR )led0_task, 			
                 (void*       )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK*    )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void*       )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR*     )&err);	
	 												 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}

//EMWINDEMO����
void emwindemo_task(void *p_arg)
{
	MainTask();
}


//TOUCH����
void touch_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{
		GUI_TOUCH_Exec();	
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_PERIODIC,&err);//��ʱ5ms
	}
}

//LED0����
void led0_task(void *p_arg)
{
	OS_ERR err;
	flag_wifi = 0;
	WIFI_Server_Init();		
	flag_wifi = 1;
	OSTmrStart((OS_TMR*)&tmr1,(OS_ERR*)&err);
	while(1)
	{
		//���Է���������
//		printf("beep1=%d\r\n",BEEP_SIGN1);
//		printf("beep2=%d\r\n",BEEP_SIGN2);
//		printf("beep3=%d\r\n",BEEP_SIGN3);
//		printf("beep4=%d\r\n",BEEP_SIGN4);
		LED0 = !LED0;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_PERIODIC,&err);//��ʱ500ms
	}
}

void USART3_IRQHandler(void)  
{	
	u8 rec_data;
	OSIntEnter();

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж� 
		{
				rec_data =(u8)USART_ReceiveData(USART3);         //(USART1->DR) ��ȡ���յ�������

        if(rec_data=='[')		  	                         //�����S����ʾ��������Ϣ����ʼλ
				{
					uart_byte_count=0x01; 
				}

			else if(rec_data==']')		                         //���E����ʾ��������Ϣ���͵Ľ���λ
				{
					if(strcmp("system",(char *)receive_str)==0){
						LED1=0;	  //����LED
						flag_page =1;
						flag_pagechange = 1;
					}else if(strcmp("pageHome",(char *)receive_str)==0){
						LED1=1;	   
						flag_page =2;
						flag_pagechange = 1;
					}else if(strcmp("shuye",(char *)receive_str)==0){
						LED1=1;	   
						flag_page =3;
						flag_pagechange = 1;
					}else if(strcmp("display",(char *)receive_str)==0){
						LED2=1;	   
						flag_page =4;
						flag_pagechange = 1;
					}else if(receive_str[0]>'0'&&receive_str[0]<'9'){ //����
						switch(receive_str[0]){
							case '1':
								sscanf((char *) receive_str,"%*d,%d,%d,%d,%d,%d",&dataDisplay[0][0],&dataDisplay[0][1],&dataDisplay[0][2],&dataDisplay[0][3],&dataDisplay[0][4]);	
								break;
							case '2':
								sscanf((char *)receive_str,"%*d,%d,%d,%d,%d,%d",&dataDisplay[1][0],&dataDisplay[1][1],&dataDisplay[1][2],&dataDisplay[1][3],&dataDisplay[1][4]);	
								break;
							case '3':
								sscanf((char *)receive_str,"%*d,%d,%d,%d,%d,%d",&dataDisplay[2][0],&dataDisplay[2][1],&dataDisplay[2][2],&dataDisplay[2][3],&dataDisplay[2][4]);	
								break;
							case '4':
								sscanf((char *)receive_str,"%*d,%d,%d,%d,%d,%d",&dataDisplay[3][0],&dataDisplay[3][1],&dataDisplay[3][2],&dataDisplay[3][3],&dataDisplay[3][4]);	
								break;	
						}	
					}
					for(uart_byte_count=0;uart_byte_count<32;uart_byte_count++)receive_str[uart_byte_count]=0x00;
					uart_byte_count=0;    
				}				  
			else if((uart_byte_count>0)&&(uart_byte_count<=USART3_REC_NUM))
				{
				   receive_str[uart_byte_count-1]=rec_data;
				   uart_byte_count++;
				}                		 
   }
		OSIntExit();	
} 
//��ʱ��1�Ļص�����
void tmr1_callback(void *p_tmr, void *p_arg)
{
	speedBuffer++;
}

void EXTI15_10_IRQHandler(void){ //12 14 0 1
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line12)!=RESET){//�ж�ĳ�����ϵ��ж��Ƿ���
		if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_12)==SET){
			delay_ms(1);
		if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_12)==SET){
			printf("pf12\r\n");
			LED2=!LED2;
			if(flag_speed==0){
				flag_speed=1;
				speedBuffer=0; 
			}else{
				flag_speed=0;
				testSpeed = 30/speedBuffer; //��λml/min
				speedBuffer =0;
				printf("testSpeed:%d\r\n",testSpeed);
			}
		}
	}
		EXTI_ClearITPendingBit(EXTI_Line12); //��� LINE �ϵ��жϱ�־λ
	}else if(EXTI_GetITStatus(EXTI_Line14)!=RESET){
		if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_14)==SET){
			delay_ms(1);
			if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_14)==SET){
//				printf("pf14\r\n");
				LED1=!LED1;
				if(flag_speed==0){
				flag_speed=1;
				speedBuffer=0; 
			}else{
				flag_speed=0;
				testSpeed = 30/speedBuffer; //��λml/min
				speedBuffer =0;
				printf("testSpeed:%d\r\n",testSpeed);
			}
		}
	}
		EXTI_ClearITPendingBit(EXTI_Line14); //��� LINE �ϵ��жϱ�־λ
	}
	OSIntExit();
}
void EXTI0_IRQHandler(void){
	OSIntEnter();
//	OS_ERR err;
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET){//�ж�ĳ�����ϵ��ж��Ƿ���
		if(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_0)==SET){
			delay_ms(1);
			if(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_0)==SET){
				LED2=!LED2;
				printf("pg0\r\n");
				if(flag_speed==0){
					flag_speed=1;
					speedBuffer=0; 
				}else{
					flag_speed=0;
					testSpeed = 30/speedBuffer; //��λml/min
					speedBuffer =0;
					printf("testSpeed:%d\r\n",testSpeed);
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line0); //��� LINE �ϵ��жϱ�־λ
	}
	OSIntExit();
}
void EXTI1_IRQHandler(void){
		OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET){//�ж�ĳ�����ϵ��ж��Ƿ���
		if(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_1)==SET){
			delay_ms(1);
			if(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_1)==SET){
				LED2=!LED2;
				printf("pg1\r\n");
				if(flag_speed==0){
					flag_speed=1;
					speedBuffer=0; 
				}else{
					flag_speed=0;
					testSpeed = 30/speedBuffer; //��λml/min
					speedBuffer =0;
					printf("testSpeed:%d\r\n",testSpeed);
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line1); //��� LINE �ϵ��жϱ�־λ
	}
	OSIntExit();
}

