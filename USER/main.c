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
可用外部io口：

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

IO 分配：
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

//任务优先级
#define START_TASK_PRIO				3
//任务堆栈大小	
#define START_STK_SIZE 				1024
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

//TOUCH任务
//设置任务优先级
#define TOUCH_TASK_PRIO				4
//任务堆栈大小
#define TOUCH_STK_SIZE				128
//任务控制块
OS_TCB TouchTaskTCB;
//任务堆栈
CPU_STK TOUCH_TASK_STK[TOUCH_STK_SIZE];
//touch任务
void touch_task(void *p_arg);

//LED0任务
//设置任务优先级
#define LED0_TASK_PRIO 				5
//任务堆栈大小
#define LED0_STK_SIZE				128
//任务控制块
OS_TCB Led0TaskTCB;
//任务堆栈
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
//led0任务
void led0_task(void *p_arg);

//EMWINDEMO任务
//设置任务优先级
#define EMWINDEMO_TASK_PRIO			7
//任务堆栈大小
#define EMWINDEMO_STK_SIZE			2048
//任务控制块
OS_TCB EmwindemoTaskTCB;
//任务堆栈
CPU_STK EMWINDEMO_TASK_STK[EMWINDEMO_STK_SIZE];
//emwindemo_task任务
void emwindemo_task(void *p_arg);
void tmr1_callback(void *p_tmr, void *p_arg); 	//定时器1回调函数

OS_TMR 	tmr1;		//定时器1
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
	
	delay_init(168);       	//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	//中断分组配置
	uart_init(115200);    	//串口波特率设置
	//ESP8266_init();
	USART3_init(115200);	    //串口初始化波特率为115200  wifi模块出厂配置的波特率为115200
	TFTLCD_Init();			//初始化LCD
	stepMotor_Init();
	W25QXX_Init();			//初始化W25Q128
	LED_Init();   			//LED初始化
	 
BEEP_Init();
	jumpSign_Init();
	//FSMC_SRAM_Init(); 		//SRAM初始化	
	mem_init(SRAMIN); 		//内部RAM初始化
	//mem_init(SRAMEX); 		//外部RAM初始化
	//mem_init(SRAMCCM);		//CCM初始化
	
	
	exfuns_init();			//为fatfs文件系统分配内存
	f_mount(fs[0],"0:",1);	//挂载SD卡
	f_mount(fs[1],"1:",1);	//挂载FLASH
	font_init();
	
//	while(font_init())		//初始化字库
//	{
//		LCD_ShowString(30,70,200,16,16,"Font Error!");
//		while(SD_Init())	//检测SD卡
//		{
//			LCD_ShowString(30,90,200,16,16,"SD Card Failed!");
//			delay_ms(200);
//			LCD_Fill(30,90,200+30,70+16,WHITE);
//			delay_ms(200);		    
//		}
//		update_font(30,90,16,"0:");	//如果字库不存在就更新字库
//		delay_ms(2000);
//		LCD_Clear(WHITE);	//清屏
//		break;
//	}
	TP_Init();			//初始化触摸屏
	
	OSInit(&err);		//初始化UCOSIII
	OS_CRITICAL_ENTER();//进入临界区
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);  //开启UCOSIII
	while(1);
}

//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC,ENABLE);//开启CRC时钟
	GUI_Init();  			//STemWin初始化
	
		//创建定时器1
	OSTmrCreate((OS_TMR		*)&tmr1,		//定时器1
                (CPU_CHAR	*)"tmr1",		//定时器名字
                (OS_TICK	 )0,			//20*10=200ms
                (OS_TICK	 )10,          //100*10=1000ms  OS_CFG_TMR_TASK_RATE_HZ=100
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //周期模式
                (OS_TMR_CALLBACK_PTR)tmr1_callback,//定时器1回调函数
                (void	    *)0,			//参数为0
                (OS_ERR	    *)&err);		//返回的错误码
								
	
	OS_CRITICAL_ENTER();	//进入临界区
	//STemWin Demo任务	
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
	//触摸屏任务
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
	//LED0任务
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
	 												 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
	OS_CRITICAL_EXIT();	//退出临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}

//EMWINDEMO任务
void emwindemo_task(void *p_arg)
{
	MainTask();
}


//TOUCH任务
void touch_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{
		GUI_TOUCH_Exec();	
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_PERIODIC,&err);//延时5ms
	}
}

//LED0任务
void led0_task(void *p_arg)
{
	OS_ERR err;
	flag_wifi = 0;
	WIFI_Server_Init();		
	flag_wifi = 1;
	OSTmrStart((OS_TMR*)&tmr1,(OS_ERR*)&err);
	while(1)
	{
		//测试蜂鸣器引脚
//		printf("beep1=%d\r\n",BEEP_SIGN1);
//		printf("beep2=%d\r\n",BEEP_SIGN2);
//		printf("beep3=%d\r\n",BEEP_SIGN3);
//		printf("beep4=%d\r\n",BEEP_SIGN4);
		LED0 = !LED0;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_PERIODIC,&err);//延时500ms
	}
}

void USART3_IRQHandler(void)  
{	
	u8 rec_data;
	OSIntEnter();

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断 
		{
				rec_data =(u8)USART_ReceiveData(USART3);         //(USART1->DR) 读取接收到的数据

        if(rec_data=='[')		  	                         //如果是S，表示是命令信息的起始位
				{
					uart_byte_count=0x01; 
				}

			else if(rec_data==']')		                         //如果E，表示是命令信息传送的结束位
				{
					if(strcmp("system",(char *)receive_str)==0){
						LED1=0;	  //点亮LED
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
					}else if(receive_str[0]>'0'&&receive_str[0]<'9'){ //数字
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
//定时器1的回调函数
void tmr1_callback(void *p_tmr, void *p_arg)
{
	speedBuffer++;
}

void EXTI15_10_IRQHandler(void){ //12 14 0 1
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line12)!=RESET){//判断某个线上的中断是否发生
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
				testSpeed = 30/speedBuffer; //单位ml/min
				speedBuffer =0;
				printf("testSpeed:%d\r\n",testSpeed);
			}
		}
	}
		EXTI_ClearITPendingBit(EXTI_Line12); //清除 LINE 上的中断标志位
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
				testSpeed = 30/speedBuffer; //单位ml/min
				speedBuffer =0;
				printf("testSpeed:%d\r\n",testSpeed);
			}
		}
	}
		EXTI_ClearITPendingBit(EXTI_Line14); //清除 LINE 上的中断标志位
	}
	OSIntExit();
}
void EXTI0_IRQHandler(void){
	OSIntEnter();
//	OS_ERR err;
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET){//判断某个线上的中断是否发生
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
					testSpeed = 30/speedBuffer; //单位ml/min
					speedBuffer =0;
					printf("testSpeed:%d\r\n",testSpeed);
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line0); //清除 LINE 上的中断标志位
	}
	OSIntExit();
}
void EXTI1_IRQHandler(void){
		OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET){//判断某个线上的中断是否发生
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
					testSpeed = 30/speedBuffer; //单位ml/min
					speedBuffer =0;
					printf("testSpeed:%d\r\n",testSpeed);
				}
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line1); //清除 LINE 上的中断标志位
	}
	OSIntExit();
}

