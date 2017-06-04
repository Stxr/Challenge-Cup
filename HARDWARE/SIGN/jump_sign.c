#include "jump_sign.h"
void jumpSign_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;


  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG, ENABLE);//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //KEY0 KZZEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource12);//PE2 连接到中断线2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource14);//PE2 连接到中断线2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource0);//PE2 连接到中断线2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource1);//PE2 连接到中断线2


	EXTI_InitStructure.EXTI_Line = EXTI_Line12|GPIO_Pin_1|GPIO_Pin_14|GPIO_Pin_0 ;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
  EXTI_Init(&EXTI_InitStructure);//配置
	 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
}
u8 getBeep(u8 id){
	switch(id){
		case 0:
			return BEEP_SIGN1;
			break;
		case 1:
			return BEEP_SIGN2;
			break;
		case 2:
			return BEEP_SIGN3;
			break;
		case 3:
			return BEEP_SIGN4;
			break;
	}
}
void maskOtherExti(u8 id){  //12 14 0 1
			switch(id){
		case 0:
			EXTI->IMR|=1<<12; //开启12线中断
			EXTI->IMR&=~(1<<14); // 屏蔽14线中断
			EXTI->IMR&=~(1<<0);
			EXTI->IMR&=~(1<<1);
			break;
		case 1:
			EXTI->IMR|=1<<14;
			EXTI->IMR&=~(1<<12);
			EXTI->IMR&=~(1<<0);
			EXTI->IMR&=~(1<<1);
			break;
		case 2:
			EXTI->IMR|=1<<0;
			EXTI->IMR&=~(1<<12);
			EXTI->IMR&=~(1<<14); // 屏蔽14线中断
			EXTI->IMR&=~(1<<1);
			break;
		case 3:
			EXTI->IMR|=1<<1;
			EXTI->IMR&=~(1<<12);
			EXTI->IMR&=~(1<<0);
			EXTI->IMR&=~(1<<14); // 屏蔽14线中断
			break;
	}
}


