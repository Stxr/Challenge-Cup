#include "step_motor.h"
/*
	
	id		�����ʼֵ   ����
	
	1        1500        2200                374  3ml     
	
	2				1600				 2200             351  8ml
	
	3				1700				 2300             365����ٶ�    420��ȫѹ��
	
	4				1000				 2000             315             380
*/
const u16 stepMotorMin[]={1500,1600,1700,1600};
const u16 stepMotorMax[]={2500,2420,2150,2050}; //max+100
u16 motor_distance[4];
void stepMotor_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);  //ʹ��GPIOA GPIOEʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5 |GPIO_Pin_15;         //LED0��LED1��LED2��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                   //����
  GPIO_Init(GPIOG, &GPIO_InitStructure);                         //��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_12;         //LED0��LED1��LED2��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                   //����
  GPIO_Init(GPIOD, &GPIO_InitStructure);                         //��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 |GPIO_Pin_8|GPIO_Pin_6;         //LED0��LED1��LED2��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                   //����
  GPIO_Init(GPIOC, &GPIO_InitStructure);                         //��ʼ��GPIO
	
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 |GPIO_Pin_6;         //LED0��LED1��LED2��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                   //����
  GPIO_Init(GPIOE, &GPIO_InitStructure);                         //��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_4;         //LED0��LED1��LED2��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                   //����
  GPIO_Init(GPIOA, &GPIO_InitStructure);                         //��ʼ��GPIO
	
//	//����ر�
	stepMotor_enable1 = 1;
	stepMotor_enable2 = 1;
	stepMotor_enable3 = 1;
	stepMotor_enable0 = 1;
//	
//	printf("stepmotor%d distance:%d\r\n",1,stepMotor_Read(1));
//	printf("stepmotor%d distance:%d\r\n",2,stepMotor_Read(2));
//	printf("stepmotor%d distance:%d\r\n",3,stepMotor_Read(3));
//	printf("stepmotor%d distance:%d\r\n",4,stepMotor_Read(4));
//	printf("stepMotor_Init OK!\r\n");
}
void stepMotor_Run(u8 id,u8 dir,int speed){
	switch (id) {
		case 0:
	//		stepMotor_enable0 =1;
			stepMotor_dir0=dir;
			stepMotor_step0=1;
			GUI_Delay(speed);
			stepMotor_step0=0;
			GUI_Delay(speed);
	//		stepMotor_enable0 =0;
			break;
		case 1:
	//		stepMotor_enable1 =1;
			stepMotor_dir1=dir;
			stepMotor_step1=1;
			GUI_Delay(speed);
			stepMotor_step1=0;
			GUI_Delay(speed);
	//	stepMotor_enable1 =0;
			break;
		case 2:
	//		stepMotor_enable2 =1;
			stepMotor_dir2=dir;
			stepMotor_step2=1;
			GUI_Delay(speed);
			stepMotor_step2=0;
			GUI_Delay(speed);
//	stepMotor_enable2 =0;
			break;
		case 3:
	//		stepMotor_enable3 =1;
			stepMotor_dir3=dir;
			stepMotor_step3=1;
			GUI_Delay(speed);
			stepMotor_step3=0;
			GUI_Delay(speed);
	//		stepMotor_enable3 =0;
			break;
		default:
			break;
	}
}
void stepMotor_Distance(u8 id,int speed,int distance){
	int i=0;
	u8 dir=0;
	i=stepMotor_Read(id);
	stepEnable(id,MOTOR_ENABLE);
	if(distance<0){ //�������������ԭ�������ϼ�
		dir=STEPMOTOR_IN;
		if(i+distance<stepMotorMin[id]){//���ʣ�µ�ֵ�ȼ�С��ֵ��
			distance=stepMotorMin[id]-i;
		}
	}else{//�����
		dir=STEPMOTOR_OUT;
		if(i+distance>stepMotorMax[id]){ //���������Χ��
			distance =stepMotorMax[id]-i;
		}
	}
//	stepMotor_Write(id,i+distance,);
	stepMotor_Write(id,i+distance); //16λ
	printf("stepInit:0X%0X\r\n",stepMotor_Read(RESETBIT));//��ӡ��ʼ����Ϣ
	printf("stepmotor%d distance:%d\r\n",id,stepMotor_Read(id));
	for(i=0;i<abs(distance)/3;i++){
		stepMotor_Run(id,dir,speed);
	}
	stepEnable(id,MOTOR_DISABLE);
}
void stepMotor_Reset(u8 id){
	int i=0;
	i=stepMotor_Read(RESETBIT);
	stepEnable(id,MOTOR_ENABLE);
	if((i&(0xf0|1<<(id-1)))==(0xf0|1<<(id-1))){ //���֮ǰ�Ѿ���ʼ���� ����λΪ1
		printf("�Ѿ���ʼ����\r\n");
		stepMotor_Distance(id,1,-stepMotorMax[id]); //
	}else{//��һ�γ�ʼ��
		printf("��һ�γ�ʼ��\r\n");
		stepMotor_Write(RESETBIT,(i|0xf0)|1<<(id-1));//��ʼ�� ������λ��һλ��ʼ���Ͱ���һλ��һ
		for(i=0;i<1200;i++){ //��λΪ0
			stepMotor_Run(id,STEPMOTOR_IN,2); //���ٸ�λ
		}
	for(i=0;i<abs(STEPMOTOR_MIN)/3;i++){ //��λ����Сֵ
		stepMotor_Run(id,STEPMOTOR_OUT,1);
	}
		stepMotor_Write(id,STEPMOTOR_MIN);//��ʼΪ0
	}
	stepEnable(id,MOTOR_DISABLE);
	printf("stepInit%d:0X%0X \r\n",id,stepMotor_Read(RESETBIT));//��ӡ��ʼ����Ϣ
	printf("stepMotor_Reset OK!\r\n");

}
//����distance
int stepMotor_Read(u8 id){
	W25QXX_Read((u8*)&motor_distance[id],MOTORSAVE+id*(sizeof(u16)+1),sizeof(u16));
	return motor_distance[id];
}

//д������
void stepMotor_Write(u8 id,u16 distance){
	motor_distance[id] = distance;
	W25QXX_Write((u8*)&motor_distance[id],MOTORSAVE+id*(sizeof(u16)+1),sizeof(u16));
}
void stepEnable(u8 id,u8 isEnable){
	switch(id){
		case 0:
			stepMotor_enable0 = isEnable;
			break;
		case 1:
			stepMotor_enable1 = isEnable;
			break;
		case 2:
			stepMotor_enable2 = isEnable;
			break;
		case 3:
			stepMotor_enable3 = isEnable;
			break;
	}
}
void setAllMotorMax(){
	stepMotor_Distance(0,1,stepMotorMax[0]-stepMotor_Read(0)-100);
	stepMotor_Distance(1,1,stepMotorMax[1]-stepMotor_Read(1)-100);
	stepMotor_Distance(2,1,stepMotorMax[2]-stepMotor_Read(2)-100);
	stepMotor_Distance(3,1,stepMotorMax[3]-stepMotor_Read(3)-100);
}
void setMotorMax(u8 id){
	stepMotor_Distance(id,1,stepMotorMax[id]-stepMotor_Read(id)-100);
}