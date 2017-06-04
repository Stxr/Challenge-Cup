#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H
#include "sys.h"
#include "delay.h"
#include "GUI.h"
#include "24cxx.h"
#include "includes.h"
#include "w25qxx.h" 
#define STEPMOTOR_OUT 1
#define STEPMOTOR_IN 0
#define STEPMOTOR_MIN  1000
#define stepMotor_dir3 PGout(4)
#define stepMotor_step3 PDout(12)
#define stepMotor_enable3 PGout(5)

#define stepMotor_dir2 PDout(6)
#define stepMotor_step2 PDout(7)
#define stepMotor_enable2 PCout(6)

#define stepMotor_dir1 PCout(7)
#define stepMotor_step1 PAout(4)
#define stepMotor_enable1 PGout(15)

#define stepMotor_dir0 PEout(5)
#define stepMotor_step0 PAout(6)
#define stepMotor_enable0 PEout(6)
//__packed typedef struct 
//{
//	u8 dir;
//	u16 distance;
//	u8 isReset;
//}_motor; 
#define RESETBIT 5
#define MOTOR_ENABLE 0
#define MOTOR_DISABLE 1
void stepMotor_Init(void);
void stepMotor_Run(u8 id,u8 dir,int speed);//序号，转动方向，速度 速度越小越快，5ms为一个单位  一次前进0.03m
void stepMotor_Distance(u8 id,int speed,int distance);//distance (0.01mm) 精度+-0.05mm
void stepMotor_Reset(u8 id); //电机复位
int stepMotor_Read(u8 id);
void stepMotor_Write(u8 id,u16 distance);
void stepEnable(u8 id,u8 isEnable);
void setAllMotorMax(void);
void setMotorMax(u8 id);
#endif
