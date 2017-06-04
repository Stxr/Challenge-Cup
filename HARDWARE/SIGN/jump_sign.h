#ifndef __JUMP_SIGN_
#define __JUMP_SIGN_
#define JUMP_SIGN1 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_12)
#define BEEP_SIGN1 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_15)

#define JUMP_SIGN2 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_14)
#define BEEP_SIGN2 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_0)

#define JUMP_SIGN3 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_0)
#define BEEP_SIGN3 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_1)

#define JUMP_SIGN4 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_1)
#define BEEP_SIGN4 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_2)
 
#include "sys.h"
void jumpSign_Init(void);
u8 getBeep(u8 id);
void maskOtherExti(u8 id);

#endif
