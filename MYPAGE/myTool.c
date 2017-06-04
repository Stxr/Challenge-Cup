#include "myTool.h"
void mySend(u8 *data){
	USART3SendChars(data,strlen((const char *)data));
}
