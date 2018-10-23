#ifndef __X10CONTROL_H__
#define __X10CONTROL_H__

#include "main.h"

void X10_Init(void);
void RF_X10_Send_On(void);
void RF_X10_Send_Off(void);
void TIM3_triggered(void);

#endif
