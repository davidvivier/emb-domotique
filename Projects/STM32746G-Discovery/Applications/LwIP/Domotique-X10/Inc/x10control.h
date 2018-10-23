#ifndef __X10CONTROL_H__
#define __X10CONTROL_H__

#include "main.h"


#define A1
#define A2

// houses
#define HOUSE_A 1
#define HOUSE_B 2
#define HOUSE_C 3
#define HOUSE_D 4


#define HOUSE HOUSE_A

#define UNIT 1


#define BIT_STATE_INDEX 18

void X10_Init(void);
void RF_X10_Send_On(void);
void RF_X10_Send_Off(void);
void TIM3_triggered(void);

#endif
