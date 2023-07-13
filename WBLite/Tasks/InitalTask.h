#ifndef INITALTASK_H
#define INITALTASK_H
#include "main.h"

extern float BatVal;

#ifdef __cplusplus
extern "C" {
void Task_Init(void);
void DMA2_Stream4_IRQHandler(void);
}
#endif
#endif