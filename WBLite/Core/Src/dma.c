/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* Configure DMA request MEMTOMEM_DMA2_Stream0 */

  /* Select Request */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_MEM2MEM);

  /* Set transfer direction */
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_MEMORY_TO_MEMORY);

  /* Set priority level */
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  /* Set DMA mode */
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  /* Set peripheral increment mode */
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_INCREMENT);

  /* Set memory increment mode */
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  /* Set peripheral data width */
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  /* Set memory data width */
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  /* Enable FIFO mode */
  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* Set FIFO threshold */
  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_0, LL_DMA_FIFOTHRESHOLD_FULL);

  /* Set memory burst size */
  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_0, LL_DMA_MBURST_SINGLE);

  /* Set peripheral burst size */
  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_0, LL_DMA_PBURST_SINGLE);

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

