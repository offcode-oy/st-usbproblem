
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7rsxx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef handle_HPDMA1_Channel0;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Configure the system Power Supply */

  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK)
  {
    /* Initialization error */
    Error_Handler();
  }

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SBS_CLK_ENABLE();

  /* System interrupt init*/

  /* Enable the XSPIM_P1 interface */
  HAL_PWREx_EnableXSPIM1();

  /* Enable the XSPIM_P2 interface */
  HAL_PWREx_EnableXSPIM2();

  /* The CSI is used by the compensation cells and must be enabled before enabling the
     compensation cells.
     For more details refer to RM0477 [SBS I/O compensation cell management] chapter.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure the compensation cell */
  HAL_SBS_ConfigCompensationCell(SBS_IO_XSPI2_CELL, SBS_IO_CELL_CODE, 0U, 0U);

  /* Enable compensation cell */
  HAL_SBS_EnableCompensationCell(SBS_IO_XSPI2_CELL);

  /* wait ready before enabled IO */
  while(HAL_SBS_GetCompensationCellReadyStatus(SBS_IO_XSPI2_CELL_READY) != 1U);

  /* high speed low voltage config */
  HAL_SBS_EnableIOSpeedOptimize(SBS_IO_XSPI2_HSLV);

  HAL_PWREx_EnableUSBHSregulator();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

static uint32_t HAL_RCC_XSPIM_CLK_ENABLED=0;

/**
* @brief XSPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hxspi: XSPI handle pointer
* @retval None
*/
void HAL_XSPI_MspInit(XSPI_HandleTypeDef* hxspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hxspi->Instance==XSPI1)
  {
  /* USER CODE BEGIN XSPI1_MspInit 0 */

  /* USER CODE END XSPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_XSPI1;
    PeriphClkInit.Xspi1ClockSelection = RCC_XSPI1CLKSOURCE_PLL2T;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_XSPIM_CLK_ENABLED++;
    if(HAL_RCC_XSPIM_CLK_ENABLED==1){
      __HAL_RCC_XSPIM_CLK_ENABLE();
    }
    __HAL_RCC_XSPI1_CLK_ENABLE();

    __HAL_RCC_GPIOP_CLK_ENABLE();
    __HAL_RCC_GPIOO_CLK_ENABLE();
    /**XSPI1 GPIO Configuration
    PP2     ------> XSPIM_P1_IO2
    PP3     ------> XSPIM_P1_IO3
    PP0     ------> XSPIM_P1_IO0
    PO0     ------> XSPIM_P1_NCS1
    PP1     ------> XSPIM_P1_IO1
    PO4     ------> XSPIM_P1_CLK
    */
    GPIO_InitStruct.Pin = QSPI_IO2_Pin|QSPI_IO3_Pin|QSPI_MOSI_IO0_Pin|QSPI_IO1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPIM_P1;
    HAL_GPIO_Init(GPIOP, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DISP2_CS_Pin|QSPI_SCLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPIM_P1;
    HAL_GPIO_Init(GPIOO, &GPIO_InitStruct);

    /* XSPI1 DMA Init */
    /* HPDMA1_REQUEST_XSPI1 Init */
    handle_HPDMA1_Channel0.Instance = HPDMA1_Channel0;
    handle_HPDMA1_Channel0.Init.Request = HPDMA1_REQUEST_XSPI1;
    handle_HPDMA1_Channel0.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_HPDMA1_Channel0.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_HPDMA1_Channel0.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_HPDMA1_Channel0.Init.DestInc = DMA_DINC_FIXED;
    handle_HPDMA1_Channel0.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_HPDMA1_Channel0.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_HPDMA1_Channel0.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_HPDMA1_Channel0.Init.SrcBurstLength = 1;
    handle_HPDMA1_Channel0.Init.DestBurstLength = 1;
    handle_HPDMA1_Channel0.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
    handle_HPDMA1_Channel0.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_HPDMA1_Channel0.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_HPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hxspi, hdmatx, handle_HPDMA1_Channel0);

    if (HAL_DMA_ConfigChannelAttributes(&handle_HPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

  /* USER CODE BEGIN XSPI1_MspInit 1 */

  /* USER CODE END XSPI1_MspInit 1 */
  }
  else if(hxspi->Instance==XSPI2)
  {
  /* USER CODE BEGIN XSPI2_MspInit 0 */

  /* USER CODE END XSPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_XSPI2;
    PeriphClkInit.Xspi2ClockSelection = RCC_XSPI2CLKSOURCE_PLL2S;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_XSPIM_CLK_ENABLED++;
    if(HAL_RCC_XSPIM_CLK_ENABLED==1){
      __HAL_RCC_XSPIM_CLK_ENABLE();
    }
    __HAL_RCC_XSPI2_CLK_ENABLE();

    __HAL_RCC_GPION_CLK_ENABLE();
    /**XSPI2 GPIO Configuration
    PN1     ------> XSPIM_P2_NCS1
    PN3     ------> XSPIM_P2_IO1
    PN0     ------> XSPIM_P2_DQS0
    PN11     ------> XSPIM_P2_IO7
    PN10     ------> XSPIM_P2_IO6
    PN9     ------> XSPIM_P2_IO5
    PN2     ------> XSPIM_P2_IO0
    PN6     ------> XSPIM_P2_CLK
    PN8     ------> XSPIM_P2_IO4
    PN4     ------> XSPIM_P2_IO2
    PN5     ------> XSPIM_P2_IO3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_11
                          |GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_2|GPIO_PIN_6
                          |GPIO_PIN_8|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_XSPIM_P2;
    HAL_GPIO_Init(GPION, &GPIO_InitStruct);

  /* USER CODE BEGIN XSPI2_MspInit 1 */

  /* USER CODE END XSPI2_MspInit 1 */
  }

}

/**
* @brief XSPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hxspi: XSPI handle pointer
* @retval None
*/
void HAL_XSPI_MspDeInit(XSPI_HandleTypeDef* hxspi)
{
  if(hxspi->Instance==XSPI1)
  {
  /* USER CODE BEGIN XSPI1_MspDeInit 0 */

  /* USER CODE END XSPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_XSPIM_CLK_ENABLED--;
    if(HAL_RCC_XSPIM_CLK_ENABLED==0){
      __HAL_RCC_XSPIM_CLK_DISABLE();
    }
    __HAL_RCC_XSPI1_CLK_DISABLE();

    /**XSPI1 GPIO Configuration
    PP2     ------> XSPIM_P1_IO2
    PP3     ------> XSPIM_P1_IO3
    PP0     ------> XSPIM_P1_IO0
    PO0     ------> XSPIM_P1_NCS1
    PP1     ------> XSPIM_P1_IO1
    PO4     ------> XSPIM_P1_CLK
    */
    HAL_GPIO_DeInit(GPIOP, QSPI_IO2_Pin|QSPI_IO3_Pin|QSPI_MOSI_IO0_Pin|QSPI_IO1_Pin);

    HAL_GPIO_DeInit(GPIOO, DISP2_CS_Pin|QSPI_SCLK_Pin);

    /* XSPI1 DMA DeInit */
    HAL_DMA_DeInit(hxspi->hdmatx);
  /* USER CODE BEGIN XSPI1_MspDeInit 1 */

  /* USER CODE END XSPI1_MspDeInit 1 */
  }
  else if(hxspi->Instance==XSPI2)
  {
  /* USER CODE BEGIN XSPI2_MspDeInit 0 */

  /* USER CODE END XSPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_XSPIM_CLK_ENABLED--;
    if(HAL_RCC_XSPIM_CLK_ENABLED==0){
      __HAL_RCC_XSPIM_CLK_DISABLE();
    }
    __HAL_RCC_XSPI2_CLK_DISABLE();

    /**XSPI2 GPIO Configuration
    PN1     ------> XSPIM_P2_NCS1
    PN3     ------> XSPIM_P2_IO1
    PN0     ------> XSPIM_P2_DQS0
    PN11     ------> XSPIM_P2_IO7
    PN10     ------> XSPIM_P2_IO6
    PN9     ------> XSPIM_P2_IO5
    PN2     ------> XSPIM_P2_IO0
    PN6     ------> XSPIM_P2_CLK
    PN8     ------> XSPIM_P2_IO4
    PN4     ------> XSPIM_P2_IO2
    PN5     ------> XSPIM_P2_IO3
    */
    HAL_GPIO_DeInit(GPION, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_11
                          |GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_2|GPIO_PIN_6
                          |GPIO_PIN_8|GPIO_PIN_4|GPIO_PIN_5);

  /* USER CODE BEGIN XSPI2_MspDeInit 1 */

  /* USER CODE END XSPI2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
