/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code implements a http server application based on 
  *          Netconn API of LwIP stack and FreeRTOS. This application uses 
  *          STM32F7xx the ETH HAL API to transmit and receive data. 
  *          The communication is done with a web browser of a remote PC.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "app_ethernet.h"
#include "httpserver-netconn.h"
#include "lcd_log.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct netif gnetif; /* network interface structure */

static TS_StateTypeDef  TS_State;

TIM_HandleTypeDef        Timer3;

extern char buf[];

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void StartThread(void const * argument);
static void BSP_Config(void);
static void Netif_Config(void);
static void MPU_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);

static void RF_Init(void);
static void TIM3_Init(void);

static void TouchscreenThread(void const * argument);

static void LCD_display_buttons(void);


uint8_t CheckForUserInput(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the MPU attributes as Device memory for ETH DMA descriptors */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();  

  /* Initialize LEDs */
  BSP_LED_Init(LED1);
  
  
  /* Configure the system clock to 200 MHz */
  SystemClock_Config(); 

  RF_Init();
	
  TIM3_Init();
	
  X10_Init();


  /* Init thread */
#if defined(__GNUC__)
  osThreadDef(Start, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 5);
#else
  osThreadDef(Start, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
#endif
  
  osThreadCreate (osThread(Start), NULL);
  
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}

/**
  * @brief  Start Thread 
  * @param  argument not used
  * @retval None
  */
static void StartThread(void const * argument)
{ 
  /* Initialize LCD */
  BSP_Config();
	

  osThreadDef(Touchscreen, TouchscreenThread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
  

  /* Create tcp_ip stack thread */
  tcpip_init(NULL, NULL);
  
  /* Initialize the LwIP stack */
  Netif_Config();
  
  /* Initialize webserver demo */
  http_server_netconn_init();
  
  /* Notify user about the network interface config */
  User_notification(&gnetif);
  
#ifdef USE_DHCP
  /* Start DHCPClient */
  osThreadDef(DHCP, DHCP_thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
  osThreadCreate (osThread(DHCP), &gnetif);
#endif

  osThreadCreate(osThread(Touchscreen)  , NULL);

  for( ;; )
  {
    /* Delete the Init Thread */ 
    osThreadTerminate(NULL);
  }
}


static void RF_Init(void) {

  GPIO_InitTypeDef  gpio_init_structure;

  __HAL_RCC_GPIOG_CLK_ENABLE();
  //gpio_rf = LED1_GPIO_PORT;
  /* Enable the GPIO_LED clock */
  //LED1_GPIO_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  gpio_init_structure.Pin = GPIO_PIN_6;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOG, &gpio_init_structure);
  
  /* By default, turn off LED */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

}

static void TIM3_Init(void) {
  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  //uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 10000) - 1;
  uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 10000) - 1;

  /* Set TIMx instance */
  Timer3.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock / 2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  Timer3.Init.Period            = 2 - 1;
  Timer3.Init.Prescaler         = 355555;
  Timer3.Init.ClockDivision     = 0;
  Timer3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Timer3.Init.RepetitionCounter = 0;
  Timer3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&Timer3) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&Timer3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6){ // TIM6
    HAL_IncTick();
  } else {
    TIM3_triggered();
  }
}
/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{ 
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
 
#ifdef USE_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  IP_ADDR4(&ipaddr,IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  IP_ADDR4(&netmask,NETMASK_ADDR0,NETMASK_ADDR1,NETMASK_ADDR2,NETMASK_ADDR3);
  IP_ADDR4(&gw,GW_ADDR0,GW_ADDR1,GW_ADDR2,GW_ADDR3);
#endif /* USE_DHCP */
  
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
  
  /*  Registers the default network interface. */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
}

/**
  * @brief  Initializes the STM327546G-Discovery's LCD  resources.
  * @param  None
  * @retval None
  */
static void BSP_Config(void)
{
  /* Initialize the LCD */
  BSP_LCD_Init();
  
  /* Initialize the LCD Layers */
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);
  
  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);
  
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  
  /* Initialize LCD Log module */
  LCD_LOG_Init();
  
  /* Show Header and Footer texts */
  LCD_LOG_SetHeader((uint8_t *)"Domotique X10");
  LCD_LOG_SetFooter((uint8_t *)"Vivier Industries");
  
  LCD_UsrLog ((char *)"  State: Ethernet Initialization ...\n");
}

/**
  * @brief  Check for user input.
  * @param  None
  * @retval Input state (1 : active / 0 : Inactive)
  */
uint8_t CheckForUserInput(void)
{
  if (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    HAL_Delay(10);
    while (BSP_PB_GetState(BUTTON_KEY) != RESET);
    return 1 ;
  }
  return 0;
}

static void TouchscreenThread(void const * argument) {
	
	uint8_t  status = 0;
  uint16_t x, y;  

	status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  if (status != TS_OK)
  {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"ERROR", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)"Touchscreen cannot be initialized", CENTER_MODE);
  }

  //LCD_UsrLog ((char *)"  TouchscreenThread entered\n");

  LCD_display_buttons();

  while (1)
  {

    if (status == TS_OK)
    {
      /* Check in polling mode in touch screen the touch status and coordinates */
      /* if touch occurred                                                      */
      BSP_TS_GetState(&TS_State);
      if(TS_State.touchDetected)
      {
        /* Get X and Y position of the touch post calibrated */
        x = TS_State.touchX[0];
        y = TS_State.touchY[0];

        if ((y > (BUTTON_ON_Y - BUTTON_ON_RADIUS)) &&
                    (y < (BUTTON_ON_Y + BUTTON_ON_RADIUS)))
                {

          if ((x > (BUTTON_ON_X - BUTTON_ON_RADIUS)) &&
              (x < (BUTTON_ON_X + BUTTON_ON_RADIUS)))
          {
              // button ON touched
              //LCD_UsrLog ((char *)"  button ON touched\n");
              //buf_send = &buf_on;
              //do_send = 1;
              //BSP_LED_Toggle(LED1);
              //BSP_LED_On(LED1);
              RF_X10_Send_On();


/*  LCD_UsrLog ((char *)"\n{");
int i = 0;
  for (i = 0; i < 32; i++) {
    if (buf[i] == 0) {
      LCD_UsrLog ((char *)"0,");
    } else {
      LCD_UsrLog ((char *)"1,");
    }
    if ((i+1)%4 == 0) {
      LCD_UsrLog ((char *)" ");
    }
  }
  LCD_UsrLog ((char *)"}\n");*/


              HAL_Delay(40);

          }
        }

        if ((y > (BUTTON_OFF_Y - BUTTON_OFF_RADIUS)) &&
                    (y < (BUTTON_OFF_Y + BUTTON_OFF_RADIUS)))
                {

          if ((x > (BUTTON_OFF_X - BUTTON_OFF_RADIUS)) &&
              (x < (BUTTON_OFF_X + BUTTON_OFF_RADIUS)))
          {
              // button OFF touched
              //LCD_UsrLog ((char *)"  button OFF touched\n");
              //BSP_LED_Off(LED1);
              //buf_send = &buf_off;
              //do_send = 1;
              RF_X10_Send_Off();

/*  LCD_UsrLog ((char *)"\n{");
int i = 0;
  for (i = 0; i < 32; i++) {
    if (buf[i] == 0) {
      LCD_UsrLog ((char *)"0,");
    } else {
      LCD_UsrLog ((char *)"1,");
    }
    if ((i+1)%4 == 0) {
      LCD_UsrLog ((char *)" ");
    }
  }
  LCD_UsrLog ((char *)"}\n");*/


              HAL_Delay(40);
          }
        }

        LCD_display_buttons();

      }

    }

    if (CheckForUserInput() > 0)
    {
      return;
    }

    HAL_Delay(10);
  }
}

static void LCD_display_buttons(void) {

    // Button ON
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_FillCircle(BUTTON_ON_X, BUTTON_ON_Y, BUTTON_ON_RADIUS);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
    BSP_LCD_DisplayStringAt(BUTTON_ON_X-10, BUTTON_ON_Y-5,  (uint8_t *)"ON", LEFT_MODE);

    // Button OFF
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_FillCircle(BUTTON_OFF_X, BUTTON_OFF_Y, BUTTON_OFF_RADIUS);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(BUTTON_OFF_X-10, BUTTON_OFF_Y-5,  (uint8_t *)"OFF", LEFT_MODE);
    
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
}
// Overrides Drivers\STM32F7xx_HAL_Driver\Src\stm32f7xx_hal.c:288
/*void HAL_IncTick(void) {
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}*/



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
	LCD_UsrLog ((char *)"  Error_Handler() reached :-(\n");
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes as Device for  Ethernet Descriptors in the SRAM1.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Configured Region Size is 256B (size of Rx and Tx ETH descriptors) 
  *       
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();
  
  /* Configure the MPU attributes as Device for Ethernet Descriptors in the SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
