/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hc05_module.h"
#include "myprintf.h"
#include "motor_controller.h"
#include "Buzzer.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Constants.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
/* USER CODE BEGIN PV */
//CONTROL DECLARATION
OdomData_t gOdom;

Pose2D_t     g_pose   = {0};
ControlCmd_t g_ctrl   = {0};
Waypoint path[NUM_WP] = {
    {1.0f, 0.0f},   // 1m straight
	{1.33f, 0.5f},
	{1.83f, 1.0f}// then diagonal
};
int current_wp = 0;

const uint16_t SERVO_CENTER_PWM = 1589;   // example in µs
const float    SERVO_K_PWM_PER_RAD = 411.0f / (26.0f * M_PI/180.0f);
const uint16_t SERVO_MIN_PWM = 1000;
const uint16_t SERVO_MAX_PWM = 2000;
int waypoint_state = 0;
//RTOS QUEUES AND TASK DECLARATION
QueueHandle_t xCanRxQueue;
osThreadId_t canRxTaskHandle;
osThreadId_t actuatorTaskHandle;
osThreadId_t controlTaskHandle;
osThreadId_t debugTaskHandle;


const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 512 * 4,                   // 2 KB of stack
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t actuatorTask_attributes = {
  .name = "actuatorTask",
  .stack_size = 512 * 4,                   // 2 KB of stack
  .priority = (osPriority_t) osPriorityAboveNormal,
};
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 512 * 4,                   // 2 KB of stack
  .priority = (osPriority_t) osPriorityAboveNormal,
};
const osThreadAttr_t debugTask_attributes = {
  .name = "debugTask",
  .stack_size = 512 * 4,                   // 2 KB of stack
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void CanRxTask(void *argument);
void ActuatorTask(void *argument);
void Control_Task(void *argument);
void update_current_waypoint(float tol);
float pure_pursuit_compute_delta(Pose2D_t pose, Waypoint path[],int current_wp);
void Debug_Task(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* --- Calibration --- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {   //Received new message in interruption
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];
        //Check new message is a valid can message
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
        	CanFrame_t frame;   //Create a frame message with our struct
        	frame.id  = rxHeader.Identifier; //Set can ID
        	memcpy(frame.data, rxData, 8); 	 //Copy data to frame data
        	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        	xQueueSendToBackFromISR(xCanRxQueue, &frame, &xHigherPriorityTaskWoken); //Send to queue back
        	portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //return to high priority task

        }
        }
    }

void update_current_waypoint(float tol)
{
    float dx = path[current_wp].x - g_pose.x;
    float dy = path[current_wp].y - g_pose.y;
    float dist = sqrtf(dx*dx + dy*dy);

    // If close enough and we have more waypoints,
    // advance to the next one
    if (dist < tol && current_wp < (NUM_WP - 1)) {
    	playTone(melody,durations,melodysize);
        current_wp++;
    }
}
//13.65
float pure_pursuit_compute_delta(Pose2D_t pose,
                                 Waypoint path[],
                                 int current_wp)
{
    // ---------------------------------------------
    // 1. Find lookahead point along the path
    // ---------------------------------------------
    Waypoint goal = path[current_wp];

    // default goal: last waypoint
    int last_wp = NUM_WP - 1;

    for (int i = current_wp; i <= last_wp; i++) {
        float dx = path[i].x - pose.x;
        float dy = path[i].y - pose.y;
        float dist = sqrtf(dx*dx + dy*dy);

        if (dist >= LOOKAHEAD_DIST) {
            goal = path[i];
            break;
        }
    }

    // ---------------------------------------------
    // 2. Transform goal point into vehicle coordinate frame
    // ---------------------------------------------
    float dx = goal.x - pose.x;
    float dy = goal.y - pose.y;

    float cos_t = cosf(pose.theta);
    float sin_t = sinf(pose.theta);

    // Vehicle frame (x forward, y left)
    float x_cg =  cos_t * dx + sin_t * dy;
    float y_cg = -sin_t * dx + cos_t * dy;

    float Ld = sqrtf(x_cg*x_cg + y_cg*y_cg);
    if (Ld < 0.001f) Ld = 0.001f;

    // ---------------------------------------------
    // 3. Pure Pursuit steering law
    // δ = atan( 2 * L * y / Ld^2 )
    // ---------------------------------------------
    float delta = atan2f(2.0f * WHEELBASE * y_cg, (Ld * Ld));

    // ---------------------------------------------
    // 4. Clamp to mechanical limits
    // ---------------------------------------------
    if (delta >  MAX_STEER_ANGLE) delta =  MAX_STEER_ANGLE;
    if (delta < -MAX_STEER_ANGLE) delta = -MAX_STEER_ANGLE;

    return delta;   // radians
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //START ALL INTERFACES
  startHCrx(&huart2);
  startMotor(&htim13);
  startServo(&htim14);
  startBuzzer(&htim4);
  setMotorStep(0);
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  //QUEUE, INTERRUPT AND TASK INITIALIZATION
  xCanRxQueue = xQueueCreate(32, sizeof(CanFrame_t));
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK){
	  Error_Handler();
  }
  canRxTaskHandle = osThreadNew(CanRxTask, NULL, &canRxTask_attributes);
  actuatorTaskHandle = osThreadNew(ActuatorTask, NULL, &actuatorTask_attributes);
  controlTaskHandle = osThreadNew(Control_Task, NULL, &controlTask_attributes);
  debugTaskHandle   = osThreadNew(Debug_Task, NULL, &debugTask_attributes);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 40;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 3;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 4;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 4;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig = {0};

        // Accept all standard IDs into RX FIFO0
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = 0;
        sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1 = 0x000;  // start ID
        sFilterConfig.FilterID2 = 0x7FF;  // end ID

        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }

        // Optional: global filter for non-matching IDs
        HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_FILTER_REMOTE,
            FDCAN_FILTER_REMOTE
        );

        if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        {
            Error_Handler();
        }

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 199;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 19999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 199;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 19999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CanRxTask(void *argument)
{
	CanFrame_t frame;
	TickType_t lastTimeTicks = 0;
	int32_t    lastTicks     = 0;
    uint8_t    havePrev     = 0;
	const float R = WHEEL_RADIUS_M;
	const float ticksPerRev = (float)TICKS_PER_REV;
    for(;;)
    {
        if (xQueueReceive(xCanRxQueue, &frame, portMAX_DELAY) == pdTRUE) {
            // Check for encoder ID
            if (frame.id == ENCODER_CAN_ID) {

            	int32_t ticks =  //reconstruct encoder message
            	   ((int32_t)frame.data[0] << 0)
            	   | ((int32_t)frame.data[1] << 8)
            	   | ((int32_t)frame.data[2] << 16)
            	   | ((int32_t)frame.data[3] << 24);
                TickType_t now = xTaskGetTickCount(); //get ticks
                if (havePrev) { //check for prev tick count
                    TickType_t dtTicks = now - lastTimeTicks;   // Time difference

                    if (dtTicks > 0) {
                        float dt = (float)dtTicks / (float)configTICK_RATE_HZ; // seconds

                        int32_t dTicks = ticks - lastTicks; //tick difference
                        float rev      = (float)dTicks / ticksPerRev; //revolutions
                        float wheelDist = (2.0f * (float)M_PI * R) * rev; // meters
                        float v        = wheelDist / dt;                  // m/s

                        // Update shared odometry state
                        gOdom.wheel_v_mps = v;
                        gOdom.last_ticks  = ticks;
                    }
                } else {
                    // if theres only one sample
                    havePrev = 1;
                }
                lastTicks     = ticks; //update ticks and time
                lastTimeTicks = now;
            }
            if (frame.id == BNO_CAN_ID){
            	FloatConverter converter;
            	// Fill the binary array directly from the CAN frame
            	converter.binary[0] = frame.data[0];
            	converter.binary[1] = frame.data[1];
            	converter.binary[2] = frame.data[2];
            	converter.binary[3] = frame.data[3];

            	// Read it back as a float automatically
            	gOdom.yaw_val = converter.float_val;
            }

        }
    }
}

void ActuatorTask(void *argument)
{
		const float DT = 0.02f;  // 50 Hz
	    TickType_t last_wake = xTaskGetTickCount();
	    // speed controller state (PI)
	    float speed_integral = 0.0f;

	    for (;;) {
	    	vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
	    	// 1. Snapshot commands and measured velocity
	    	float v_cmd   = g_ctrl.v_cmd;
	    	float delta   = g_ctrl.delta_cmd;
	    	float v_meas  = gOdom.wheel_v_mps;

	    	// 2. SPEED PID → motor PWM
	    	float Kp = 0.5f, Ki = 0.001f;
	    	float error = v_cmd - v_meas;
	    	speed_integral += error * DT;

	    	float u = Kp * error + Ki * speed_integral;

	    	float base_pwm = (v_cmd > 0.0f) ? 0.3f : 0.0f;
	    	float pwm = base_pwm + u;   // 0..1

	    	if (pwm < 0.0f) pwm = 0.0f;
	    	if (pwm > 1.0f) pwm = 1.0f;

	    	uint16_t motor_pwm = (uint16_t)(pwm * MAX_VEL);

	    	 // 3. STEERING: δ → servo PWM

	    	uint16_t steer_pwm = (float)SERVO_CENTER_PWM + SERVO_K_PWM_PER_RAD * delta;
	    	if (steer_pwm < SERVO_MIN_PWM) steer_pwm = SERVO_MIN_PWM;
	    	if (steer_pwm > SERVO_MAX_PWM) steer_pwm = SERVO_MAX_PWM;

	         // 4. Apply to hardware
//	         printf("Motor_pwm = %d steering_pwm = %d \r\n",motor_pwm,steer_pwm);
	    	 setMotorStep(motor_pwm);
	    	 setServo(steer_pwm);
	    }
	}

void Control_Task(void *argument)
{
    const float DT = 0.02f;   // 50 Hz
    const float WAYPOINT_TOL = 0.03f;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));

        // 1. Snapshot sensors
        float v_meas  = gOdom.wheel_v_mps;
        float yaw     = gOdom.yaw_val;

        // 2. Update odometry
        g_pose.theta = yaw;
        g_pose.x += v_meas * cosf(g_pose.theta) * DT;
        g_pose.y += v_meas * sinf(g_pose.theta) * DT;



        // 3. Waypoint management
        //update_current_waypoint(WAYPOINT_TOL);  // same as we wrote before

        	///////////////////////////////////////////////////////////////
        	float dx = path[current_wp].x - g_pose.x;
        	float dy = path[current_wp].y - g_pose.y;
        	float dist = sqrtf(dx*dx + dy*dy);

        	// If close enough and we have more waypoints,
        	    // advance to the next one
        	    if (dist < 0.05f && current_wp < (NUM_WP)) {
        	    	if(current_wp == 2){
        	    		waypoint_state = 1;
        	    	}else{
        	        current_wp++;
        	    	}
        	    }

        	//////////////////////////////////////////////////////////////
      if(!waypoint_state){

        // 4. Pure Pursuit for steering
        float delta_cmd = pure_pursuit_compute_delta(g_pose, path, current_wp);

        // 5. Speed profile
        float v_cmd = 0.08f; // m/s
        float dx = path[current_wp].x - g_pose.x;
        float dy = path[current_wp].y - g_pose.y;
        float dist = sqrtf(dx*dx + dy*dy);
        if (dist < 0.3f) v_cmd = 0.2f;
        if (dist < 0.01f) v_cmd = 0.0f;

        g_ctrl.v_cmd     = v_cmd;
        g_ctrl.delta_cmd = delta_cmd;
      }
      else{
    	  g_ctrl.v_cmd     = 0.0;
    	  g_ctrl.delta_cmd = 0.0;
//    	  playTone(melody,durations,melodysize);
      }
        // 6. Store commands for Actuator_Task

    }
}

void Debug_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100)); // 10 Hz

        sendHC("x=%.2f y=%.2f θ=%.2f° v=%.2f v_cmd=%.2f δ=%.1f°\r\n",
               g_pose.x, g_pose.y,
               g_pose.theta * 180.0f / M_PI,
			   gOdom.wheel_v_mps,
               g_ctrl.v_cmd,
               g_ctrl.delta_cmd * 180.0f / M_PI);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
