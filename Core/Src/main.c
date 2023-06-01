/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include"math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint32_t adcbuffer[2];
uint32_t adc_new;
uint16_t speed;
uint32_t adc;
float current;
float voltage;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

//Devir Ölçme İçin
uint32_t ICValue = 10000, temp1, pwm;
uint16_t  devirDakika;
uint32_t devirFaktoru;
uint16_t preScaler = 1024;
uint16_t ICbuffer[5] = {0,0,0,0,0};
uint16_t sumIC=0, ortalamaIC=10000;
uint8_t i, tim3_overflow_flag = 0i;

//PID için
float kp = 0.15;  // PID sabitleri
float ki = 0.47;//0.2;
float kd = 0.02; //sonra bakılacak

float error = 0.0;  // hata değeri
float derivative = 0.0;  // türev değeri
float setpoint ;  // hedef hız değeri
float pwm_output = 0.0;  // pwm çıkış değeri
uint32_t last_time = 0;

float amplification;

uint32_t pwm_max = 65535;
uint32_t pwm1;
uint16_t duty_cycle;
uint8_t switch1_durum = 0;
uint8_t switch2_durum = 0;

uint32_t zaman = 0, son_zaman = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void read_switch_states(uint8_t *switch1_durum, uint8_t *switch2_durum); //switchleri okuyon fonk.


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //adc okuma.
{
	if(hadc->Instance == ADC1)
	{
		speed = adcbuffer[0];
		adc = adcbuffer[1];
	}
}

void display_values(int duty_cycle, int current, int devirDakika, int setpoint_int) {
    HAL_Delay(50);
    char adc1[5], adc2[5], adc3[5], adc4[5];

    adc1[4] = '\0';// ekrandaki eski sayıyı silmek için.
    snprintf(adc1, 5, "%04d", duty_cycle);
    SSD1306_GotoXY (40, 0);
    SSD1306_Puts ( adc1 , &Font_11x18, 1);

    adc2[4] = '\0';
    snprintf(adc2, 5, "%04d", current);
    SSD1306_GotoXY (30, 20);
    SSD1306_Puts ( adc2 , &Font_11x18, 1);

    adc3[4] = '\0';
    snprintf(adc3, 5, "%04d", devirDakika);
    SSD1306_GotoXY (40, 40);
    SSD1306_Puts (adc3, &Font_11x18, 1);

    adc4[4] = '\0';
    snprintf(adc4, 5, "%04d", setpoint_int);
    SSD1306_GotoXY (95, 25);
    SSD1306_Puts ( adc4 , &Font_7x10, 1);
    SSD1306_UpdateScreen();
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


    SSD1306_Init();

    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // Start timer in interrupt mode
    HAL_TIM_Base_Start(&htim3); // Start timer in counter mode

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_ADC_Start_DMA(&hadc1, adcbuffer, 2);

    devirFaktoru = (60000000/preScaler)*36;


 	SSD1306_GotoXY (0,0);
 	SSD1306_Puts ("PWM= ", &Font_11x18, 1);
   	SSD1306_GotoXY (0,20);
   	SSD1306_Puts ("mA= ", &Font_11x18, 1);
 	SSD1306_GotoXY (0,40);
 	SSD1306_Puts ("RPM= ", &Font_11x18, 1);
	SSD1306_GotoXY (75,25);
 	SSD1306_Puts ("SP= ", &Font_7x10, 1);
   	SSD1306_UpdateScreen();
   	HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  voltage = speed*0.8056640625; //mV cinsinden
	  if (voltage <= 60) {
	    amplification = -(0.00004*voltage*voltage*voltage) + (0.0033*voltage*voltage) + (0.0348*voltage) + 5.6288;
	  } else if (voltage <= 180) {
	    amplification = -(0.0001*voltage*voltage) + (0.0501*voltage)+ 8.5763;
	  } else if (voltage <= 400) {
	    amplification = (0.0069*voltage) + 12.6904;
	  } else {
	    amplification = 15.9;
	  }
	  current = ((voltage)/amplification)/0.150; //mA cinsinden




	   if ((HAL_GetTick() - son_zaman) > 500) { // motor durduğunda hatalı değer vermesin sıfırlasın.
	        ICValue = 0;
	        devirDakika = 0;
	    }
      devirDakika = devirFaktoru/ortalamaIC;
      sumIC -= ICbuffer[4];

      for (i = 4; i > 0; --i){
	      ICbuffer[i] = ICbuffer[i - 1];
      }
      ICbuffer[0] = ICValue;
      sumIC += ICValue;
      ortalamaIC = sumIC / 5;



       setpoint = (4096-adc) * 0.9765625;// 0-4096 arası adc değeri 0-4000 rpm arasına ayarlanmıştır.
       int setpoint_int = (int) floor(setpoint); // küsüratı yuvarla
       if (setpoint_int > 4000) {
    	   setpoint_int = 4000;
        }
       else if (setpoint_int < 500){
    	   if ((switch1_durum && !switch2_durum) || (!switch1_durum && switch2_durum)){

    		   setpoint_int = 500;
    	   }
    	   else {
    	           setpoint_int = 0;
    	           }

       }


      error = setpoint_int - devirDakika;   // hata değeri
      static float integral = 0;
      static float last_error = 0;
      uint32_t current_time = HAL_GetTick(); // zamanı ölç
      float dt = (current_time - last_time) / 1000.0; //  ms to s
      last_time = current_time;
       // integral değerini hesapla
       integral += error*dt;
       // türev değerini hesapla
       derivative = (error - last_error)/dt;
       // PID çıkışını hesapla
       pwm_output = (kp * error + ki * integral + kd * derivative)*0.256; // buradaki değer 0-4000 arası oluyor, bunu 0-1024 arasına sınırlıyoruz.
       pwm = 1024-pwm_output; // çıkışı tersle
       duty_cycle = (1024-pwm)*0.9765625; // ekrana yazdıracağımız 0-1024 değerini 0-1000 arasına sınırlıyoruz.
       if (duty_cycle < 0) {
    	   duty_cycle = 0;
       }
       else if (duty_cycle > 1000) {
    	   duty_cycle = 1000;
       }
       // PWM çıkışını sınırla (20 - 1024 arasında)
       if (pwm < 20) {
         pwm = 20;
       }
       else if (pwm > 1024) {
         pwm = 1024;
       }
      // son hata değerini güncelle
        last_error = error;





       display_values(duty_cycle, current, devirDakika, setpoint_int); //ekrana yazdır.
       read_switch_states(&switch1_durum, &switch2_durum);  // switchlerin durumlarını kontrol et.

    if (switch1_durum && !switch2_durum) {      // Switch1  set, Switch2 reset, Motor Yönü saat yönü
     	SSD1306_GotoXY (90,5);
     	SSD1306_Puts ("CW  ", &Font_7x10, 1); // Ekrana motorun saat yönünde olduğunu yazdır
     	SSD1306_UpdateScreen();

 		HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_SET);
 		HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET);

   	    HAL_GPIO_TogglePin(DIR1LED_GPIO_Port, DIR1LED_Pin);
 	    HAL_GPIO_WritePin(DIR2LED_GPIO_Port, DIR2LED_Pin, GPIO_PIN_RESET);

 	   	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, pwm_max); // pwm_max değeri timerların %100 görev saykılıyla çalışması ve lojik 1 olarak davranması için.
 	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
    }

    else if (!switch1_durum && switch2_durum) {      // Switch2 set, Switch1 reset, Motor Yönü saat yönü tersi
     	       SSD1306_GotoXY (90,5);
     	       SSD1306_Puts ("CCW ", &Font_7x10, 1); // Ekrana motorun saat yönünün tersinde olduğunu yazdır
     	       SSD1306_UpdateScreen();

        	   HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET);
        	   HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_SET);

        	   HAL_GPIO_WritePin(DIR1LED_GPIO_Port, DIR1LED_Pin, GPIO_PIN_RESET);
        	   HAL_GPIO_TogglePin(DIR2LED_GPIO_Port, DIR2LED_Pin);

        	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
        	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, pwm_max);
           }

    else {
               // İki switch de set ya da reset, Motoru durdur.
	           SSD1306_GotoXY (90,5);
	           SSD1306_Puts ("STOP", &Font_7x10, 1); // Ekrana motorun durduğunu yazdır
	           SSD1306_UpdateScreen();

        	   HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,GPIO_PIN_RESET);
        	   HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,GPIO_PIN_RESET);

        	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, pwm_max);
        	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, pwm_max);

        		HAL_GPIO_WritePin(DIR1LED_GPIO_Port, DIR1LED_Pin, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(DIR2LED_GPIO_Port, DIR2LED_Pin, GPIO_PIN_SET);

           }

    {


       	}

    }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1023;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR2LED_Pin|DIR1LED_Pin|DIR1_Pin|DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR2LED_Pin DIR1LED_Pin DIR1_Pin DIR2_Pin */
  GPIO_InitStruct.Pin = DIR2LED_Pin|DIR1LED_Pin|DIR1_Pin|DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */



/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //TIM3 yükselen kenar oku.
{
  if (htim->Instance == TIM3){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			temp1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if(tim3_overflow_flag == 1){
				temp1 = 65535;
				tim3_overflow_flag = 0;
			}
			if(temp1>100) {
				ICValue = temp1;
				son_zaman = zaman;
				zaman = HAL_GetTick();
			}
		   else {
			   ICValue = 0;
			   devirDakika = 0;
			            }
			TIM3->CNT=0;
		}


  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		ICValue = 65535;
		tim3_overflow_flag = 1;
	}
}
void read_switch_states(uint8_t *switch1_durum, uint8_t *switch2_durum) {
    *switch1_durum = HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET;
    *switch2_durum = HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == GPIO_PIN_RESET;
}


/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
