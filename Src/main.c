/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int16_t LeftSpeed=0,RightSpeed=0;
	int16_t left_speed_desire1;		//سʏǚλ̙׈
int16_t right_speed_desire1;		//Ԓʏǚλ̙׈

int32_t speed_error_left_pre1=0;
int32_t speed_error_right_pre1=0;

int16_t left1;           //ʹԃքسʏҠëǷ݆˽ֵ
int16_t right1;          //ʹԃքԒʏҠëǷ݆˽ֵ
int circle=0;
int pause=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int FindBarrier(void);
void Motor(int);
void Run(int,int);
void Back(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  //int LeftBarrier=0,RightBarrier=0.,FrontBarrier=0;    //前方 左右是否有障�? 1为有 0为无

	int Mode;                                            //根据障碍情况调整前进方式 0前进 1左转 2右转 3后�??

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
 HAL_TIM_Base_Start_IT(&htim1);
 HAL_TIM_Base_Init(&htim1);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   //HAL_GPIO_WritePin(GPIOB, 1,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
	  Mode=FindBarrier();
		Motor(Mode);
			HAL_Delay(10);
			//if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_RESET)
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB3 PB4 
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : left_Pin right_Pin */
  GPIO_InitStruct.Pin = left_Pin|right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//tim1时钟中断回调函数
{
	int a=0;
	  if (htim->Instance == TIM1)
		{
			if(circle>=20)
			{
				left1=LeftSpeed;
				
				right1=RightSpeed;
				/*if(left1==0||right1==0)
					a+=1;
				else
					a=0;
				if(a>=3)
					pause=20;
				LeftSpeed=0;
				RightSpeed=0;
				*/
								LeftSpeed=0;
				RightSpeed=0;
				circle=0;
			
			//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			 }
			circle += 1 ;
   //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);//回调函数
		}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//GPIO口中断回调函�?
{
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	if(GPIO_Pin==left_Pin)
		LeftSpeed+=1;
	if(GPIO_Pin==right_Pin)
		RightSpeed+=1;
	
	
}

int FindBarrier(void)
{
	int a=0,b=0,c=0,d=0,e=0,f=0;  //前左，前右，左 右四个方向读取的传感器情况    1为有障碍  0为无障碍
	int mode;
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==GPIO_PIN_RESET)
		a=1;
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==GPIO_PIN_RESET)
		b=1;	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_RESET)
		c=1;
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_RESET)
		d=1;
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_RESET)
		e=1;	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==GPIO_PIN_RESET)
		f=1;
	//if(a==0||b==0)//Mode从0到5依次为直行，左转，右转，左修正，右修正
		if(a==0&&b==0&&c==0&&d==0)
	 
{ mode=0;
if(c==1)
	mode=4;
if(d==1)
	mode=5;}
//	else	if(a==1&&b==0&&c==1)
	//		mode=5;
	//	else if(a==0&&b==1&&d==1)
	//		mode=4；
	else if(e==0)
	{
		mode=1;
	}
	else if(f==0)
	{
		mode=2;
	}
	else
	{
		mode=3;
	}
	//if(c!=0&&d!=0&&a==0&&b==0)
		//return mode=0;
	//if((a!=0&&c!=0)||(b!=0&&d!=0))
		//return mode=3;
	//if(a==0&&b!=0)
		//mode=6;
	//else if(a!=0&&b==0)
		//mode=7;
	return mode;
}

void Motor(int mode)
{
		//int StraightSpeed=50,TurnSpeed=0,RepairSpeed=40;                         //直行速度和转向鿟�?   转向时一侧轮保持原鿿 丿侧轮采取较低的转向鿟度
	  int StraightSpeed=20,TurnSpeed=0,RepairSpeed=10,BackSpeed=-20,SlowTurnSpeed=10;
	switch(mode){
		case 0:Run(StraightSpeed,StraightSpeed);break;
	  case 1: Run(TurnSpeed,StraightSpeed+10);break;
	  case 2:Run(StraightSpeed+10,TurnSpeed);break;
	  case 3:  Run(SlowTurnSpeed,-SlowTurnSpeed);break;
		//case 3:Run(BackSpeed,BackSpeed);HAL_Delay(50);break;
		case 4: Run(RepairSpeed,StraightSpeed);break;
		case 5:Run(StraightSpeed,RepairSpeed);break;
		//case 6:Run(TurnSpeed,StraightSpeed+10);HAL_Delay(30);break;
		//case 7:Run(StraightSpeed+10,TurnSpeed);HAL_Delay(30);break;
	
	}
}
 
void Back()
{

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
	  __HAL_TIM_SET_COMPARE((&htim1),TIM_CHANNEL_1,500);
	  __HAL_TIM_SET_COMPARE((&htim1),TIM_CHANNEL_2,500);
	HAL_Delay(100);
	
	
}
void Run(int left_speed_desire1,int right_speed_desire1 )   //�?左一�?
{
	int error_sum_left1 = 0;
  int error_sum_right1 = 0;

	int32_t speed_error_left1=0;
	int32_t speed_error_right1=0;
	int32_t errD_l1,errD_r1;
        
	static int32_t pwm_left1=0;
  static int32_t pwm_right1=0;
  float pwm_Kp = 16, pwm_Ki = 0.9, pwm_Kd=0;
	
  int ERR_SUM_LIMIT=2000;

	  
		speed_error_left1 = left_speed_desire1 - left1;
		speed_error_right1 = right_speed_desire1 - right1;
        
		errD_l1=speed_error_left1-speed_error_left_pre1;
		errD_r1=speed_error_right1-speed_error_right_pre1;

		error_sum_left1 += speed_error_left1;
		error_sum_right1 += speed_error_right1;

        
		pwm_left1 =  pwm_Kp*speed_error_left1 + pwm_Ki*error_sum_left1 +pwm_Kd*errD_l1;
		pwm_right1 =   pwm_Kp*speed_error_right1 + pwm_Ki*error_sum_right1 +pwm_Kd*errD_r1;

        
		if(error_sum_left1>ERR_SUM_LIMIT)
		{
			error_sum_left1=ERR_SUM_LIMIT;
		}
		else if(error_sum_left1<-ERR_SUM_LIMIT)
		{
			error_sum_left1=-ERR_SUM_LIMIT;
		}
		if(error_sum_right1>ERR_SUM_LIMIT)
		{
			error_sum_right1=ERR_SUM_LIMIT;
		}
		else if(error_sum_right1<-ERR_SUM_LIMIT)
		{
			error_sum_right1=-ERR_SUM_LIMIT;
		}

if(pwm_left1>=0)
{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);//
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
}
else
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
}
if(pwm_right1>=0)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);//
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
}
else{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
}
   __HAL_TIM_SET_COMPARE((&htim1),TIM_CHANNEL_2,pwm_left1);
	 __HAL_TIM_SET_COMPARE((&htim1),TIM_CHANNEL_1,pwm_right1);

	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
