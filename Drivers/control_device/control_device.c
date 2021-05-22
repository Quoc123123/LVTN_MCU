/*******************************************************************************
* Title                 :   Driver for R305
* Filename              :   control_devive.h
* Author                :   QuocBrave
* Origin Date           :   03/09/2020
* Notes                 :   None
*******************************************************************************/

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <control_device.h>
#include "main.h"
#include "log_debug.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define SPEAKER_GPIO_Port           	(GPIOD)
#define SPEAKER_GPIO_Pin                (GPIO_PIN_12)

#define RELAY_GPIO_Port           		(GPIOD)
#define RELAY_GPIO_Pin            		(GPIO_PIN_1)



/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/
#define PWM_TIMER_NUM                   (TIM4)
#define PWM_TIMER_CLOCK_SPEED           (108000000)
#define PWM_TIMER_PRESCALER             (108)
#define PWM_FREQUENCE                   (999)


/******************************************************************************
* MACROS
*******************************************************************************/

/******************************************************************************
* TYPEDEFS
*******************************************************************************/

/******************************************************************************
* VARIABLES
*******************************************************************************/
TIM_HandleTypeDef htim4;
static bool speaker_is_start = false;


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void ctrl_dev_gpio_init(void);
static void ctrl_dev_gpio_deinit(void);
static void ctrl_dev_timer_init(void);
static void ctrl_dev_timer_deinit(void);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : ctrl_dev_init
* Brief    : 
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void ctrl_dev_init(void)
{
    ctrl_dev_gpio_init();
    ctrl_dev_timer_init();
    PRINT_INFO_LOG_LINE("Speaker driver is initialized successfully !!!");
}

/******************************************************************************
* Function : ctrl_dev_deinit
* Brief    : 
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void ctrl_dev_deinit(void)
{
    ctrl_dev_gpio_deinit();
    ctrl_dev_timer_deinit();
}

/******************************************************************************
* Function : ctrl_dev_spk_start
* Brief    :
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void ctrl_dev_spk_start(void)
{
	if(!speaker_is_start)
	{
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		speaker_is_start = true;
	}
}

/******************************************************************************
* Function : ctrl_dev_spk_stop
* Brief    : 
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void ctrl_dev_spk_stop(void)
{
	if(speaker_is_start)
	{
	   HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	   speaker_is_start = false;
	}
}


/******************************************************************************
* Function : ctrl_dev_spk_set_duty
* Brief    : 
* Input    : 
            PWM_DUTY_PERCENT_0             
            PWM_DUTY_PERCENT_10            
            PWM_DUTY_PERCENT_20            
            PWM_DUTY_PERCENT_30            
            PWM_DUTY_PERCENT_40            
            PWM_DUTY_PERCENT_50            
            PWM_DUTY_PERCENT_60            
            PWM_DUTY_PERCENT_70            
            PWM_DUTY_PERCENT_80            
            PWM_DUTY_PERCENT_90            
            PWM_DUTY_PERCENT_100           
* Output   : None.
* Return   : None.
*******************************************************************************/
void ctrl_dev_spk_set_duty(uint8_t duty)
{
	ctrl_dev_spk_stop();
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty);
	ctrl_dev_spk_start();

}

/******************************************************************************
* Function : ctrl_dev_relay_on
* Brief    : 
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void ctrl_dev_relay_on(bool en)
{
    if(en)
    {
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_GPIO_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_GPIO_Pin, GPIO_PIN_RESET);
    }
}


/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
static void ctrl_dev_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_GPIO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_GPIO_Pin, GPIO_PIN_RESET);

  // Speaker pin
  GPIO_InitStruct.Pin = SPEAKER_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPEAKER_GPIO_Port, &GPIO_InitStruct);

  // Relay pin
  GPIO_InitStruct.Pin = RELAY_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

}

static void ctrl_dev_gpio_deinit(void)
{
    HAL_GPIO_DeInit(SPEAKER_GPIO_Port, SPEAKER_GPIO_Pin);
    HAL_GPIO_DeInit(RELAY_GPIO_Port, RELAY_GPIO_Pin);
}

static void ctrl_dev_timer_init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = PWM_TIMER_NUM;
  htim4.Init.Prescaler = PWM_TIMER_PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PWM_FREQUENCE;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = PWM_DUTY_PERCENT_0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void ctrl_dev_timer_deinit(void)
{
	__HAL_RCC_TIM4_CLK_DISABLE();
}

/*** END OF FILE **************************************************************/
