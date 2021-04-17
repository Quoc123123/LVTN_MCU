/*******************************************************************************
* Title                 :   Driver for R305
* Filename              :   led.c
* Author                :   QuocBrave
* Origin Date           :   03/09/2020
* Notes                 :   None
*******************************************************************************/

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include "led.h"
#include "main.h"
#include "log_debug.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define LED_GREEN_GPIO_Port             (GPIOB)
#define LED_GREEN_Pin            	    (GPIO_PIN_0)

#define LED_BLUE_GPIO_Port              (GPIOB)
#define LED_BLUE_Pin            	    (GPIO_PIN_7)

#define LED_RED_GPIO_Port               (GPIOB)
#define LED_RED_Pin            	        (GPIO_PIN_14)


/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/

/******************************************************************************
* MACROS
*******************************************************************************/

/******************************************************************************
* TYPEDEFS
*******************************************************************************/

/******************************************************************************
* VARIABLES
*******************************************************************************/


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void led_gpio_init(void);
static void led_gpio_deinit(void);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
void led_init(void)
{
    led_gpio_init();
}

void led_deinit(void)
{
    led_gpio_deinit();
}


void led_green_on(bool en)
{
    if(en)
    {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    }
}

void led_blue_on(bool en)
{
    if(en)
    {
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
    }
}

void led_red_on(bool en)
{
    if(en)
    {
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    }
}


/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
static void led_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  // GREEN pin
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  // BLUE pin
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  // RED pin
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

}

static void led_gpio_deinit(void)
{
    HAL_GPIO_DeInit(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    HAL_GPIO_DeInit(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    HAL_GPIO_DeInit(LED_RED_GPIO_Port, LED_RED_Pin);
}



/*** END OF FILE **************************************************************/
