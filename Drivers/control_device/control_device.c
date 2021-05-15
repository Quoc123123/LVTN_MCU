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
#include <control_device.h>
#include "main.h"
#include "log_debug.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define SPEAKER_GPIO_Port           	(GPIOD)
#define SPEAKER_GPIO_Pin                (GPIO_PIN_0)

#define RELAY_GPIO_Port           		(GPIOD)
#define RELAY_GPIO_Pin            		(GPIO_PIN_1)



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
static void ctrl_dev_gpio_init(void);
static void ctrl_dev_gpio_deinit(void);

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
}

/******************************************************************************
* Function : ctrl_dev_spk_on
* Brief    : 
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void ctrl_dev_spk_on(bool en)
{
    if(en)
    {
        HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_GPIO_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_GPIO_Pin, GPIO_PIN_RESET);
    }
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

  // GREEN pin
  GPIO_InitStruct.Pin = SPEAKER_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPEAKER_GPIO_Port, &GPIO_InitStruct);

  // BLUE pin
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



/*** END OF FILE **************************************************************/
