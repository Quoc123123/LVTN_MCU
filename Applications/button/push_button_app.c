/*******************************************************************************
* Title                 :   Driver for button
* Filename              :   push_button_app.h
* Author                :   QuocBrave
* Origin Date           :   05/16/2020
* Notes                 :   None
*******************************************************************************/


/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <stdbool.h>
#include "main.h"
#include "cmsis_os2.h"
#include "push_button_app.h"
#include "control_device.h"
#include "led.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define PB_THREAD_NAME             ("pb_debounce_thread")
#define PB_THREAD_STACK_SIZE       (1024) // Bytes
#define PB_THREAD_PRIORITY         ((osPriority_t) osPriorityRealtime)

#define PB_PRESSED              1
#define PB_RELEASED             0

/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/

/******************************************************************************
* MACROS
*******************************************************************************/

/******************************************************************************
* TYPEDEFS
*******************************************************************************/
typedef enum {
  PB_STATE_STANDBY = 0,
  PB_STATE_PRESS,
} pb_state_t;

static const osThreadAttr_t pb_thread_attr =
{
  .name       = PB_THREAD_NAME,
  .priority   = PB_THREAD_PRIORITY,
  .stack_size = PB_THREAD_STACK_SIZE,
};


static __attribute__((unused)) osThreadId_t pb_thread_handle;
static pb_state_t pb_state = PB_STATE_STANDBY;
static osSemaphoreId_t  pb_sem;

/******************************************************************************
* VARIABLES
*******************************************************************************/


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void pb_gpio_init(void);
static void pb_gpio_deinit(void);
static void pb_thread_entry (void *argument);
static void pb_init(void);
static void pb_deinit(void);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

/******************************************************************************
* EXTI's CALLBACK FUNCTION
*******************************************************************************/
void pb_exti_int_cb(void)
{
  if(pb_state == PB_STATE_STANDBY)
  {
    pb_state = PB_STATE_PRESS;
    osSemaphoreRelease(pb_sem);
  }
}

/******************************************************************************
* Function : pb_thread_init
* Brief    : Init push button thread
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void pb_thread_init(void)
{
  pb_init();
  pb_sem = osSemaphoreNew(1, 0, NULL);
  pb_thread_handle = osThreadNew(pb_thread_entry, NULL, &pb_thread_attr);
}


/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : pb_init
* Brief    : Init push button
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void pb_init(void)
{
  pb_gpio_init();
} 

/******************************************************************************
* Function : pb_deinit
* Brief    : Deinit push button
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static __attribute__((unused)) void pb_deinit(void)
{
  pb_gpio_deinit();
} 

static void pb_thread_entry (void *argument)
{
    while(1)
    {
        osSemaphoreAcquire(pb_sem, osWaitForever);
        osDelay(20);
        if(PB_PRESSED == HAL_GPIO_ReadPin(PB_GPIO_Port, PB_GPIO_Pin))
        {
        	// admin turn off all of the devices
			led_red_on(false);
			led_green_on(false);
			ctrl_dev_spk_set_duty(PWM_DUTY_PERCENT_0);
			ctrl_dev_relay_on(false);
        }
        pb_state = PB_STATE_STANDBY;
    }
}

static void pb_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //=======================================================
  //  Config push button volume's pin
  //=======================================================
  GPIO_InitStruct.Pin       = PB_GPIO_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PB_GPIO_Port, &GPIO_InitStruct);


  //=======================================================
  // Enable interrupts push-button
  //=======================================================
  HAL_NVIC_SetPriority(PB_EXTI_SOURCE, 5, 0);
  HAL_NVIC_EnableIRQ(PB_EXTI_SOURCE);
}

static __attribute__((unused)) void pb_gpio_deinit(void)
{
  HAL_GPIO_DeInit(PB_GPIO_Port, PB_GPIO_Pin);
  HAL_NVIC_DisableIRQ(PB_EXTI_SOURCE);
}

/*** END OF FILE **************************************************************/
