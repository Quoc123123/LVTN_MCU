/*******************************************************************************
* Title                 :   Heart beat applications
* Filename              :   heart_beat_app.h
* Author                :   QuocBrave
* Origin Date           :   07/11/2020
* Notes                 :   None.
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include "main.h"
#include "log_debug.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "string.h"
#include "led.h"
#include "heart_beat_app.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define HEART_BEAT_THREAD_NAME           			("heart_beat_thread")
#define HEART_BEAT_THREAD_STACK_SIZE     			(128 * 4)
#define HEART_BEAT_THREAD_PRIORITY       			((osPriority_t) osPriorityBelowNormal)


/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/


/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/

// definitions for rfid thread
const  osThreadAttr_t heart_beat_thread_attr =
{
  .name       = HEART_BEAT_THREAD_NAME,
  .priority   = HEART_BEAT_THREAD_PRIORITY,
  .stack_size = HEART_BEAT_THREAD_STACK_SIZE
};


static __attribute__((unused)) osThreadId_t heart_beat_thread_handle;


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void heart_beat_thread_entry (void *argument);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : int rfid_thread_init(void)
* Brief    : Creating thread for rfid
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
int heart_beat_thread_init(void)
{
	heart_beat_thread_handle = osThreadNew(heart_beat_thread_entry, NULL, &heart_beat_thread_attr);
	if(heart_beat_thread_handle == NULL)
	{
		PRINT_INFO_LOG("failed to create Heart beat thread!\r\n");
		return 1;
	}
	
	return 0;
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
static void heart_beat_thread_entry (void *argument)
{
    while(1)
    {
        led_blue_on(true);
        osDelay(1000);
        led_blue_on(false);
        osDelay(1000);
    }
}

/*************** END OF FILES *************************************************/
