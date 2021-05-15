/*******************************************************************************
* Title                 :   Rfid Application
* Filename              :   rfid_app.c
* Author                :   QuocBrave
* Origin Date           :   07/11/2020
* Notes                 :   None.
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include "rfid_app.h"
#include "rfid_command.h"
#include "main.h"
#include "log_debug.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "mfrc522.h"
#include "string.h"
#include "led.h"
#include "control_device.h"



/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define RFID_TX_MSG_BUF_LEN         		(20)   // Bytes
#define RFID_RX_MSG_BUF_LEN         		(20)   // Bytes
#define TAGS_CODE_SIZE  					(5)
#define RFID_SEND_DATA_SIZE					(13)

#define RFID_THREAD_NAME           			("rfid_thread")
#define RFID_THREAD_STACK_SIZE     			(128 * 4)
#define RFID_THREAD_PRIORITY       			((osPriority_t) osPriorityRealtime)

#define CONTROL_THREAD_NAME        	  		("control_thread")
#define CONTROL_THREAD_STACK_SIZE     		(128 * 4)
#define CONTROL_THREAD_PRIORITY       		((osPriority_t) osPriorityNormal)

#define LED_TURN_OFF						(0x00)
#define LED_RED_ON 							(0x01)
#define LED_GREEN_ON 					 	(0x02)

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/

typedef enum
{
	OPEN_DOOR = 0,
	CLOSE_DOOR,

} CONTROL_DOOR_STATUS;


/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
static uint8_t rfid_tx_msg_buf        [RFID_TX_MSG_BUF_LEN];
static uint8_t rfid_rx_msg_buf        [RFID_RX_MSG_BUF_LEN];

// definitions for rfid thread
const  osThreadAttr_t rfid_thread_attr =
{
  .name       = RFID_THREAD_NAME,
  .priority   = RFID_THREAD_PRIORITY,
  .stack_size = RFID_THREAD_STACK_SIZE
};

// definitions for control thread
const  osThreadAttr_t control_thread_attr =
{
  .name       = CONTROL_THREAD_NAME,
  .priority   = CONTROL_THREAD_PRIORITY,
  .stack_size = CONTROL_THREAD_STACK_SIZE
};

static __attribute__((unused)) osThreadId_t rfid_thread_handle;
static __attribute__((unused)) osThreadId_t control_thread_handle;

static osSemaphoreId_t control_device_sem;

static uint8_t tags_code[TAGS_CODE_SIZE];
static uint8_t rfid_msg_id = 0;
static uint8_t rfid_dma_data[1];
static uint8_t control_value[2];



/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void control_release_dma_from_pc(void);
static void control_wait_dma_from_pc(void);
static void rfid_thread_entry(void *argument);
static void control_thread_entry(void *argument);
static void control_device_init(void);

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
int rfid_thread_init(void)
{
	rfid_thread_handle = osThreadNew(rfid_thread_entry, NULL, &rfid_thread_attr);
	if(rfid_thread_handle == NULL)
	{
		PRINT_INFO_LOG("failed to create RFID thread!\r\n");
		return 1;
	}
	
	return 0;
}
/******************************************************************************
* Function : int rfid_thread_init(void)
* Brief    : Creating thread for the control device
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
int control_thread_init(void)
{
	control_thread_handle = osThreadNew(control_thread_entry, NULL, &control_thread_attr);
	if(rfid_thread_handle == NULL)
	{
		PRINT_INFO_LOG("failed to create RFID thread!\r\n");
		return 1;
	}

	return 0;
}

/******************************************************************************
* Function : void rfid_send_msg_to_pc(uint8_t *p_data, uint16_t len)
* Brief    : send data received from sensor to pc
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void rfid_send_msg_to_pc(uint8_t *p_data, uint16_t len)
{
	uint16_t idx = 0;
	rfid_tx_msg_buf[idx++] = 'R';
	rfid_tx_msg_buf[idx++] = 'F';
	rfid_tx_msg_buf[idx++] = 'I';
	rfid_tx_msg_buf[idx++] = 'R';
	rfid_tx_msg_buf[idx++] = p_data[0];
	rfid_tx_msg_buf[idx++] = p_data[1];
	rfid_tx_msg_buf[idx++] = p_data[2];
	rfid_tx_msg_buf[idx++] = p_data[3];
	rfid_tx_msg_buf[idx++] = p_data[4];
	rfid_tx_msg_buf[idx++] = '$';
	rfid_tx_msg_buf[idx++] = '$';
	rfid_tx_msg_buf[idx++] = '$';
	rfid_tx_msg_buf[idx++] = '$';

	rfid_uart_tx_send_string(rfid_tx_msg_buf, idx);
}

/******************************************************************************
* Function : void rfid_receive_complete_callback(UART_HandleTypeDef *huart)
* Brief    : 
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void rfid_receive_complete_callback(UART_HandleTypeDef *huart)
{
  static uint16_t rx_idx = 0;
  static uint16_t frame_size = 0;
  static bool is_received_header = false;

  // read the incoming byte
  if(huart->Instance == USART1)
  {
	rfid_rx_msg_buf[rx_idx++] = rfid_dma_data[0];

	if(is_received_header == false)
	{
		// Get mesasage header 
		if(rx_idx == RFID_FRAME_HEADER_SIZE)
		{
			if((rfid_rx_msg_buf[0] == 'R') && (rfid_rx_msg_buf[1] == 'F') && (rfid_rx_msg_buf[2] == 'I') && (rfid_rx_msg_buf[3] == 'C'))
			{
				rfid_msg_id = RFID_REQ_MSG_ID;
				is_received_header = true;
				frame_size = 0;
			}
			else
			{
				rx_idx--;
				// Shift the received array left to remove the first received byte
				rfid_rx_msg_buf[0] = rfid_rx_msg_buf[1];
				rfid_rx_msg_buf[1] = rfid_rx_msg_buf[2];
				rfid_rx_msg_buf[2] = rfid_rx_msg_buf[3];
				rfid_msg_id = 0;
				frame_size = 0;
			}
		}
	}

	if(is_received_header && (rfid_msg_id == RFID_REQ_MSG_ID) && (frame_size == 0))
	{
		if(rx_idx == (RFID_FRAME_HEADER_SIZE + 2))  // received payload size
        {
          frame_size = RFID_FRAME_HEADER_SIZE + RFID_FRAME_FOOTER_SIZE + (rfid_rx_msg_buf[5] | (uint16_t)rfid_rx_msg_buf[4] << 8) + RFID_PAYLOAD_LENGHT_SIZE;
        }
	}

	if(is_received_header && (rx_idx == frame_size))
	{
		if((rfid_rx_msg_buf[frame_size - RFID_FRAME_FOOTER_SIZE] == RFID_FRAME_FOOTER_VALUE) && 
			(rfid_rx_msg_buf[frame_size - RFID_FRAME_FOOTER_SIZE + 1] == RFID_FRAME_FOOTER_VALUE) && 
			(rfid_rx_msg_buf[frame_size - RFID_FRAME_FOOTER_SIZE + 2] == RFID_FRAME_FOOTER_VALUE) && 
			(rfid_rx_msg_buf[frame_size - RFID_FRAME_FOOTER_SIZE + 3] == RFID_FRAME_FOOTER_VALUE))
		{
			control_value[0] = rfid_rx_msg_buf[RFID_FRAME_HEADER_SIZE + RFID_PAYLOAD_LENGHT_SIZE];
			control_value[1] = rfid_rx_msg_buf[RFID_FRAME_HEADER_SIZE + RFID_PAYLOAD_LENGHT_SIZE + 1];
			control_release_dma_from_pc();

		}

		memset(rfid_rx_msg_buf, 0x00, sizeof(rfid_rx_msg_buf));
		rx_idx = 0;
		is_received_header = false;
		frame_size = 0;
	}
	rfid_uart_rx_receive_string_dma(rfid_dma_data, 1);
  }
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
static void rfid_thread_entry(void *argument)
{
	// Init UART TX and RX for transmit/receive data
	mfrc522_init();
	HAL_Delay(10);
	while(1)
	{
		if(mfrc522_check(tags_code) == MI_OK)
		{
//			PRINT_INFO_LOG("[%02x-%02x-%02x-%02x-%02x] \r\n", tags_code[0], tags_code[1],
//															  tags_code[2], tags_code[3],
//															  tags_code[4]);

			rfid_send_msg_to_pc(tags_code, sizeof(tags_code));
		}
		osDelay(3);
	}
}

static void control_thread_entry(void *argument)
{
	control_device_init();
	while(1)
	{
		control_wait_dma_from_pc();
		switch (rfid_msg_id)
		{
			case RFID_REQ_MSG_ID:
				if(control_value[0] == LED_TURN_OFF)
				{
					led_red_on(false);
					led_green_on(false);
					ctrl_dev_spk_on(false);
					ctrl_dev_relay_on(false);

				}
				else
				{
					if((control_value[1] & LED_RED_ON) == LED_RED_ON)
					{
						led_red_on(true);
						ctrl_dev_spk_on(false);
						ctrl_dev_relay_on(false);
					}
					if((control_value[1] & LED_GREEN_ON) == LED_GREEN_ON)
					{
						led_green_on(true);
						ctrl_dev_spk_on(true);
						ctrl_dev_relay_on(true);
					}
				}

			default:
				break;
		}
	}
}

static void control_device_init(void)
{
	control_device_sem = osSemaphoreNew(1, 0, NULL);
	rfid_uart_rx_receive_string_dma(rfid_dma_data, 1);
	HAL_Delay(10);
}

static void control_wait_dma_from_pc(void)
{
	osSemaphoreAcquire(control_device_sem, osWaitForever);
}

static void control_release_dma_from_pc(void)
{
	osSemaphoreRelease(control_device_sem);
}
	

/*************** END OF FILES *************************************************/
