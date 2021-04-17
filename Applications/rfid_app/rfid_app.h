/*******************************************************************************
* Title                 :   Rfid Application
* Filename              :   rfid_app.c
* Author                :   QuocBrave
* Origin Date           :   07/11/2020
* Notes                 :   None.
*******************************************************************************/
#ifndef RFID_APP_H_
#define RFID_APP_H_

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "main.h"


/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/

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
int rfid_thread_init(void);
int control_thread_init(void);
void rfid_send_msg_to_pc(uint8_t *p_data, uint16_t len);
void rfid_receive_complete_callback(UART_HandleTypeDef *huart);

#endif // RFID_APP_H_
/*** END OF FILE **************************************************************/
