/*******************************************************************************
* Title                 :   Print log debug header file
* Filename              :   log_debug.h
* Author                :   QuocBrave
* Origin Date           :   08/09/2020
* Notes                 :   None
*******************************************************************************/

#ifndef LOG_DEBUG_H_
#define LOG_DEBUG_H_

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <stdio.h>

/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/


#define LOG_DEBUG_EN          1 // 1: Enable debug log over UART
                                // 0: Disable debug log over UART

#define INFO_LOG_DEBUG_EN     1
#define ERROR_LOG_DEBUG_EN    1


/******************************************************************************
* MACROS
*******************************************************************************/
#if (INFO_LOG_DEBUG_EN && LOG_DEBUG_EN)
  #define PRINT_INFO_LOG(...)  printf(__VA_ARGS__)
#else
  #define PRINT_INFO_LOG(...)
#endif

#if (ERROR_LOG_DEBUG_EN && LOG_DEBUG_EN)
  #define PRINT_ERROR_LOG(...)  printf(__VA_ARGS__)
#else
  #define PRINT_ERROR_LOG(...)
#endif

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
extern void log_debug_init(void);

#endif // LOG_DEBUG_H_
/*** END OF FILE **************************************************************/
