/*******************************************************************************
* Title                 :   Print log debug source file
* Filename              :   log_debug.c
* Author                :   QuoBrave
* Origin Date           :   12/09/2020
* Notes                 :   None
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include "main.h"
#include "log_debug.h"


/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define LOG_DEBUG_UART_INSTANCE   (USART3)
#define LOG_DEBUG_UART_BAUD_RATE  (230400)

#define LOG_DEBUG_UART_TX_PIN      GPIO_PIN_10       // TX : Port C - Pin 10
#define LOG_DEBUG_UART_TX_PORT     GPIOC


/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/
#define DEBUG_UART_CLK_EN()         __HAL_RCC_USART3_CLK_ENABLE()
#define DEBUG_UART_GPIO_CLK_EN()    __HAL_RCC_GPIOC_CLK_ENABLE()


/******************************************************************************
* TYPEDEFS
*******************************************************************************/


/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
#if (LOG_DEBUG_EN)
UART_HandleTypeDef uart_log_debug = {
  .Instance             = LOG_DEBUG_UART_INSTANCE,
  .Init.BaudRate        = LOG_DEBUG_UART_BAUD_RATE,
  .Init.WordLength      = UART_WORDLENGTH_8B,
  .Init.StopBits        = UART_STOPBITS_1,
  .Init.Parity          = UART_PARITY_NONE,
  .Init.Mode            = UART_MODE_TX,
  .Init.HwFlowCtl       = UART_HWCONTROL_NONE,
  .Init.OverSampling = UART_OVERSAMPLING_16
};
#endif
/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/


/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

/******************************************************************************
* Function : log_debug_init
* Brief    : Initialize print log debug over uart
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void log_debug_init(void)
{
#if (LOG_DEBUG_EN)
  GPIO_InitTypeDef gpio_init;
  // enable clock uart and gpio
  DEBUG_UART_CLK_EN();
  DEBUG_UART_GPIO_CLK_EN();

  if(HAL_UART_Init(&uart_log_debug) != HAL_OK)
  {
    Error_Handler();
  }

  gpio_init.Mode      = GPIO_MODE_AF_PP;
  gpio_init.Pull      = GPIO_NOPULL;
  gpio_init.Pin       = LOG_DEBUG_UART_TX_PIN;
  gpio_init.Speed     = GPIO_SPEED_FREQ_MEDIUM;
  gpio_init.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(LOG_DEBUG_UART_TX_PORT, &gpio_init);

#endif
}

#if (LOG_DEBUG_EN)
  #if defined(__GNUC__)
  int _write(int fd, char * ptr, int len)
  {
    HAL_UART_Transmit(&uart_log_debug, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
  }
  #elif defined (__ICCARM__)
  #include "LowLevelIOInterface.h"
  size_t __write(int handle, const unsigned char * buffer, size_t size)
  {
    HAL_UART_Transmit(&uart_log_debug, (uint8_t *) buffer, size, HAL_MAX_DELAY);
    return size;
  }
  #elif defined (__CC_ARM)
  int fputc(int ch, FILE *f)
  {
      HAL_UART_Transmit(&uart_log_debug, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
      return ch;
  }
  #endif
#endif

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/



/*************** END OF FILES *************************************************/
