/*******************************************************************************
* Title                 :   Rfid Command
* Filename              :   rfid_command.h
* Author                :   QuocBrave
* Origin Date           :   07/11/2020
* Notes                 :   None.
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include "main.h"
#include "rfid_command.h"
#include "log_debug.h"
#include "rfid_app.h"
/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define RFID_UART_TX_SEND_TIMEOUT          (1000)
#define RFID_UART_RX_RECEIVE_TIMEOUT       (1000)

//---------------------------------------------------------------------------------------
// UART configuration
//---------------------------------------------------------------------------------------
// General
#define RFID_UART_NUM                      (USART1)
#define RFID_UART_BAUD_RATE                (19200)

// TX pin
#define RFID_UART_TX_GPIO_Port             (GPIOA)
#define RFID_UART_TX_Pin            	   (GPIO_PIN_9)
#define RFID_UART_TX_AF_Pin				   (GPIO_AF7_USART1)

// RX pin
#define RFID_UART_RX_GPIO_Port             (GPIOA)
#define RFID_UART_RX_Pin            	   (GPIO_PIN_10)
#define RFID_UART_RX_AF_Pin				   (GPIO_AF7_USART1)

// Enable clock
#define RFID_UART_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE()
#define	RFID_UART_TX_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define RFID_UART_RX_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()

// Disable clock
#define RFID_UART_CLK_DISABLE()            __HAL_RCC_USART1_CLK_DISABLE()
#define RFID_UART_TX_DISABLE()             __HAL_RCC_GPIOA_CLK_DISABLE()
#define RFID_UART_RX_CLK_DISABLE()         __HAL_RCC_GPIOA_CLK_DISABLE()

#define RFID_UART_FORCE_RESET()            __HAL_RCC_USART1_FORCE_RESET()
#define RFID_UART_RELEASE_RESET()          __HAL_RCC_USART1_RELEASE_RESET()

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void rfid_uart_init(void);
static void rfid_uart_deinit(void);
static void rfid_gpio_init(void);
static void rfid_gpio_deinit(void);
/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
DMA_HandleTypeDef hdma_uart1_rx;

 UART_HandleTypeDef rfid_uart_cfg = {
  // General config
  .Instance = RFID_UART_NUM,
  .Init.BaudRate = RFID_UART_BAUD_RATE,
  .Init.WordLength = UART_WORDLENGTH_8B,
  .Init.StopBits = UART_STOPBITS_1,
  .Init.Parity = UART_PARITY_NONE,
  .Init.Mode = UART_MODE_TX_RX,
  .Init.HwFlowCtl = UART_HWCONTROL_NONE,
  .Init.OverSampling = UART_OVERSAMPLING_16,
};


/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

/******************************************************************************
* Function : void rfid_comm_init(void)
* Brief    : Init uart driver
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void rfid_comm_init(void)
{
	rfid_gpio_init();
	rfid_uart_init();
	rfid_uart_cfg.RxCpltCallback = rfid_receive_complete_callback;
}

/******************************************************************************
* Function : void rfid_comm_deinit(void)
* Brief    : DeInit uart driver
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void rfid_comm_deinit(void)
{
	rfid_uart_deinit();
}

/******************************************************************************
* Function : rfid_uart_tx_send_string(uint8_t* str, uint16_t msg_size)
* Brief    : Send the whole input string through UART
* Input    : str      - pointer to sent string
*            msg_size - size of input string
* Output   : None.
* Return   : None.
*******************************************************************************/
void rfid_uart_tx_send_string(uint8_t* str, uint16_t msg_size)
{
	HAL_UART_Transmit(&rfid_uart_cfg, str, msg_size, RFID_UART_TX_SEND_TIMEOUT);
}

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

/******************************************************************************
* Function : void rfid_uart_rx_receive_string_dma(uint8_t *str, uint16_t msg_size)
* Brief    : Receive data from pc
* Input    : None
* Output   : None.
* Return   : None.
*******************************************************************************/
void rfid_uart_rx_receive_string_dma(uint8_t *str, uint16_t msg_size)
{
	HAL_UART_Receive_DMA(&rfid_uart_cfg, str, msg_size);
}
/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/

static void rfid_uart_init(void)
{
   // Enable UART clock
	RFID_UART_CLK_ENABLE();
  if(HAL_UART_Init(&rfid_uart_cfg) != HAL_OK)
  {
	// print error log
  }
}

static void rfid_uart_deinit(void)
{
  if( HAL_UART_DeInit(&rfid_uart_cfg) != HAL_OK)
  {
	 // print error log
  }

  // Disable UART clock
  RFID_UART_CLK_DISABLE();
}

static void rfid_gpio_init(void)
{
  //---------------------------------------------------------------------------
  // Step 1: Configure the UART clock source. The clock is derived from the SYSCLK
  //---------------------------------------------------------------------------

  //---------------------------------------------------------------------------
  // Step 2: Enable peripherals and GPIO clock
  //---------------------------------------------------------------------------
  // Enable GPIO TX/RX clock
    RFID_UART_TX_CLK_ENABLE();
	RFID_UART_RX_CLK_ENABLE();

  //---------------------------------------------------------------------------
  // Step 3: Configure peripheral GPIO
  //---------------------------------------------------------------------------
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // TX pin
  GPIO_InitStruct.Pin = RFID_UART_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = RFID_UART_TX_AF_Pin;
  HAL_GPIO_Init(RFID_UART_TX_GPIO_Port, &GPIO_InitStruct);

  // RX pin
  GPIO_InitStruct.Pin = RFID_UART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = RFID_UART_RX_AF_Pin;
  HAL_GPIO_Init(RFID_UART_RX_GPIO_Port, &GPIO_InitStruct);

  // Configure DMA for receive data
  __HAL_RCC_DMA2_CLK_ENABLE();
  hdma_uart1_rx.Instance = DMA2_Stream2;
  hdma_uart1_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_uart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_uart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart1_rx.Init.Mode = DMA_NORMAL;
  hdma_uart1_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_uart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_uart1_rx) != HAL_OK)
  {
    Error_Handler();
  }
  
 __HAL_LINKDMA(&rfid_uart_cfg, hdmarx, hdma_uart1_rx);

  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  // Enable NVIC USART
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static __attribute__((unused))void rfid_gpio_deinit(void)
{
  //---------------------------------------------------------------------------
  // Step 1: Reset peripherals
  //---------------------------------------------------------------------------
	RFID_UART_FORCE_RESET();
	RFID_UART_RELEASE_RESET();
  //---------------------------------------------------------------------------
  // Step 2: Disable peripherals and GPIO clocks
  //---------------------------------------------------------------------------
  HAL_GPIO_DeInit(RFID_UART_TX_GPIO_Port, RFID_UART_TX_Pin);
  HAL_GPIO_DeInit(RFID_UART_RX_GPIO_Port, RFID_UART_RX_Pin);
}


/*************** END OF FILES *************************************************/
