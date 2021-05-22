/*******************************************************************************
* Title                 :   Driver for mfrc522
* Filename              :   mfrc522.h
* Author                :   QuocBrave
* Origin Date           :   03/09/2020
* Notes                 :   None
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include <stdio.h>
#include "main.h"
#include "mfrc522.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_spi.h"
#include "log_debug.h"


/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define MFRC522_SPI_CS_GPIO_Port           	(GPIOB)
#define MFRC522_SPI_CS_Pin            		(GPIO_PIN_15)

// CLK pin
#define MFRC522_SPI_CLK_GPIO_Port          	(GPIOA)
#define MFRC522_SPI_CLK_Pin           		(GPIO_PIN_5)

// MOSI pin
#define MFRC522_SPI_MOSI_GPIO_Port         	(GPIOA)
#define MFRC522_SPI_MOSI_Pin          		(GPIO_PIN_7)

// MISO pin
#define MFRC522_SPI_MISO_GPIO_Port         	(GPIOA)
#define MFRC522_SPI_MISO_Pin          		(GPIO_PIN_6)

// POWER pin
#define MFRC522_POWER_EN_GPIO_Port         	(GPIOC)
#define MFRC522_POWER_EN_Pin          		(GPIO_PIN_4)

// Enable clock
#define MFRC522_SPI_CLK_ENABLE()           __HAL_RCC_SPI1_CLK_ENABLE()
#define MFRC522_SPI_CS_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define MFRC522_SPI_CLK_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define MFRC522_SPI_MOSI_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define MFRC522_SPI_MISO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

// Disable clock
#define MFRC522_SPI_CLK_DISABLE()          __HAL_RCC_SPI1_CLK_DISABLE()
#define MFRC522_SPI_CS_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
#define MFRC522_SPI_CLK_CLK_DISABLE()      __HAL_RCC_GPIOA_CLK_DISABLE()
#define MFRC522_SPI_MOSI_CLK_DISABLE()     __HAL_RCC_GPIOA_CLK_DISABLE()
#define MFRC522_SPI_MISO_CLK_DISABLE()     __HAL_RCC_GPIOA_CLK_DISABLE()

// Reset the SPI1
#define MFRC522_SPI_FORCE_RESET()          __HAL_RCC_SPI1_FORCE_RESET()
#define MFRC522_SPI_RELEASE_RESET()        __HAL_RCC_SPI1_RELEASE_RESET()



//-- PAGE 0: COMMAND AND STATUS REGISTERS
//-----------------------------------------------------------------------------
// Definition of the registers
//-----------------------------------------------------------------------------
#define MFRC522_REG_RESERVED00							0x00
#define MFRC522_REG_COMMAND								0x01
#define MFRC522_REG_COMM_IE_N							0x02
#define MFRC522_REG_DIV1_EN								0x03
#define MFRC522_REG_COMM_IRQ							0x04
#define MFRC522_REG_DIV_IRQ								0x05
#define MFRC522_REG_ERROR								0x06
#define MFRC522_REG_STATUS1								0x07
#define MFRC522_REG_STATUS2								0x08
#define MFRC522_REG_FIFO_DATA							0x09
#define MFRC522_REG_FIFO_LEVEL							0x0A
#define MFRC522_REG_WATER_LEVEL							0x0B
#define MFRC522_REG_CONTROL								0x0C
#define MFRC522_REG_BIT_FRAMING							0x0D
#define MFRC522_REG_COLL								0x0E
#define MFRC522_REG_RESERVED01							0x0F

//-- PAGE 1: COMMAND
//-----------------------------------------------------------------------------
#define MFRC522_REG_RESERVED10							0x10
#define MFRC522_REG_MODE								0x11
#define MFRC522_REG_TX_MODE								0x12
#define MFRC522_REG_RX_MODE								0x13
#define MFRC522_REG_TX_CONTROL							0x14
#define MFRC522_REG_TX_AUTO								0x15
#define MFRC522_REG_TX_SELL								0x16
#define MFRC522_REG_RX_SELL								0x17
#define MFRC522_REG_RX_THRESHOLD						0x18
#define MFRC522_REG_DEMOD								0x19
#define MFRC522_REG_RESERVED11							0x1A
#define MFRC522_REG_RESERVED12							0x1B
#define MFRC522_REG_MIFARE								0x1C
#define MFRC522_REG_RESERVED13							0x1D
#define MFRC522_REG_RESERVED14							0x1E
#define MFRC522_REG_SERIALSPEED							0x1F

//-- PAGE 2: CONFIGURATION
//-----------------------------------------------------------------------------
#define MFRC522_REG_RESERVED20							0x20
#define MFRC522_REG_CRC_RESULT_M						0x21
#define MFRC522_REG_CRC_RESULT_L						0x22
#define MFRC522_REG_RESERVED21							0x23
#define MFRC522_REG_MOD_WIDTH							0x24
#define MFRC522_REG_RESERVED22							0x25
#define MFRC522_REG_RF_CFG								0x26
#define MFRC522_REG_GS_N								0x27
#define MFRC522_REG_CWGS_PREG							0x28
#define MFRC522_REG__MODGS_PREG							0x29
#define MFRC522_REG_T_MODE								0x2A
#define MFRC522_REG_T_PRESCALER							0x2B
#define MFRC522_REG_T_RELOAD_H							0x2C
#define MFRC522_REG_T_RELOAD_L							0x2D
#define MFRC522_REG_T_COUNTER_VALUE_H					0x2E
#define MFRC522_REG_T_COUNTER_VALUE_L					0x2F

//-- PAGE 3: TEST REGISTER
//-----------------------------------------------------------------------------
#define MFRC522_REG_RESERVED30							0x30
#define MFRC522_REG_TEST_SEL1							0x31
#define MFRC522_REG_TEST_SEL2							0x32
#define MFRC522_REG_TEST_PIN_EN							0x33
#define MFRC522_REG_TEST_PIN_VALUE						0x34
#define MFRC522_REG_TEST_BUS							0x35
#define MFRC522_REG_AUTO_TEST							0x36
#define MFRC522_REG_VERSION								0x37
#define MFRC522_REG_ANALOG_TEST							0x38
#define MFRC522_REG_TEST_ADC1							0x39
#define MFRC522_REG_TEST_ADC2							0x3A
#define MFRC522_REG_TEST_ADC0							0x3B
#define MFRC522_REG_RESERVED31							0x3C
#define MFRC522_REG_RESERVED32							0x3D
#define MFRC522_REG_RESERVED33							0x3E
#define MFRC522_REG_RESERVED34							0x3F

//-----------------------------------------------------------------------------
// Registers components
//-----------------------------------------------------------------------------
// MFRC522 Commands
#define PCD_IDLE										0x00	// NO action; Cancel the current command
#define PCD_AUTHENT										0x0E  	// Authentication Key
#define PCD_RECEIVE										0x08   	// Receive Data
#define PCD_TRANSMIT									0x04   	// Transmit data
#define PCD_TRANSCEIVE									0x0C   	// Transmit and receive data,
#define PCD_RESETPHASE									0x0F   	// Reset
#define PCD_CALCCRC										0x03   	// CRC Calculate
#define MFRC522_COMMAND_MASK							0xF0
// Mifare_One card command word
#define PICC_REQIDL										0x26   	// find the antenna area does not enter hibernation
#define PICC_REQALL										0x52   	// find all the cards antenna area
#define PICC_ANTICOLL									0x93   	// anti-collision
#define PICC_SElECTTAG									0x93   	// election card
#define PICC_AUTHENT1A									0x60   	// authentication key A
#define PICC_AUTHENT1B									0x61   	// authentication key B
#define PICC_READ										0x30   	// Read Block
#define PICC_WRITE										0xA0   	// write block
#define PICC_DECREMENT									0xC0   	// debit
#define PICC_INCREMENT									0xC1   	// recharge
#define PICC_RESTORE									0xC2   	// transfer block data to the buffer
#define PICC_TRANSFER									0xB0   	// save the data in the buffer
#define PICC_HALT										0x50   	// Sleep


#define MFRC522_DUMMY									0x00	// Dummy byte
#define MFRC522_MAX_LEN									16		// Buf len byte

#define MFRC522_TX1RF_EN								0x01
#define MFRC522_TX2RF_EN								0x02

// Temp
# define MFRC522_REG_COMM_IRQ_MASK					    0x80

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/
#define MFRC522_CS_HIGH() HAL_GPIO_WritePin(MFRC522_SPI_CS_GPIO_Port, MFRC522_SPI_CS_Pin, GPIO_PIN_SET)
#define MFRC522_CS_LOW() HAL_GPIO_WritePin(MFRC522_SPI_CS_GPIO_Port, MFRC522_SPI_CS_Pin, GPIO_PIN_RESET)

/******************************************************************************
* TYPEDEFS
*******************************************************************************/


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void 	mfrc522_spi_init(void);
static void 	mfrc522_spi_deinit(void);
static void 	mfrc522_gpio_init(void);
static void 	mfrc522_gpio_deinit(void);
static void 	mfrc522_setup(void);
static void 	mfrc522_power_en(bool en);
static void 	mfrc522_antenna_on(bool en);
static void 	write_regs(uint8_t reg, uint8_t value);
static void 	mfrc522_set_data(uint8_t reg, uint8_t mask);
static void 	mfrc522_clear_data(uint8_t reg, uint8_t mask);
static uint8_t  read_regs(uint8_t reg);

/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
static SPI_HandleTypeDef mfrc522_spi_cfg = {
  // General config
  .Instance = MFRC522_SPI,
  .Init.Mode = SPI_MODE_MASTER,
  .Init.Direction = SPI_DIRECTION_2LINES,
  .Init.DataSize = SPI_DATASIZE_8BIT,
  .Init.CLKPolarity = SPI_POLARITY_LOW,
  .Init.CLKPhase = SPI_PHASE_1EDGE,
  .Init.NSS = SPI_NSS_SOFT,
  .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32,
  .Init.FirstBit = SPI_FIRSTBIT_MSB,
  .Init.TIMode = SPI_TIMODE_DISABLE,
  .Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
  .Init.CRCPolynomial = 10,
};

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

/******************************************************************************
* Function : mfrc522_init
* Brief    : Init mfrc522 driver
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void mfrc522_init(void)
{
  mfrc522_spi_init();
  mfrc522_gpio_init();
  mfrc522_setup();
  PRINT_INFO_LOG_LINE("RFID driver is initialized successfully !!!");
}

/******************************************************************************
* Function : mfrc522_deinit
* Brief    : DeInit mfrc522 driver
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void mfrc522_deinit(void)
{
  mfrc522_spi_deinit();
}

/******************************************************************************
* Function : mfrc52_power_en
* Brief    : enable/disable power for mfrc522
* Input    : en    - true: enable
* 				   - false: disable
* Output   : None.
* Return   : None.
*******************************************************************************/
void mfrc522_power_en(bool en)
{
	if(en)
	{
	  HAL_GPIO_WritePin(MFRC522_POWER_EN_GPIO_Port, MFRC522_POWER_EN_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(MFRC522_POWER_EN_GPIO_Port, MFRC522_POWER_EN_Pin, GPIO_PIN_SET);
	}
	else
	{
	  HAL_GPIO_WritePin(MFRC522_POWER_EN_GPIO_Port, MFRC522_POWER_EN_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1);
	}
}

uint8_t mfrc522_check(uint8_t* id) {
	uint8_t status;
	status = mfrc522_request(PICC_REQIDL, id);							// Find cards, return card type
	if (status == MI_OK) status = mfrc522_anticoll(id);			// Card detected. Anti-collision, return card serial number 4 bytes
//	mfrc522_halt();																					// Command card into hibernation
	return status;
}

uint8_t mfrc522_compare(uint8_t* CardID, uint8_t* CompareID) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i]) return MI_ERR;
	}
	return MI_OK;
}


uint8_t mfrc522_request(uint8_t reqMode, uint8_t* TagType) {
	uint8_t status;
	uint16_t backBits;																			// The received data bits

	write_regs(MFRC522_REG_BIT_FRAMING, 0x07);		// TxLastBists = BitFramingReg[2..0]
	TagType[0] = reqMode;
	status = mfrc522_to_card(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
	if ((status != MI_OK) || (backBits != 0x10)) status = MI_ERR;
	return status;
}

uint8_t mfrc522_to_card(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen) {
	uint8_t status = MI_ERR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (command) {
		case PCD_AUTHENT: {
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE: {
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
		break;
	}

	write_regs(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
	mfrc522_clear_data(MFRC522_REG_COMM_IRQ, 0x80);
	mfrc522_set_data(MFRC522_REG_FIFO_LEVEL, 0x80);
	write_regs(MFRC522_REG_COMMAND, PCD_IDLE);

	// Writing data to the FIFO
	for (i = 0; i < sendLen; i++) write_regs(MFRC522_REG_FIFO_DATA, sendData[i]);

	// Execute the command
	write_regs(MFRC522_REG_COMMAND, command);
	if (command == PCD_TRANSCEIVE) mfrc522_set_data(MFRC522_REG_BIT_FRAMING, 0x80);		// StartSend=1,transmission of data starts

	// Waiting to receive data to complete
	i = 2000;	// i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms
	do {
		// CommIrqReg[7..0]
		// Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = read_regs(MFRC522_REG_COMM_IRQ);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	mfrc522_clear_data(MFRC522_REG_BIT_FRAMING, 0x80);																// StartSend=0

	if (i != 0)  {
		if (!(read_regs(MFRC522_REG_ERROR) & 0x1B)) {
			status = MI_OK;
			if (n & irqEn & 0x01) status = MI_NOTAGERR;
			if (command == PCD_TRANSCEIVE) {
				n = read_regs(MFRC522_REG_FIFO_LEVEL);
				lastBits = read_regs(MFRC522_REG_CONTROL) & 0x07;
				if (lastBits) *backLen = (n-1)*8+lastBits; else *backLen = n*8;
				if (n == 0) n = 1;
				if (n > MFRC522_MAX_LEN) n = MFRC522_MAX_LEN;
				for (i = 0; i < n; i++) backData[i] = read_regs(MFRC522_REG_FIFO_DATA);		// Reading the received data in FIFO
			}
		} else status = MI_ERR;
	}
	return status;
}

uint8_t mfrc522_anticoll(uint8_t* serNum) {
	uint8_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	write_regs(MFRC522_REG_BIT_FRAMING, 0x00);												// TxLastBists = BitFramingReg[2..0]
	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = mfrc522_to_card(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
	if (status == MI_OK) {
		// Check card serial number
		for (i = 0; i < 4; i++) serNumCheck ^= serNum[i];
		if (serNumCheck != serNum[i]) status = MI_ERR;
	}
	return status;
}

void mfrc522_calculate_crc(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData) {
	uint8_t i, n;

	mfrc522_clear_data(MFRC522_REG_DIV_IRQ, 0x04);													// CRCIrq = 0
	mfrc522_set_data(MFRC522_REG_FIFO_LEVEL, 0x80);													// Clear the FIFO pointer
	// Write_MFRC522(CommandReg, PCD_IDLE);

	// Writing data to the FIFO
	for (i = 0; i < len; i++) write_regs(MFRC522_REG_FIFO_DATA, *(pIndata+i));
	write_regs(MFRC522_REG_COMMAND, PCD_CALCCRC);

	// Wait CRC calculation is complete
	i = 0xFF;
	do {
		n = read_regs(MFRC522_REG_DIV_IRQ);
		i--;
	} while ((i!=0) && !(n&0x04));																						// CRCIrq = 1

	// Read CRC calculation result
	pOutData[0] = read_regs(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = read_regs(MFRC522_REG_CRC_RESULT_M);
}

uint8_t mfrc522_select_tag(uint8_t* serNum) {
	uint8_t i;
	uint8_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9];

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) buffer[i+2] = *(serNum+i);
	mfrc522_calculate_crc(buffer, 7, &buffer[7]);		//??
	status = mfrc522_to_card(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
	if ((status == MI_OK) && (recvBits == 0x18)) size = buffer[0]; else size = 0;
	return size;
}

uint8_t mfrc522_auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum) {
	uint8_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12];

	// Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++) buff[i+2] = *(Sectorkey+i);
	for (i=0; i<4; i++) buff[i+8] = *(serNum+i);
	status = mfrc522_to_card(PCD_AUTHENT, buff, 12, buff, &recvBits);
	if ((status != MI_OK) || (!(read_regs(MFRC522_REG_STATUS2) & 0x08))) status = MI_ERR;
	return status;
}

uint8_t mfrc522_read(uint8_t blockAddr, uint8_t* recvData) {
	uint8_t status;
	uint16_t unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	mfrc522_calculate_crc(recvData,2, &recvData[2]);
	status = mfrc522_to_card(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);
	if ((status != MI_OK) || (unLen != 0x90)) status = MI_ERR;
	return status;
}

uint8_t mfrc522_write(uint8_t blockAddr, uint8_t* writeData) {
	uint8_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18];

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	mfrc522_calculate_crc(buff, 2, &buff[2]);
	status = mfrc522_to_card(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);
	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) status = MI_ERR;
	if (status == MI_OK) {
		// Data to the FIFO write 16Byte
		for (i = 0; i < 16; i++) buff[i] = *(writeData+i);
		mfrc522_calculate_crc(buff, 16, &buff[16]);
		status = mfrc522_to_card(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) status = MI_ERR;
	}
	return status;
}


void mfrc522_reset(void) {
	write_regs(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}



void mfrc522_halt(void) {
	uint16_t unLen;
	uint8_t buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	mfrc522_calculate_crc(buff, 2, &buff[2]);
	mfrc522_to_card(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
static void mfrc522_antenna_on(bool en)
{
	uint8_t temp;
	if(en)
	{
		temp = read_regs(MFRC522_REG_TX_CONTROL);
		if (!(temp & 0x03)) mfrc522_set_data(MFRC522_REG_TX_CONTROL, 0x03);
	}
	else
	{
		mfrc522_clear_data(MFRC522_REG_TX_CONTROL, 0x03);
	}
}

static void write_regs(uint8_t reg, uint8_t value)
{
  uint8_t tx_buff[2];
  // format addr
  tx_buff[0] = ((reg << 1) & 0x7E) ;
  tx_buff[1] = value;

  while(HAL_SPI_GetState(&mfrc522_spi_cfg) != HAL_SPI_STATE_READY);
  MFRC522_CS_LOW();
  if(HAL_SPI_Transmit(&mfrc522_spi_cfg, tx_buff, sizeof(tx_buff), MFRC522_TRANSMISSION_TIMEOUT) != HAL_OK)
  {
	Error_Handler();
  }
  MFRC522_CS_HIGH();
}

static uint8_t read_regs(uint8_t reg)
{
  uint8_t tx_buff[1];
  uint8_t rx_buff[2];
  // format addr
  tx_buff[0] = (((reg << 1) & 0x7E) | (0x80)) ;

  while(HAL_SPI_GetState(&mfrc522_spi_cfg) != HAL_SPI_STATE_READY);
  MFRC522_CS_LOW();
  if(HAL_SPI_TransmitReceive(&mfrc522_spi_cfg, tx_buff, rx_buff, 3, MFRC522_TRANSMISSION_TIMEOUT) != HAL_OK)
  {
	Error_Handler();
  }
  MFRC522_CS_HIGH();

  return rx_buff[1];
}

static void mfrc522_spi_init(void)
{
  // Enable SPI clock
   MFRC522_SPI_CLK_ENABLE();
  if(HAL_SPI_Init(&mfrc522_spi_cfg) != HAL_OK)
  {
	// print error log
	Error_Handler();
  }
}

static void mfrc522_spi_deinit(void)
{
  if( HAL_SPI_DeInit(&mfrc522_spi_cfg) != HAL_OK)
  {
	 // print error log
	 Error_Handler();
  }
  // Disable SPI clock
  MFRC522_SPI_CLK_DISABLE();
}


static void mfrc522_setup(void)
{
	mfrc522_power_en(true);

	mfrc522_reset();
	write_regs(MFRC522_REG_T_MODE, 0x8D);
	write_regs(MFRC522_REG_T_PRESCALER, 0x3E);
	write_regs(MFRC522_REG_T_RELOAD_L, 30);
	write_regs(MFRC522_REG_T_RELOAD_H, 0);
	write_regs(MFRC522_REG_RF_CFG, 0x70);				// 48dB gain
	write_regs(MFRC522_REG_TX_AUTO, 0x40);
	write_regs(MFRC522_REG_MODE, 0x3D);
	mfrc522_antenna_on(true);

}

static void mfrc522_set_data(uint8_t reg, uint8_t mask)
{
	write_regs(reg, read_regs(reg) | mask);
}

static void mfrc522_clear_data(uint8_t reg, uint8_t mask)
{
	write_regs(reg, read_regs(reg) & (~mask));
}

static void mfrc522_gpio_init(void)
{
  //---------------------------------------------------------------------------
  // Step 1: Configure the I2C clock source. The clock is derived from the SYSCLK
  //---------------------------------------------------------------------------

  //---------------------------------------------------------------------------
  // Step 2: Enable peripherals and GPIO clock
  //---------------------------------------------------------------------------
  // Enable GPIO TX/RX clock
  MFRC522_SPI_CS_CLK_ENABLE();
  MFRC522_SPI_CLK_CLK_ENABLE();
  MFRC522_SPI_MOSI_CLK_ENABLE();
  MFRC522_SPI_MISO_CLK_ENABLE();

  //---------------------------------------------------------------------------
  // Step 3: Configure peripheral GPIO
  //---------------------------------------------------------------------------
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(MFRC522_SPI_CS_GPIO_Port, MFRC522_SPI_CS_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = MFRC522_SPI_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(MFRC522_SPI_MOSI_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MFRC522_SPI_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(MFRC522_SPI_MISO_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MFRC522_SPI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(MFRC522_SPI_CLK_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MFRC522_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MFRC522_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  // Configure MFRC522 POWER EN Pin
  GPIO_InitStruct.Pin = MFRC522_POWER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MFRC522_POWER_EN_GPIO_Port, &GPIO_InitStruct);

}

static __attribute__((unused)) void mfrc522_gpio_deinit(void)
{
  //---------------------------------------------------------------------------
  // Step 1: Reset peripherals
  //---------------------------------------------------------------------------
	MFRC522_SPI_FORCE_RESET();
	MFRC522_SPI_RELEASE_RESET();
  //---------------------------------------------------------------------------
  // Step 2: Disable peripherals and GPIO clocks
  //---------------------------------------------------------------------------
  HAL_GPIO_DeInit(MFRC522_SPI_CS_GPIO_Port, MFRC522_SPI_CS_Pin);
  HAL_GPIO_DeInit(MFRC522_SPI_CLK_GPIO_Port, MFRC522_SPI_CLK_Pin);
  HAL_GPIO_DeInit(MFRC522_SPI_MOSI_GPIO_Port, MFRC522_SPI_MOSI_Pin);
  HAL_GPIO_DeInit(MFRC522_SPI_MISO_GPIO_Port, MFRC522_SPI_MISO_Pin);
}



/*************** END OF FILES *************************************************/
