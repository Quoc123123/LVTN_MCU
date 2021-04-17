/*******************************************************************************
* Title                 :   Driver for mfrc522
* Filename              :   mfrc522.h
* Author                :   QuocBrave
* Origin Date           :   03/09/2020
* Notes                 :   None
*******************************************************************************/


#ifndef MFRC522_H_
#define MFRC522_H_
/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/


/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/
#define MFRC522_SPI						SPI1
#define MFRC522_TRANSMISSION_TIMEOUT 	5000

// Status enumeration, Used with most functions
#define MI_OK							 0
#define MI_NOTAGERR						 1
#define MI_ERR							 2

/******************************************************************************
* MACROS
*******************************************************************************/
#define BIT(x)                          (uint8_t)(1UL << (x))
#define MASK_BIT(x)						(uint8_t)(~(1UL << (x)))

/******************************************************************************
* TYPEDEFS
*******************************************************************************/

/******************************************************************************
* VARIABLES
*******************************************************************************/


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
extern void     mfrc522_init(void);
extern void     mfrc522_deinit(void);
extern uint8_t  mfrc522_request(uint8_t reqMode, uint8_t* TagType);
extern uint8_t  mfrc522_to_card(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
extern uint8_t  mfrc522_anticoll(uint8_t* serNum);
extern void     mfrc522_calulate_crc(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
extern uint8_t  mfrc522_select_tag(uint8_t* serNum);
extern uint8_t  mfrc522_auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
extern uint8_t  mfrc522_read(uint8_t blockAddr, uint8_t* recvData);
extern uint8_t  mfrc522_write(uint8_t blockAddr, uint8_t* writeData);
extern void     mfrc522_reset(void);
extern void     mfrc522_halt(void);
extern uint8_t  mfrc522_check(uint8_t* id);
extern uint8_t  mfrc522_compare(uint8_t* CardID, uint8_t* CompareID);
#endif // TEMPLATE_H_
/*** END OF FILE **************************************************************/
