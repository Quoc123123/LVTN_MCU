/*******************************************************************************
* Title                 :   Driver for control device
* Filename              :   control_devive.h
* Author                :   QuocBrave
* Origin Date           :   03/09/2020
* Notes                 :   None
*******************************************************************************/


#ifndef CONTROL_DEVICE_H_
#define CONTROL_DEVICE_H_
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
#define PWM_DUTY_PERCENT_0              (0)
#define PWM_DUTY_PERCENT_10             (100)
#define PWM_DUTY_PERCENT_20             (200)
#define PWM_DUTY_PERCENT_30             (300)
#define PWM_DUTY_PERCENT_40             (400)
#define PWM_DUTY_PERCENT_50             (500)
#define PWM_DUTY_PERCENT_60             (600)
#define PWM_DUTY_PERCENT_70             (700)
#define PWM_DUTY_PERCENT_80             (800)
#define PWM_DUTY_PERCENT_90             (900)
#define PWM_DUTY_PERCENT_100            (1000)

/******************************************************************************
* MACROS
*******************************************************************************/

/******************************************************************************
* TYPEDEFS
*******************************************************************************/

/******************************************************************************
* VARIABLES
*******************************************************************************/
void ctrl_dev_init(void);
void ctrl_dev_deinit(void);
void ctrl_dev_relay_on(bool en);
void ctrl_dev_spk_start(void);
void ctrl_dev_spk_stop(void);
void ctrl_dev_spk_set_duty(uint8_t duty);

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
#endif // CONTROL_DEVICE_H_
/*** END OF FILE **************************************************************/
