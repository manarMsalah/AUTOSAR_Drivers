/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for Port Module.
 *
 * Author: Manar Salah
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

/* Pre-compile option for presence of Port_SetPinDirection API */
#define PORT_SET_PIN_DIRECTION_API                (STD_ON)

/* Pre-compile option for presence of Port_SetPinMode API */
#define PORT_SET_PIN_MODE_API                     (STD_OFF)

/*Pin mode changeable during runtime */
#define PORT_PIN_MODE_CHANGEABLE                 (STD_ON)
#define PORT_PIN_MODE_UNCHANGEABLE                 (STD_OFF)

/*Pin direction changeable during runtime */
#define PORT_PIN_DIRECTION_CHANGEABLE            (STD_ON)
#define PORT_PIN_DIRECTION_UNCHANGEABLE           (STD_OFF)

/* Number of Micro-controller Channels */
#define NUMBER_OF_CHANNELS             (32U)

/* PIN Index in the array of structures in Port_PBcfg.c */
#define PORTA_PIN0_ID_INDEX        (uint8)0
#define PORTA_PIN1_ID_INDEX        (uint8)1
#define PORTA_PIN2_ID_INDEX        (uint8)2
#define PORTA_PIN3_ID_INDEX        (uint8)3
#define PORTA_PIN4_ID_INDEX        (uint8)4
#define PORTA_PIN5_ID_INDEX        (uint8)5
#define PORTA_PIN6_ID_INDEX        (uint8)6
#define PORTA_PIN7_ID_INDEX        (uint8)7
#define PORTB_PIN0_ID_INDEX        (uint8)8
#define PORTB_PIN1_ID_INDEX        (uint8)9
#define PORTB_PIN2_ID_INDEX        (uint8)10
#define PORTB_PIN3_ID_INDEX        (uint8)11
#define PORTB_PIN4_ID_INDEX        (uint8)12
#define PORTB_PIN5_ID_INDEX        (uint8)13
#define PORTB_PIN6_ID_INDEX        (uint8)14
#define PORTB_PIN7_ID_INDEX        (uint8)15
#define PORTC_PIN0_ID_INDEX        (uint8)16
#define PORTC_PIN1_ID_INDEX        (uint8)17
#define PORTC_PIN2_ID_INDEX        (uint8)18
#define PORTC_PIN3_ID_INDEX        (uint8)19
#define PORTC_PIN4_ID_INDEX        (uint8)20
#define PORTC_PIN5_ID_INDEX        (uint8)21
#define PORTC_PIN6_ID_INDEX        (uint8)22
#define PORTC_PIN7_ID_INDEX        (uint8)23
#define PORTD_PIN0_ID_INDEX        (uint8)24
#define PORTD_PIN1_ID_INDEX        (uint8)25
#define PORTD_PIN2_ID_INDEX        (uint8)26
#define PORTD_PIN3_ID_INDEX        (uint8)27
#define PORTD_PIN4_ID_INDEX        (uint8)28
#define PORTD_PIN5_ID_INDEX        (uint8)29
#define PORTD_PIN6_ID_INDEX        (uint8)30
#define PORTD_PIN7_ID_INDEX        (uint8)31

/* PORT Configured Port ID's  */
#define PORTA_NUM                (uint8)0 /* PORTA */
#define PORTB_NUM                (uint8)1 /* PORTB */
#define PORTC_NUM                (uint8)2 /* PORTC */
#define PORTD_NUM                (uint8)3 /* PORTD */

/* PORT Configured PIN ID's */
#define PIN0             (uint8)0
#define PIN1             (uint8)1
#define PIN2             (uint8)2
#define PIN3             (uint8)3
#define PIN4             (uint8)4
#define PIN5             (uint8)5
#define PIN6             (uint8)6
#define PIN7             (uint8)7


/* PORT Configured PIN mode */
#define PORT_PIN_MODE_ADC            (uint8)0
#define PORT_PIN_MODE_CAN            (uint8)1
#define PORT_PIN_MODE_DIO            (uint8)2
#define PORT_PIN_MODE_DIO_GPT        (uint8)3
#define PORT_PIN_MODE_DIO_WDG        (uint8)4
#define PORT_PIN_MODE_FLEXRAY        (uint8)5
#define PORT_PIN_MODE_ICU            (uint8)6
#define PORT_PIN_MODE_LIN            (uint8)7
#define PORT_PIN_MODE_MEM            (uint8)8
#define PORT_PIN_MODE_PWM            (uint8)9
#define PORT_PIN_MODE_SPI            (uint8)10

/* PORT Configured PIN initial mode */
#define PORT_PIN_INITIAL_MODE       PORT_PIN_MODE_DIO

/* PORT Configured PIN initial level value */
#define PORT_PIN_LEVEL_HIGH          (1U)
#define PORT_PIN_LEVEL_LOW           (0U)

/* PORT Configured internal pull-up */
#define ENABLE_INTERNAL_PULL_UP      (STD_ON)
#define DISABLE_INTERNAL_PULL_UP      (STD_OFF)

/* random value for not applicable cases */
#define NOT_APPLICABLE             (uint8)999

#endif /* PORT_CFG_H */
