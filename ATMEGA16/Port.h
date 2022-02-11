 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for Port Module.
 *
 * Author: Manar Salah
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H


#include "Micro_Config.h"


/* Id for the company in the AUTOSAR
 * for example company ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (124U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port init */
#define PORT_INIT_SID           (uint8)0x00

/* Service ID for port set pin direction */
#define PORT_SET_PIN_DIRECTION_SID          (uint8)0x01

/* Service ID for refresh port direction */
#define REFRESH_PORT_DIRECTION_SID              (uint8)0x02

/* Service ID for port get version info */
#define PORT_GET_VERSION_INFO_SID             (uint8)0x03

/* Service ID for port set pin mode */
#define PORT_SET_PIN_MODE_SID     (uint8)0x04


/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report Invalid Port Pin ID  */
#define PORT_E_PARAM_PIN (uint8)0x0A

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE             (uint8)0x0B

/* API Port_Init service called with wrong parameter. */
#define PORT_E_PARAM_CONFIG    (uint8)0x0C

/* API Port_SetPinMode service called when mode is unchangeable. */
#define PORT_E_PARAM_INVALID_MODE      (uint8)0x0D

/* API Port_SetPinMode service called when mode is unchangeable. */
#define PORT_E_MODE_UNCHANGEABLE      (uint8)0x0E

/* API service called without module initialization */
#define PORT_E_UNINIT      (uint8)0x0F

/*
 * APIs called with a Null Pointer
 */
#define PORT_E_PARAM_POINTER             (uint8)0x10


/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Type definition for Port_PinType used by the PORT APIs */
typedef uint8 Port_PinType;

/* Type definition for Port_PinModeType used by the PORT APIs */
typedef uint8 Port_PinModeType;

/*enum for Port_PinDirectionType*/
typedef enum
{
	/* Member Sets port pin as input */
	PORT_PIN_IN=0,
	/* Member Sets port pin as output */
	PORT_PIN_OUT=1
}Port_PinDirectionType;


/* Structure for Port_ConfigChannel */
/****************************************************************************************************************/
/* Description: Structure to describe each individual PIN contains:
 *	1. the PORT Which the pin belongs to. 0, 1, 2 or 3
 *	2. the number of the pin in the PORT.
 *	3. if pin direction changeable.
 *	4. if pin mode changeable.
 *  5. the direction of pin --> INPUT or OUTPUT
 *  6. the mode of pin --> DIO, DIO_GPT, DIO_WDG, ADC, CAN, LIN, SPI, FLEXRAY, MEM, ICU or PWM
 *  7. activation of internal pull-up --> ENABLE or DISABLE in case direction is input.
 *  8. initial value for pin in case direction is output.
 */
/***************************************************************************************************************/
typedef struct
{
	uint8 Port_num;

	Port_PinType Pin_num;

	uint8 Pin_direction_changeable;

	uint8 Pin_mode_changeable;

	Port_PinDirectionType Pin_direction;

	Port_PinModeType Pin_mode;

	uint8 Activation_internal_pullups;

	uint8 Pin_level_init;

}Port_ConfigChannel;




/* Data Structure required for initializing the port Driver */
typedef struct
{
	Port_ConfigChannel Channel[NUMBER_OF_CHANNELS];
}Port_ConfigType;



/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* Function for port init API */
void Port_Init( const Port_ConfigType* ConfigPtr );

/* Function for port set pin direction API */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction );
#endif

/* Function for refresh port direction API */
void Port_RefreshPortDirection( void );

/* Function for port Get Version Info API */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );
#endif

/* Function for port set pin mode API */
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;


#endif /* PORT_H */
