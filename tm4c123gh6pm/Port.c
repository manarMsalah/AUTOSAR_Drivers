/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for Port Module.
 *
 * Author: Manar Salah
 ******************************************************************************/

#include "Port.h"
#include "Dio_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
		|| (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
		|| (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC Port_ConfigChannel * Port_PortChannels = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
 * Service Name: Port_Init
 * Service ID[hex]: 0x00
 * Sync/Async: Synchronous
 * Reentrancy: Non reentrant
 * Parameters (in): ConfigPtr - Pointer to post-build configuration data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Initialize the Port module.
 ************************************************************************************/
void Port_Init( const Port_ConfigType* ConfigPtr )
{
	/*point to the PB configuration structure using a global pointer.
	 * This global pointer is global to be used by other functions to read the PB configuration structures*/
	Port_PortChannels = ConfigPtr->Channel; /* address of the first Channels structure --> Channels[0] */

	uint8 * Port_Ptr = NULL_PTR;
	uint8 * DDR_Ptr = NULL_PTR;
	uint8 i;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
				PORT_E_PARAM_CONFIG);
	}
	else
#endif
	{
    /*for loop to cover all the microcontrollers pins*/
		for(i=0;i<NUMBER_OF_CHANNELS;i++)
		{
			/* Point to the correct DDR & PORT register according to the Port Id stored in the Port_Num member */
			switch(Port_PortChannels[i].Port_num)
			{
			case 0:	Port_Ptr = &PORTA_REG;
		        	DDR_Ptr = &DDRA;
			break;
			case 1:	Port_Ptr = &PORTB_REG;
			        DDR_Ptr = &DDRB;
			break;
			case 2:	Port_Ptr = &PORTC_REG;
			        DDR_Ptr = &DDRC;
			break;
			case 3:	Port_Ptr = &PORTD_REG;
			        DDR_Ptr = &DDRD;
			break;
			}
            /*check the pin direction output or input*/
			if(Port_PortChannels[i].Pin_direction == PORT_PIN_OUT)
			{
				SET_BIT(*DDR_Ptr,Port_PortChannels[i].Pin_num);// set the corresponding bit in the DDR register to configure it as output pin
                /*set the initial value for output pins*/
				if(Port_PortChannels[i].Pin_level_init == PORT_PIN_LEVEL_HIGH)
				{
					SET_BIT(*Port_Ptr,Port_PortChannels[i].Pin_num); // set the corresponding bit in the PORT register to set the initial value of output pin to 1
				}
				else if(Port_PortChannels[i].Pin_level_init == PORT_PIN_LEVEL_LOW)
				{
					CLEAR_BIT(*Port_Ptr,Port_PortChannels[i].Pin_num); // clear the corresponding bit in the PORT register to set the initial value of output pin to 0
				}
			}
			else if(Port_PortChannels[i].Pin_direction == PORT_PIN_IN)
			{
				CLEAR_BIT(*DDR_Ptr,Port_PortChannels[i].Pin_num); // clear the corresponding bit in the DDR register to configure it as input pin
                 /*enable or disable internal pull-up resistor for input pins*/
				if(Port_PortChannels[i].Activation_internal_pullups == ENABLE_INTERNAL_PULL_UP)
				{
					SET_BIT(*Port_Ptr,Port_PortChannels[i].Pin_num); // set the corresponding bit in the PORT register to enable the internal pull-up resistor
				}
				else if(Port_PortChannels[i].Activation_internal_pullups == DISABLE_INTERNAL_PULL_UP)
				{
					/*No action needed*/
				}
			}
                   /*pin mode is not applicable for AVR*/
		}
		Port_Status = PORT_INITIALIZED; /*Set the module state to initialized*/
	}

}

/************************************************************************************
 * Service Name: Port_SetPinDirection
 * Service ID[hex]: 0x01
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Port Pin ID number, Port Pin Direction
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Set the port pin direction
 ************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
	uint8 * DDR_Ptr = NULL_PTR;
	boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
	/* Check if the Port Pin not configured as changeable */
	if(Port_PortChannels[Pin].Pin_direction_changeable == PORT_PIN_DIRECTION_UNCHANGEABLE)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
	/* Check if the used  Pin ID passed is within the valid range */
	if (NUMBER_OF_CHANNELS > Pin)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
#endif

	/* In-case there are no errors */
	if(FALSE == error)
	{
		/* point to the required DDR Register */
		switch(Port_PortChannels[Pin].Port_num)
		{
		case  0: DDR_Ptr = &DDRA;
		break;
		case  1: DDR_Ptr = &DDRB;
		break;
		case  2: DDR_Ptr = &DDRC;
		break;
		case  3: DDR_Ptr = &DDRD;
		break;
		}
		if(Direction == PORT_PIN_OUT)
		{
			SET_BIT(*DDR_Ptr,Port_PortChannels[Pin].Pin_num); // set the corresponding bit in the DDR register to configure it as output pin
		}
		else if(Direction == PORT_PIN_IN)
		{
			CLEAR_BIT(*DDR_Ptr,Port_PortChannels[Pin].Pin_num); // clear the corresponding bit in the DDR register to configure it as input pin
		}

	}
}
#endif
/************************************************************************************
 * Service Name: Port_RefreshPortDirection
 * Service ID[hex]: 0x02
 * Sync/Async: Synchronous
 * Reentrancy: Non reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to refresh port direction.
 ************************************************************************************/
void Port_RefreshPortDirection( void )
{
	uint8 * DDR_Ptr = NULL_PTR;
	uint8 i;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);

	}
	else
#endif
	{
		for(i=0;i<NUMBER_OF_CHANNELS;i++)
				{
			/* Point to the correct DDR register according to the Port Id stored in the Port_Num member */
					switch(Port_PortChannels[i].Port_num)
					{
					case 0:	DDR_Ptr = &DDRA;
					break;
					case 1:	DDR_Ptr = &DDRB;
					break;
					case 2:	DDR_Ptr = &DDRC;
					break;
					case 3: DDR_Ptr = &DDRD;
					break;
					}

					/*Check if the pin direction changeable*/
					if(Port_PortChannels[i].Pin_direction_changeable == PORT_PIN_DIRECTION_CHANGEABLE )
					{
						if(Port_PortChannels[i].Pin_direction == PORT_PIN_OUT)
						{
							SET_BIT(*DDR_Ptr,Port_PortChannels[i].Pin_num);  // set the corresponding bit in the DDR register to configure it as output pin
						}
						else if(Port_PortChannels[i].Pin_direction == PORT_PIN_IN)
						{
							CLEAR_BIT(*DDR_Ptr,Port_PortChannels[i].Pin_num); // clear the corresponding bit in the DDR register to configure it as input pin
						}
					}
					else
					{
						/*No action needed*/
					}
				}

	}
}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): VersionInfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Function to get the version information of this module.
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
{
	boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
		error = TRUE;
	}
	else
	{
		/*No action needed*/
	}
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
#endif /* (DIO_DEV_ERROR_DETECT == STD_ON) */

	/* In-case there are no errors */
	if(FALSE == error)
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
#endif

/************************************************************************************
 * Service Name: Port_SetPinMode
 * Service ID[hex]: 0x04
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Port Pin ID number, New Port Pin mode to be set on port pin.
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Set the port pin mode.
 ************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
	boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
	/* Check if the Port Pin not configured as changeable */
	if (Port_PortChannels[Pin].Pin_mode_changeable==PORT_PIN_MODE_UNCHENGABLE )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}

	/* Check if the used  Pin ID passed is within the valid range */
	if (NUMBER_OF_CHANNELS > Pin)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}

	/* Check if the used  Port Pin Mode passed not valid */
	if ((Mode != PORT_PIN_MODE_ADC)
	 || (Mode != PORT_PIN_MODE_CAN)
	 || (Mode != PORT_PIN_MODE_DIO)
	 || (Mode != PORT_PIN_MODE_DIO_GPT)
	 || (Mode != PORT_PIN_MODE_DIO_WDG)
	 || (Mode != PORT_PIN_MODE_FLEXRAY)
	 || (Mode != PORT_PIN_MODE_ICU)
	 || (Mode != PORT_PIN_MODE_LIN)
	 || (Mode != PORT_PIN_MODE_MEM)
	 || (Mode != PORT_PIN_MODE_PWM)
	 || (Mode != PORT_PIN_MODE_SPI))
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
#endif

	/* In-case there are no errors */
		if(FALSE == error)
		{
			/*Mode not applicable in AVR*/
		}

}
#endif
