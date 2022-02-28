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
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
|| (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
  || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const Port_ConfigChannel * Port_PortChannels = NULL_PTR;
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
  * This global pointer is global to be used by other functions to read the PB configuration structures
  * address of the first Channels structure --> Channels[0] */  
  Port_PortChannels = ConfigPtr->Channel; 
  
  /* point to the required Port Registers base address */
  volatile uint32 * Port_Ptr = NULL_PTR; 
  
  volatile uint32 delay = 0;
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
    /*for loop to cover all GPIO pins*/
    for(i=0;i<NUMBER_OF_CHANNELS;i++)
    {
      /* Point to the correct port base address according to the Port Id stored in the Port_Num member */
      switch(Port_PortChannels[i].Port_num)
      {
      case  0: Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
      break;
      case  1: Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
      break;
      case  2: Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
      break;
      case  3: Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
      break;
      case  4: Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
      break;
      case  5: Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
      break; 
      }
      
      /* Enable clock for PORT and allow time for clock to start*/
      SYSCTL_REGCGC2_REG |= (1 <<  Port_PortChannels[i].Port_num);
      delay=SYSCTL_REGCGC2_REG;
      
      if( i == PORTD_PIN0_ID_INDEX || i == PORTD_PIN7_ID_INDEX ) /* PD7 or PF0 */
      {
        /* Unlock the GPIOCR register */
        *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
        /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[i].Pin_num);  
      }
      else if( i >= PORTC_PIN0_ID_INDEX && i <= PORTC_PIN3_ID_INDEX ) /* PC0 to PC3 */
      {
        /* Do Nothing ...  this is the JTAG pins */
      }
      else
      {
        /* Do Nothing ... No need to unlock the commit register for this pin */
      }
      
      
      /* Adjust the pin initial direction */  
      
      /*check the pin direction output or input*/
      if(Port_PortChannels[i].Pin_direction == PORT_PIN_OUT)
      {
        /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].Pin_num);
        
        /*  Adjust the initial value */
        /* check the initial value for output pins */
        if(Port_PortChannels[i].Pin_level_init == PORT_PIN_LEVEL_HIGH)
        {
          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DATA_REG_OFFSET),Port_PortChannels[i].Pin_num); 
        }
        else if(Port_PortChannels[i].Pin_level_init == PORT_PIN_LEVEL_LOW)
        {
          /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DATA_REG_OFFSET),Port_PortChannels[i].Pin_num); 
        }
        else
        {
          /* Do Nothing */
        }
      }
      else if(Port_PortChannels[i].Pin_direction == PORT_PIN_IN)
      {
        /* clear the corresponding bit in the GPIODIR register to configure it as input pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].Pin_num); 
        
        /* Adjust the internal resistor */
        /* Check the internal resistor for input pins pull-up or pull-down or disabled */
        if(Port_PortChannels[i].resistor == PULL_UP)
        {
          /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_UP_REG_OFFSET),Port_PortChannels[i].Pin_num); 
        }
        else if(Port_PortChannels[i].resistor == PULL_DOWN)
        {
          /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_DOWN_REG_OFFSET),Port_PortChannels[i].Pin_num);
        }
        else if(Port_PortChannels[i].resistor == DISABLE)
        {
          /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_UP_REG_OFFSET),Port_PortChannels[i].Pin_num);
          /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_DOWN_REG_OFFSET),Port_PortChannels[i].Pin_num);
          
        }
        else
        {
          /*No action needed*/
        }
      }
      else
      {
        /*No action needed*/
      }     
      
      
      /* Adjust the pin initial mode*/
      
      /* Modes for analog pins*/
      
      if(Port_PortChannels[i].Pin_mode == PORT_PIN_MODE_ADC || Port_PortChannels[i].Pin_mode == PORT_PIN_MODE_ANALOG_COMPARATOR_INPUT || Port_PortChannels[i].Pin_mode == PORT_PIN_MODE_USB_ANALOG )
      {
        /* Clear the corresponding bit in the GPIOAMSEL register to disable digital functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[i].Pin_num);
        /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[i].Pin_num);
        /* Set the corresponding bit in the GPIODEN register to enable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[i].Pin_num);
        
      }
      else
      {
        /* Modes for digital pins*/
        
        /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[i].Pin_num);
        
        if(Port_PortChannels[i].Pin_mode == PORT_PIN_MODE_DIO)
        {
          /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[i].Pin_num);
          /* Clear the PMCx bits for this pin */
          *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[i].Pin_num * 4));
          /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[i].Pin_num);
        }
        else
        {
          /* Modes for digital pins except DIO */
          
          /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[i].Pin_num);
          /* Clear the PMCx bits for this pin as pre-step to Set them by the requested control number */
          *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[i].Pin_num * 4));
          
          switch(Port_PortChannels[i].Pin_mode)
          {                                          
          case PORT_PIN_MODE_DIO_GPT: 
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_PortChannels[i].Pin_num * 4));
            break; 
            
          case PORT_PIN_MODE_PWM_MODULE0:  
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_PortChannels[i].Pin_num * 4));                                    
            break;                                  
            
          case PORT_PIN_MODE_PWM_MODULE1: 
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_PortChannels[i].Pin_num * 4));
            break;                        
            
          case PORT_PIN_MODE_QEI:
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006 << (Port_PortChannels[i].Pin_num * 4));
            break;
            
          case PORT_PIN_MODE_CAN: 
            /* Check pins for modes those have several control number */
            if(i == PORTF_PIN0_ID_INDEX || i == PORTF_PIN3_ID_INDEX)
            {
              /* Set the PMCx bits for this pin by the mode control number*/
              *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_PortChannels[i].Pin_num * 4));
            }
            else
            {
              /* Set the PMCx bits for this pin by the mode control number*/
              *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PortChannels[i].Pin_num * 4));
            } 
            
            break;
            
          case PORT_PIN_MODE_SSI3:
          case PORT_PIN_MODE_UART:
          case PORT_PIN_MODE_JTAG:
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PortChannels[i].Pin_num * 4));                     
            break;
            
          case  PORT_PIN_MODE_SSI:
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_PortChannels[i].Pin_num * 4));               
            break;
            
          case PORT_PIN_MODE_I2C:
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_PortChannels[i].Pin_num * 4));
            break;
            
            
          case PORT_PIN_MODE_UART1:
            /* Check pins for modes those have several control number */
            if( i == PORTB_PIN0_ID_INDEX || i == PORTB_PIN1_ID_INDEX )
            {
              /* Set the PMCx bits for this pin by the mode control number*/
              *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PortChannels[i].Pin_num * 4));
            }
            else 
            {
              /* Set the PMCx bits for this pin by the mode control number*/
              *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_PortChannels[i].Pin_num * 4));
            }
            
            break;  
            
          case PORT_PIN_MODE_UART1_CONTROL:
            /* Check pins for modes those have several control number */
            if( i == PORTF_PIN0_ID_INDEX || i == PORTF_PIN1_ID_INDEX )
            { 
              /* Set the PMCx bits for this pin by the mode control number*/
              *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PortChannels[i].Pin_num * 4));
            }
            else 
            {
              /* Set the PMCx bits for this pin by the mode control number*/
              *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PortChannels[i].Pin_num * 4));
            }
            
            break; 
            
          case PORT_PIN_MODE_USB_DIGITAL:
          case PORT_PIN_MODE_NMI:
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PortChannels[i].Pin_num * 4));  
            break;
            
          case PORT_PIN_MODE_ANALOG_COMPARATOR_OUTPUT:
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000009 << (Port_PortChannels[i].Pin_num * 4));
            break; 
            
          case PORT_PIN_MODE_CORE:
            /* Set the PMCx bits for this pin by the mode control number*/
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= (0x0000000E << (Port_PortChannels[i].Pin_num * 4));
            break;                       
          }  
          /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[i].Pin_num);
        }
      }     
    }
    /*Set the module state to initialized*/
    Port_Status = PORT_INITIALIZED; 
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
  /* point to the required Port Registers base address */
  volatile uint32 * Port_Ptr = NULL_PTR; 
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
    /* Point to the correct port base address according to the Port Id stored in the Port_Num member */
    switch(Port_PortChannels[Pin].Port_num)
    {
    case  0: Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    break;
    case  1: Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    break;
    case  2: Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    break;
    case  3: Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    break;
    case  4: Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    break;
    case  5: Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    break; 
    }
    
    if(Direction == PORT_PIN_OUT)
    {
      
      /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);   
      
    }
    else if(Direction == PORT_PIN_IN)
    {
      /* clear the corresponding bit in the GPIODIR register to configure it as input pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].Pin_num); 
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
  volatile uint32 * Port_Ptr = NULL_PTR; /* point to the required Port Registers base address */
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
    /*for loop to cover all GPIO pins*/
    for(i=0;i<NUMBER_OF_CHANNELS;i++)
    {
      
      /* Point to the correct port base address according to the Port Id stored in the Port_Num member */
      switch(Port_PortChannels[i].Port_num)
      {
      case  0: Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
      break;
      case  1: Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
      break;
      case  2: Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
      break;
      case  3: Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
      break;
      case  4: Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
      break;
      case  5: Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
      break;
      }
      
      
      
      
      /*Check if the pin direction changeable*/
      if(Port_PortChannels[i].Pin_direction_changeable == PORT_PIN_DIRECTION_CHANGEABLE )
      {
        
        if(Port_PortChannels[i].Pin_direction == PORT_PIN_OUT)
        {
          /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].Pin_num);   
        }
        else if(Port_PortChannels[i].Pin_direction == PORT_PIN_IN)
        {
          /* clear the corresponding bit in the GPIODIR register to configure it as input pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].Pin_num); 
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
* Service Name: Port_SetDigitalFunction
* Service ID[hex]: Not AUTOSAR Function
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pointer take the required Port Registers base address from caller, Port Pin ID number, Control number for required mode
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to configure the required registers for digital pin mode.
************************************************************************************/
void Port_SetDigitalFunction(volatile uint32 *PortBase_Ptr, uint8 Pin,  uint32 ControlNum)
{
  /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);
  /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);
  /* Clear the PMCx bits for this pin as pre-step to Set them by the requested control number */
  *(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].Pin_num * 4));
  /* Set the PMCx bits for this pin by the requested control number*/
  *(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) |= (ControlNum << (Port_PortChannels[Pin].Pin_num * 4));
  /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);  
}

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
  /* point to the required Port Registers base address */
  volatile uint32 * Port_Ptr = NULL_PTR; 
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
  if (Port_PortChannels[Pin].Pin_mode_changeable  ==  PORT_PIN_MODE_UNCHANGEABLE )
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
      || (Mode != PORT_PIN_MODE_ANALOG_COMPARATOR_INPUT)
        || (Mode != PORT_PIN_MODE_USB_ANALOG)
          || (Mode != PORT_PIN_MODE_DIO)
            || (Mode != PORT_PIN_MODE_DIO_GPT)
              || (Mode != PORT_PIN_MODE_PWM_MODULE0)
                || (Mode != PORT_PIN_MODE_PWM_MODULE1)  
                  || (Mode != PORT_PIN_MODE_QEI)
                    || (Mode != PORT_PIN_MODE_NMI)
                      || (Mode != PORT_PIN_MODE_CAN)
                        || (Mode != PORT_PIN_MODE_SSI)
                          || (Mode != PORT_PIN_MODE_SSI3)  
                            || (Mode != PORT_PIN_MODE_I2C)
                              || (Mode != PORT_PIN_MODE_UART)
                                || (Mode != PORT_PIN_MODE_UART1)
                                  || (Mode != PORT_PIN_MODE_UART1_CONTROL)
                                    || (Mode != PORT_PIN_MODE_USB_DIGITAL)
                                      || (Mode != PORT_PIN_MODE_JTAG)
                                        || (Mode != PORT_PIN_MODE_ANALOG_COMPARATOR_OUTPUT)
                                          || (Mode != PORT_PIN_MODE_CORE))  
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
    /* Point to the correct port base address according to the Port Id stored in the Port_Num member */
    switch(Port_PortChannels[Pin].Port_num)
    {
    case  PORTA_NUM: Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    break;
    case  PORTB_NUM: Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    break;
    case  PORTC_NUM: Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    break;
    case  PORTD_NUM: Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    break;
    case  PORTE_NUM: Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    break;
    case  PORTF_NUM: Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    break;
    
    }
    
    /* Modes for analog pins*/
    
    /* Check if the mode is valid for this pin */
    if( (Mode == PORT_PIN_MODE_ADC && (Pin == PORTB_PIN4_ID_INDEX || Pin == PORTB_PIN5_ID_INDEX || (Pin >= PORTD_PIN0_ID_INDEX && Pin<= PORTD_PIN3_ID_INDEX) || (Pin >= PORTE_PIN0_ID_INDEX && Pin<= PORTE_PIN5_ID_INDEX)))
       || (Mode == PORT_PIN_MODE_ANALOG_COMPARATOR_INPUT && (Pin >= PORTC_PIN4_ID_INDEX && Pin<= PORTC_PIN7_ID_INDEX))  
         || (Mode == PORT_PIN_MODE_USB_ANALOG && (Pin == PORTD_PIN4_ID_INDEX || Pin == PORTD_PIN5_ID_INDEX)) )
    {
      /* Clear the corresponding bit in the GPIOAMSEL register to disable digital functionality on this pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);
      /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);
      /* Set the corresponding bit in the GPIODEN register to enable analog functionality on this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);
    }
    else
    {
      /* Modes for digital pins */
      switch(Mode)
      {
      case PORT_PIN_MODE_DIO:
        /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);                   
        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);        
        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].Pin_num * 4));
        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].Pin_num);
        
        break;
        
      case PORT_PIN_MODE_DIO_GPT: 
        /* Check if the mode is valid for this pin */
        if((Pin >= PORTB_PIN0_ID_INDEX && Pin <= PORTD_PIN7_ID_INDEX) || (Pin >= PORTF_PIN0_ID_INDEX && Pin <= PORTF_PIN4_ID_INDEX))
        {
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000007);      
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break; 
        
      case  PORT_PIN_MODE_PWM_MODULE0:
        /* Check if the mode is valid for this pin */
        if( (Pin >= PORTB_PIN4_ID_INDEX && Pin >= PORTB_PIN7_ID_INDEX) || Pin == PORTC_PIN4_ID_INDEX || Pin == PORTC_PIN5_ID_INDEX 
           || (Pin >= PORTD_PIN0_ID_INDEX && Pin >= PORTD_PIN2_ID_INDEX) || Pin == PORTD_PIN6_ID_INDEX || Pin == PORTE_PIN4_ID_INDEX 
             || Pin == PORTE_PIN5_ID_INDEX || Pin == PORTF_PIN2_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000004);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;                                  
        
      case  PORT_PIN_MODE_PWM_MODULE1: 
        /* Check if the mode is valid for this pin */
        if( Pin == PORTA_PIN6_ID_INDEX || Pin == PORTA_PIN7_ID_INDEX 
           || Pin == PORTD_PIN0_ID_INDEX || Pin ==PORTD_PIN1_ID_INDEX    
             || Pin == PORTE_PIN4_ID_INDEX || Pin == PORTE_PIN5_ID_INDEX 
               || (Pin >= PORTF_PIN0_ID_INDEX && Pin >= PORTF_PIN4_ID_INDEX))  
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000005);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;                        
        
      case  PORT_PIN_MODE_QEI:
        /* Check if the mode is valid for this pin */
        if( (Pin >= PORTC_PIN4_ID_INDEX && Pin <= PORTC_PIN6_ID_INDEX) || Pin == PORTD_PIN3_ID_INDEX || Pin == PORTD_PIN6_ID_INDEX 
           || Pin == PORTD_PIN7_ID_INDEX || Pin == PORTF_PIN0_ID_INDEX || Pin == PORTF_PIN1_ID_INDEX || Pin == PORTF_PIN4_ID_INDEX )
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000006);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      case PORT_PIN_MODE_NMI: 
        /* Check if the mode is valid for this pin */
        if(Pin == PORTD_PIN7_ID_INDEX || Pin == PORTF_PIN0_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000008);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      case PORT_PIN_MODE_CAN:
        /* Check if the mode is valid for this pin */
        if(Pin == PORTF_PIN0_ID_INDEX || Pin == PORTF_PIN3_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000003);
        }
        else if( PORTA_PIN0_ID_INDEX || Pin == PORTA_PIN1_ID_INDEX || PORTB_PIN4_ID_INDEX || Pin == PORTB_PIN5_ID_INDEX || PORTE_PIN4_ID_INDEX || Pin == PORTE_PIN5_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000008);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
           
           break;
           
      case  PORT_PIN_MODE_SSI3:
        /* Check if the mode is valid for this pin */
        if(Pin >= PORTD_PIN0_ID_INDEX && Pin <= PORTD_PIN3_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000001);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      case  PORT_PIN_MODE_SSI:
        /* Check if the mode is valid for this pin */
        if( (Pin >= PORTA_PIN2_ID_INDEX && Pin <= PORTA_PIN5_ID_INDEX)    
           ||(Pin >= PORTB_PIN4_ID_INDEX && Pin <= PORTB_PIN7_ID_INDEX)
             ||(Pin >= PORTD_PIN0_ID_INDEX && Pin <= PORTD_PIN3_ID_INDEX)
               ||(Pin >= PORTF_PIN0_ID_INDEX && Pin <= PORTF_PIN3_ID_INDEX))
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000002);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      case PORT_PIN_MODE_I2C:
        /* Check if the mode is valid for this pin */
        if( Pin == PORTA_PIN6_ID_INDEX || Pin == PORTA_PIN7_ID_INDEX || Pin == PORTB_PIN2_ID_INDEX || Pin == PORTB_PIN3_ID_INDEX     
           ||Pin == PORTD_PIN0_ID_INDEX || Pin == PORTD_PIN1_ID_INDEX || Pin == PORTE_PIN4_ID_INDEX || Pin == PORTE_PIN5_ID_INDEX )
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000003);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      case PORT_PIN_MODE_UART:
        /* Check if the mode is valid for this pin */
        if( Pin == PORTA_PIN0_ID_INDEX || Pin == PORTA_PIN1_ID_INDEX || (Pin >= PORTC_PIN4_ID_INDEX && Pin <= PORTC_PIN7_ID_INDEX)
           || (Pin >= PORTD_PIN4_ID_INDEX && Pin <= PORTE_PIN1_ID_INDEX) || Pin == PORTE_PIN4_ID_INDEX || Pin == PORTE_PIN5_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000001);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      case PORT_PIN_MODE_UART1:
        /* Check if the mode is valid for this pin */
        if( Pin == PORTB_PIN0_ID_INDEX || Pin == PORTB_PIN1_ID_INDEX )
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000001);
        }
        else if( Pin == PORTC_PIN4_ID_INDEX || Pin == PORTC_PIN5_ID_INDEX )
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000002);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;  
        
      case PORT_PIN_MODE_UART1_CONTROL: 
        /* Check if the mode is valid for this pin */
        if( Pin == PORTF_PIN0_ID_INDEX || Pin == PORTF_PIN1_ID_INDEX )
        { 
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000001);
        }
        else if( Pin == PORTC_PIN4_ID_INDEX || Pin == PORTC_PIN5_ID_INDEX )
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000008);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break; 
        
      case PORT_PIN_MODE_USB_DIGITAL: 
        /* Check if the mode is valid for this pin */
        if(Pin == PORTC_PIN6_ID_INDEX || Pin == PORTC_PIN7_ID_INDEX || Pin == PORTD_PIN2_ID_INDEX || Pin == PORTD_PIN3_ID_INDEX || Pin == PORTF_PIN4_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000008);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      case PORT_PIN_MODE_JTAG:
        /* Check if the mode is valid for this pin */
        if(Pin >= PORTC_PIN0_ID_INDEX && Pin <= PORTC_PIN3_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000001);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      case PORT_PIN_MODE_ANALOG_COMPARATOR_OUTPUT:
        /* Check if the mode is valid for this pin */
        if(Pin == PORTF_PIN0_ID_INDEX || Pin == PORTF_PIN1_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x00000009);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break; 
        
      case PORT_PIN_MODE_CORE:
        /* Check if the mode is valid for this pin */
        if(Pin >= PORTF_PIN1_ID_INDEX && Pin <= PORTF_PIN3_ID_INDEX)
        {
          /*Function call to configure the required registers for this digital pin mode */
          Port_SetDigitalFunction(Port_Ptr, Pin, 0x0000000E);
        }
        else
        {
          /*Invalid mode for this pin*/
        }
        
        break;
        
      default: 
        
        /*Invalid mode*/
        
        break;
        
      }
           
           
    }  
           
           
  }
           
}
#endif
           