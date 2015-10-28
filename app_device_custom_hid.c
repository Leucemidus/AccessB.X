// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2015 Microchip Technology Inc. (www.microchip.com)
Modifications 2015 Omar Andrés Trevizo Rascón

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
*******************************************************************************/
//DOM-IGNORE-END

/** INCLUDES *******************************************************/
#include <usb.h>
#include <usb_device_hid.h>
#include <string.h>
#include <system.h>
#include <AccessB.h>
#include "UDF.h"


/** VARIABLES ******************************************************/
/* Some processors have a limited range of RAM addresses where the USB module
 * is able to access.  The following section is for those devices.  This section
 * assigns the buffers that need to be used by the USB module into those
 * specific areas.
 */
#if defined(FIXED_ADDRESS_MEMORY)
    #if defined(COMPILER_MPLAB_C18)
        #pragma udata HID_CUSTOM_OUT_DATA_BUFFER = HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS
        unsigned char ReceivedDataBuffer[64];
        #pragma udata HID_CUSTOM_IN_DATA_BUFFER = HID_CUSTOM_IN_DATA_BUFFER_ADDRESS
        unsigned char ToSendDataBuffer[64];
        #pragma udata

    #else defined(__XC8)
        unsigned char ReceivedDataBuffer[64] @ HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS;
        unsigned char ToSendDataBuffer[64] @ HID_CUSTOM_IN_DATA_BUFFER_ADDRESS;
    #endif
#else
    unsigned char ReceivedDataBuffer[64];
    unsigned char ToSendDataBuffer[64];
#endif

volatile USB_HANDLE USBOutHandle;    
volatile USB_HANDLE USBInHandle;

/** DEFINITIONS ****************************************************/
typedef enum {
    CONFIRM_CMD = 0xFF,
            RESET = 0xFE,
            I2C_NAK = 0xFD,
            I2C_ACK = 0xFC,
    READ_SFR = 0x01,
    WRITE_SFR = 0x02,
    ADC_CFG = 0x06,
    ADC_VALUE = 0x07,
    ANALOGCMP_CFG = 0x08,
            SPI_CFG = 0x0A,
            SPI_TRANSFERENCE = 0x0C,
            SFR_CHANGE_BIT_VALUE = 0x0D,
            I2C_CFG = 0x0E,
            I2C_TRANSFERENCE = 0x10,
            CCP_CFG = 0x11,
            PWM_FPWM = 0x12,
            PWM_DC = 0x13,
            EUSART_TX = 0x14,
            EUSART_RX = 0x15,
            SFR_READ_BIT_VALUE = 0x16,
            UDF_CALL = 0x18,
            UDF_PROGRAM = 0x19,
            ADC_START = 0xFB,
            ADC_STOP = 0xFA,
            TEST = 0xFF
} COMMUNICATION_CMDS;

/** FUNCTIONS ******************************************************/

/*********************************************************************
* Function: void APP_DeviceCustomHIDInitialize(void);
*
* Overview: Initializes the Custom HID demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCustomHIDInitialize()
{
    //initialize the variable holding the handle for the last
    // transmission
    USBInHandle = 0;

    //enable the HID endpoint
    USBEnableEndpoint(CUSTOM_DEVICE_HID_EP, USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = (volatile USB_HANDLE)HIDRxPacket(CUSTOM_DEVICE_HID_EP,(uint8_t*)&ReceivedDataBuffer,64);
}

/*********************************************************************
* Function: void APP_DeviceCustomHIDTasks(void);
*
* Overview: Keeps the Custom HID demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_DeviceCustomHIDInitialize() and APP_DeviceCustomHIDStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCustomHIDTasks()
{   
    //Check if we have received an OUT data packet from the host
    if(HIDRxHandleBusy(USBOutHandle) == false)
    {   
        //We just received a packet of data from the USB host.
        //Check the first uint8_t of the packet to see what command the host
        //application software wants us to fulfill.
        switch(ReceivedDataBuffer[0])				//Look at the data the host sent, to see what kind of application specific command it sent.
        {
            case READ_SFR:

                ReadSfr();
                break;

            case WRITE_SFR:

                WriteSfr();
                break;

            case ADC_CFG:

                AdcCfg();
                break;

            case ADC_VALUE:

                AdcValue();
                break;

            case ANALOGCMP_CFG:

                AnalogCmpCfg();
                break;

            case SPI_CFG:

                SpiCfg();
                break;

            case SPI_TRANSFERENCE:

                SpiTransference();
                break;

            case SFR_CHANGE_BIT_VALUE:

                SfrChangeBitValue();
                break;

            case I2C_CFG:

                I2cCfg();
                break;

            case I2C_TRANSFERENCE:

                I2cTransference();
                break;

            case CCP_CFG:

                CcpCfg();
                break;

            case PWM_FPWM:

                PwmFpwm();
                break;

            case PWM_DC:

                PwmDc();
                break;

            case SFR_READ_BIT_VALUE:

                SfrReadBitValue();
                break;

            case EUSART_TX:

                EusartTx();
                break;

            case EUSART_RX:

                EusartRx();
                break;

            case TEST:

                Test();
                break;

            case UDF_CALL:

                UDF();
                break;

            case UDF_PROGRAM:

                UDF_Program();
                break;
        }
        //Re-arm the OUT endpoint, so we can receive the next OUT data packet 
        //that the host may try to send us.
        USBOutHandle = HIDRxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ReceivedDataBuffer, 64);
    }
}