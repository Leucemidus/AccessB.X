// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2015 Omar Andr閟 Trevizo Rasc髇

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

/**INCLUDES*****************************************************/
#include "plib.h"
#include <AccessB.h>
#include <usb.h>
#include <usb_device_hid.h>
#include <string.h>
#include <system.h>


/**GLOBAL VARIABLES********************************************/
extern volatile USB_HANDLE USBOutHandle;
extern volatile USB_HANDLE USBInHandle;
///TODO: ya no se usa
//extern volatile unsigned int BufferCounter;
//extern volatile unsigned int BufferPingPong;
unsigned int ADCON0Tmp; //Archivos temporales de ADC
unsigned int ADCON2Tmp;
unsigned int FSR0Ltemp;
unsigned int FSR0Htemp;

#if defined(FIXED_ADDRESS_MEMORY)
    #if defined(COMPILER_MPLAB_C18)
        #pragma udata HID_CUSTOM_OUT_DATA_BUFFER = HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS
        extern unsigned char ReceivedDataBuffer[64];
        #pragma udata HID_CUSTOM_IN_DATA_BUFFER = HID_CUSTOM_IN_DATA_BUFFER_ADDRESS
        extern unsigned char ToSendDataBuffer[64];
        #pragma udata

    #else defined(__XC8)
        extern unsigned char ReceivedDataBuffer[64] @ HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS;
        extern unsigned char ToSendDataBuffer[64] @ HID_CUSTOM_IN_DATA_BUFFER_ADDRESS;
    #endif
#else
    extern unsigned char ReceivedDataBuffer[64];
    extern unsigned char ToSendDataBuffer[64];
#endif


/**DEFINITIONS************************************************/
#define _XTAL_FREQ	48000000
#define Ping 0

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
            ADC_START = 0xFB,
            ADC_STOP = 0xFA,
            TEST = 0xFF,
            READY = 0xFF
} COMMUNICATION_CMDS;

typedef enum {
    CS_RA0 = 0,
CS_RA1 = 1,
CS_RA2 = 2,
CS_RA3 = 3,
CS_RA4 = 4,
CS_RA5 = 5,
CS_RB2 = 6,
CS_RB3 = 7,
CS_RB4 = 8,
CS_RB5 = 9,
CS_RB6 = 10,
CS_RB7 = 11,
CS_RC0 = 12,
CS_RC1 = 13,
CS_RC2 = 14,
CS_RC6 = 15
}SPI_CHIP_SELECT;


/**FUNCTIONS*****************************************************/

void ReadSfr()
{
	if (!HIDTxHandleBusy(USBInHandle)) 
	{
		int FSR0Ltemp;
		int FSR0Htemp;
		//Existen registros de 16 bits divididos en dos bytes, este codigo se usa para leer los 16 bits, si cualquiera de las direcciones siguientes es la que se envia, corresponde a uno de esos 12 registros de 16 bits
		if ((ReceivedDataBuffer[2] == 0x66) || (ReceivedDataBuffer[2] == 0xAF) || (ReceivedDataBuffer[2] == 0xB2) || (ReceivedDataBuffer[2] == 0xBB) || (ReceivedDataBuffer[2] == 0xBE) || (ReceivedDataBuffer[2] == 0xC3) || (ReceivedDataBuffer[2] == 0xCE) || (ReceivedDataBuffer[2] == 0xD6) || (ReceivedDataBuffer[2] == 0xD9) || (ReceivedDataBuffer[2] == 0xE1) || (ReceivedDataBuffer[2] == 0xE9) || (ReceivedDataBuffer[2] == 0xF3)) {
			FSR0Ltemp = FSR0L;//Context saving
			FSR0Htemp = FSR0H;//Context saving

			FSR0H = 0x0F;//Apunta al banco 15 donde estan todos los SFR
			FSR0L = ReceivedDataBuffer[2]; //AddressL, AddressH no se requiere para SFR

			ToSendDataBuffer[3] = POSTINC0; //DatosL
			ToSendDataBuffer[4] = INDF0; //DatosH
                        ToSendDataBuffer[5] = 0;
                        ToSendDataBuffer[6] = 0;

			ToSendDataBuffer[0] = READ_SFR;
			ToSendDataBuffer[1] = ReceivedDataBuffer[1];
			ToSendDataBuffer[2] = ReceivedDataBuffer[2];
			//Echo de regreso para confirmar que se recibio el comando
			USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
		} else //Lectura para registro de 8 bits
		{
			FSR0H = 0x0F; //Banco 15 donde estan todos los SFR
			FSR0L = ReceivedDataBuffer[2]; //Direcci髇 del SFR
			ToSendDataBuffer[3] = INDF0; //L
			ToSendDataBuffer[4] = 0; //H
                        ToSendDataBuffer[5] = 0;
                        ToSendDataBuffer[6] = 0;

			ToSendDataBuffer[0] = READ_SFR;
			ToSendDataBuffer[1] = ReceivedDataBuffer[1];
			ToSendDataBuffer[2] = ReceivedDataBuffer[2]; //Echo de la direccion leeida
			//Echo de regreso para confirmar que se recibio el comando
			USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
		}
		FSR0L = FSR0Ltemp;
		FSR0H = FSR0Htemp;
	}

}

void WriteSfr()
{
	if (!HIDTxHandleBusy(USBInHandle)) 
	{
		//Existen registros de 16 bits divididos en dos bytes, este codigo se usa para escribir los 16 bits, si cualquiera de las direcciones siguientes es la que se envia, corresponde a uno de esos 12 registros de 16 bits
		int FSR0Ltemp;
		int FSR0Htemp;
		if ((ReceivedDataBuffer[2] == 0x66) || (ReceivedDataBuffer[2] == 0xAF) || (ReceivedDataBuffer[2] == 0xB2) || (ReceivedDataBuffer[2] == 0xBB) || (ReceivedDataBuffer[2] == 0xBE) || (ReceivedDataBuffer[2] == 0xC3) || (ReceivedDataBuffer[2] == 0xCE) || (ReceivedDataBuffer[2] == 0xD6) || (ReceivedDataBuffer[2] == 0xD9) || (ReceivedDataBuffer[2] == 0xE1) || (ReceivedDataBuffer[2] == 0xE9) || (ReceivedDataBuffer[2] == 0xF3)) {
			FSR0Ltemp = FSR0L;//Context saving
			FSR0Htemp = FSR0H;//Context saving

			FSR0H = 0x0F; //AddrH El que se envia de Host no es necesario
			FSR0L = ReceivedDataBuffer[2]; //AddressL, AddressH no se requiere para SFR
			POSTINC0 = ReceivedDataBuffer[3]; //DataL
			INDF0 = ReceivedDataBuffer[4]; //DataH

			ToSendDataBuffer[0] = WRITE_SFR; //CMD
			ToSendDataBuffer[1] = ReceivedDataBuffer[1]; //AddrH
			ToSendDataBuffer[2] = ReceivedDataBuffer[2]; //AddrL
			//Echo de regreso para confirmar que se recibio el comando
			USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
		} else //Escritura en registro de 8 bits
		{
			FSR0H = 0x0F; //AddrH El que se envia del Host no es necesario
			FSR0L = ReceivedDataBuffer[2]; //AddrL
			INDF0 = ReceivedDataBuffer[3]; //DataL

			ToSendDataBuffer[0] = WRITE_SFR; //CMD
			ToSendDataBuffer[1] = ReceivedDataBuffer[1]; //AddrH
			ToSendDataBuffer[2] = ReceivedDataBuffer[2]; //AddrL Echo de la direccion modificada

			//Echo de regreso para confirmar que se recibio el comando
			USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
		}
		FSR0L = FSR0Ltemp;
		FSR0H = FSR0Htemp;
	}
}

void AdcCfg()
{
	ADCON0Tmp = ADCON0;
	ADCON2Tmp = ADCON2;
	//Solo altero bits de configuracion
	ADCON0 = ReceivedDataBuffer[1] | (ADCON0Tmp & 0x03);
	ADCON1 = ReceivedDataBuffer[2];
	ADCON2 = ReceivedDataBuffer[3]; // | (ADCON2Tmp & 0x80);

	//Configuraci贸n de los registros TRIS asociados al ADC segun configuracion
	switch (ADCON1 & 0xF) {
		case 0:
			TRISA = 0x2F;
			TRISB = 0x1F;
			break;

		case 1:
			TRISA = 0x2F;
			TRISB = 0x1F;
			break;

		case 2:
			TRISA = 0x2F;
			TRISB = 0x1F;
			break;

		case 3:
			TRISA = 0x2F;
			TRISB = 0x1E;
			break;

		case 4:
			TRISA = 0x2F;
			TRISB = 0x0D;
			break;

		case 5:
			TRISA = 0x2F;
			TRISB = 0x0C;
			break;

		case 6:
			TRISA = 0x2F;
			TRISB = 0x04;
			break;

		case 0x0A:
			TRISA = 0x2F;
			break;

		case 0x0B:
			TRISA = 0x0F;
			break;

		case 0x0C:
			TRISA = 0x07;
			break;

		case 0x0D:
			TRISA = 0x03;
			break;

		case 0x0E:
			TRISA = 0x01;
			break;

		case 0x0F:

			break;

	}
}

void AdcValue()
{
	if (!HIDTxHandleBusy(USBInHandle)) {
		//Confirmaci贸n de recepcion
		ToSendDataBuffer[0] = ADC_VALUE; //Eco del comando enviado
		ToSendDataBuffer[1] = CONFIRM_CMD; //Confirmaci贸n de recepcion de comando
		USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);

		//Espero a que se pueda transmitir de nuevo
		while (HIDTxHandleBusy(USBInHandle)) {
		}

		//Inicia conversion
		if (!HIDTxHandleBusy(USBInHandle)) {
			//Channel selection

			ADCON0bits.CHS = ReceivedDataBuffer[1];
			ADCON0bits.GODONE = 1;
			while (ADCON0bits.GODONE) {
			} //Espero a que termine la conversion
			ToSendDataBuffer[0] = ADC_VALUE; //CMD
			ToSendDataBuffer[1] = ADRESL; //L
			ToSendDataBuffer[2] = ADRESH; //H
			USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
		}
	}
}

void AnalogCmpCfg()
{
	//Cargo configuraci贸n al modulo de comparadores analogos
	CMCON = ReceivedDataBuffer[1];

	switch (CMCON & 0x07) {
		case 0:
			TRISA = 0b00001111;
			break;

		case 1:
			TRISA = 0b00001001;
			break;

		case 2:
			TRISA = 0b00001111;
			break;

		case 3:
			TRISA = 0b00001111;
			break;

		case 4:
			TRISA = 0b00001011;
			break;

		case 5:
			TRISA = 0b00001011;
			break;

		case 6:
			TRISA = 0b00001111;
			break;

		case 7:
			TRISA = 0b00000000;
	}
}

void SpiCfg()
{
	SSPSTAT = ReceivedDataBuffer[1];
	SSPCON1 = ReceivedDataBuffer[2];

	switch(SSPCON1 & 0b00001100)
	{
		case 0b00000000: //Master mode
			TRISBbits.RB0 = 1;//SDI in
			TRISBbits.RB1 = 0;//SCL out
			TRISCbits.RC7 = 0;//SDO out
//                        TRISAbits.RA5 = 0;//CS out
//                        PORTAbits.RA5 = 1;//CS en alto desde el inicio
			break;

		case 0b00000100: //Slave mode
			TRISBbits.RB0 = 1; //SDI in
			TRISBbits.RB1 = 1; //SCL in
			TRISCbits.RC7 = 0; //SDO out
//                        TRISAbits.RA5 = 1; //SS in
			break;
	}
}

void SpiTransference()
{
	if (!HIDTxHandleBusy(USBInHandle))
	{
		//Confirmaci贸n de recepcion
		ToSendDataBuffer[0] = SPI_TRANSFERENCE; //Eco del comando enviado
		ToSendDataBuffer[1] = CONFIRM_CMD; //Confirmaci贸n de recepcion de comando
		USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);

		//Espero a que se pueda transmitir de nuevo
		while (HIDTxHandleBusy(USBInHandle)) {}

		//Inicia transferencia SPI
		if (!HIDTxHandleBusy(USBInHandle))
		{
			int counter = 0;
			if(ReceivedDataBuffer[1] > ReceivedDataBuffer[2])
			{
				counter = ReceivedDataBuffer[1];
			}
			else
			{
				counter = ReceivedDataBuffer[2];
			}
			for(int i = 0; i < counter; i++) //-1
			{
				switch(ReceivedDataBuffer[3]) //Chip select
				{
					case CS_RA0:
						if(TRISAbits.RA0 == 1)
						{
							TRISAbits.RA0 = 0;
							LATAbits.LA0 = 1;
							__delay_ms(1);
						}
						LATAbits.LA0 = 0;
						break;
					case CS_RA1:
						if(TRISAbits.RA1 == 1)
						{
							TRISAbits.RA1 = 0;
							LATAbits.LA1 = 1;
							__delay_ms(1);
						}
						LATAbits.LA1 = 0;
						break;
					case CS_RA2:
						if(TRISAbits.RA2 == 1)
						{
							TRISAbits.RA2 = 0;
							LATAbits.LA2 = 1;
							__delay_ms(1);
						}
						LATAbits.LA2 = 0;
						break;
					case CS_RA3:
						if(TRISAbits.RA3 == 1)
						{
							TRISAbits.RA3 = 0;
							LATAbits.LA3 = 1;
							__delay_ms(1);
						}
						LATAbits.LA3 = 0;
						break;
					case CS_RA4:
						if(TRISAbits.RA4 == 1)
						{
							TRISAbits.RA4 = 0;
							LATAbits.LA4 = 1;
							__delay_ms(1);
						}
						LATAbits.LA4 = 0;
						break;
					case CS_RA5:
						if(TRISAbits.RA5 == 1)
						{
							TRISAbits.RA5 = 0;
							LATAbits.LA5 = 1;
							__delay_ms(1);
						}
						LATAbits.LA5 = 0;
						break;
					case CS_RB2:
						if(TRISBbits.RB2 == 1)
						{
							TRISBbits.RB2 = 0;
							LATBbits.LB2 = 1;
							__delay_ms(1);
						}
						LATBbits.LB2 = 0;
						break;
					case CS_RB3:
						if(TRISBbits.RB3 == 1)
						{
							TRISBbits.RB3 = 0;
							LATBbits.LB3 = 1;
							__delay_ms(1);
						}
						LATBbits.LB3 = 0;
						break;
					case CS_RB4:
						if(TRISBbits.RB4 == 1)
						{
							TRISBbits.RB4 = 0;
							LATBbits.LB4 = 1;
							__delay_ms(1);
						}
						LATBbits.LB4 = 0;
						break;
					case CS_RB5:
						if(TRISBbits.RB5 == 1)
						{
							TRISBbits.RB5 = 0;
							LATBbits.LB5 = 1;
							__delay_ms(1);
						}
						LATBbits.LB5 = 0;
						break;
					case CS_RB6:
						if(TRISBbits.RB6 == 1)
						{
							TRISBbits.RB6 = 0;
							LATBbits.LB6 = 1;
							__delay_ms(1);
						}
						LATBbits.LB6 = 0;
						break;
					case CS_RB7:
						if(TRISBbits.RB7 == 1)
						{
							TRISBbits.RB7 = 0;
							LATBbits.LB7 = 1;
							__delay_ms(1);
						}
						LATBbits.LB7 = 0;
						break;
					case CS_RC0:
						if(TRISCbits.RC0 == 1)
						{
							TRISCbits.RC0 = 0;
							LATCbits.LC0 = 1;
							__delay_ms(1);
						}
						LATCbits.LC0 = 0;
						break;
					case CS_RC1:
						if(TRISCbits.RC1 == 1)
						{
							TRISCbits.RC1 = 0;
							LATCbits.LC1 = 1;
							__delay_ms(1);
						}
						LATCbits.LC1 = 0;
						break;
					case CS_RC2:
						if(TRISCbits.RC2 == 1)
						{
							TRISCbits.RC2 = 0;
							LATCbits.LC2 = 1;
							__delay_ms(1);
						}
						LATCbits.LC2 = 0;
						break;
					case CS_RC6:
						if(TRISCbits.RC6 == 1)
						{
							TRISCbits.RC6 = 0;
							LATCbits.LC6 = 1;
							__delay_ms(1);
						}
						LATCbits.LC6 = 0;
						break;
				}

				if(i < ReceivedDataBuffer[2]) //Numero de bytes a recibir
				{
					SSPBUF = ReceivedDataBuffer[i + 4];
					while(!SSPSTATbits.BF){}
					ToSendDataBuffer[i] = SSPBUF;
				}
				else
				{
					SSPBUF = 0xA9; //Dummy data byte
					while(!SSPSTATbits.BF){}
					ToSendDataBuffer[i] = SSPBUF;
				}
				switch(ReceivedDataBuffer[3]) //Chip select
				{
					case CS_RA0:
						LATAbits.LA0 = 1;
						break;
					case CS_RA1:
						LATAbits.LA1 = 1;
						break;
					case CS_RA2:
						LATAbits.LA2 = 1;
						break;
					case CS_RA3:
						LATAbits.LA3 = 1;
						break;
					case CS_RA4:
						LATAbits.LA4 = 1;
						break;
					case CS_RA5:
						LATAbits.LA5 = 1;
						break;
					case CS_RB2:
						LATBbits.LB2 = 1;
						break;
					case CS_RB3:
						LATBbits.LB3 = 1;
						break;
					case CS_RB4:
						LATBbits.LB4 = 1;
						break;
					case CS_RB5:
						LATBbits.LB5 = 1;
						break;
					case CS_RB6:
						LATBbits.LB6 = 1;
						break;
					case CS_RB7:
						LATBbits.LB7 = 1;
						break;
					case CS_RC0:
						LATCbits.LC0 = 1;
						break;
					case CS_RC1:
						LATCbits.LC1 = 1;
						break;
					case CS_RC2:
						LATCbits.LC2 = 1;
						break;
					case CS_RC6:
						LATCbits.LC6 = 1;
						break;
				}
			}
		}
		USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	 }
}

void SfrChangeBitValue()
{
	FSR0Ltemp = FSR0L;//Context saving
	FSR0Htemp = FSR0H;//Context saving

	FSR0H = 0x0F;//Siempre es el mismo valor para los SFR
	FSR0L = ReceivedDataBuffer[2];

	if(ReceivedDataBuffer[4] == 1)
	{
		switch(ReceivedDataBuffer[3])
		{
			case 0:
				asm("BANKSEL(INDF0)");
				asm("BSF INDF0, 0, 0");
				break;

			case 1:
				asm("BANKSEL(INDF0)");
				asm("BSF INDF0, 1, 0");
				break;

			case 2:
				asm("BANKSEL(INDF0)");
				asm("BSF INDF0, 2, 0");
				break;

			case 3:
				asm("BANKSEL(INDF0)");
				asm("BSF INDF0, 3, 0");
				break;

			case 4:
				asm("BANKSEL(INDF0)");
				asm("BSF INDF0, 4, 0");
				break;

			case 5:
				asm("BANKSEL(INDF0)");
				asm("BSF INDF0, 5, 0");
				break;

			case 6:
				asm("BANKSEL(INDF0)");
				asm("BSF INDF0, 6, 0");
				break;

			case 7:
				asm("BANKSEL(INDF0)");
				asm("BSF INDF0, 7, 0");
				break;
		}
	}

	if(ReceivedDataBuffer[4] == 0)
	{
		switch(ReceivedDataBuffer[3])
		{
			case 0:
				asm("BANKSEL(INDF0)");
				asm("BCF INDF0, 0, 0");
				break;

			case 1:
				asm("BANKSEL(INDF0)");
				asm("BCF INDF0, 1, 0");
				break;

			case 2:
				asm("BANKSEL(INDF0)");
				asm("BCF INDF0, 2, 0");
				break;

			case 3:
				asm("BANKSEL(INDF0)");
				asm("BCF INDF0, 3, 0");
				break;

			case 4:
				asm("BANKSEL(INDF0)");
				asm("BCF INDF0, 4, 0");
				break;

			case 5:
				asm("BANKSEL(INDF0)");
				asm("BCF INDF0, 5, 0");
				break;

			case 6:
				asm("BANKSEL(INDF0)");
				asm("BCF INDF0, 6, 0");
				break;

			case 7:
				asm("BANKSEL(INDF0)");
				asm("BCF INDF0, 7, 0");
				break;
		}
	}
	FSR0L = FSR0Ltemp;
	FSR0H = FSR0Htemp;
}

void I2cCfg()
{
	SSPSTAT = ReceivedDataBuffer[1];
	SSPCON1 = ReceivedDataBuffer[2];
	SSPADD = ReceivedDataBuffer[3];
	TRISBbits.RB0 = 1; //SDA SCL Open drain input
	TRISBbits.RB1 = 1;
}

void I2cTransference()
{
	//ReceivedDataBuffer[1] ADDRESSH
	//ReceivedDataBuffer[2] ADDRESSL with Read/Write bit
	//REceivedDataBuffer[3] ADDRESS TYPE 7-10 bits
	//ReceivedDataBuffer[4] Number of Tx bytes
	//ReceivedDataBuffer[5] Number of Rx bytes
	//ReceivedDataBuffer[6] Restart?
	//ReceivedDataButter[7-63] Data to be TX

	if (!HIDTxHandleBusy(USBInHandle))
	{
		//Confirmaci贸n de recepcion
		ToSendDataBuffer[0] = I2C_TRANSFERENCE; //Eco del comando enviado
		ToSendDataBuffer[1] = CONFIRM_CMD; //Confirmaci贸n de recepcion de comando
		USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);

		//Espero a que se pueda transmitir de nuevo
		while (HIDTxHandleBusy(USBInHandle)) {}

		 //Inicio la transferencia de datos
		if (!HIDTxHandleBusy(USBInHandle))
		{
			PIR1bits.SSPIF = 0;
			//Start pattern
			SSPCON2bits.SEN = 1;
			while(!PIR1bits.SSPIF){}
			PIR1bits.SSPIF = 0;
			//Address W/R byte
			if(ReceivedDataBuffer[3] == 7)//7 bit address
			{
				SSPBUF = ReceivedDataBuffer[2]; //Slave Address W/R bit
				while(!PIR1bits.SSPIF){}
				PIR1bits.SSPIF = 0;
				if(!SSPCON2bits.ACKSTAT == 0) 
				{
					SSPCON2bits.PEN = 1; //Inicio secuencia de paro
					while(!PIR1bits.SSPIF){}
					PIR1bits.SSPIF = 0;
					 ToSendDataBuffer[0] = I2C_NAK;
					USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
					return;//break; //No ACK from slave
				}
			}
			else if(ReceivedDataBuffer[3] == 10)//10 bit address
			{
				SSPBUF = ReceivedDataBuffer[1]; //Slave Address High byte W/R bit
				while(!PIR1bits.SSPIF){}
				PIR1bits.SSPIF = 0;
				if(!SSPCON2bits.ACKSTAT == 0)
				{
					SSPCON2bits.PEN = 1; //Inicio secuencia de paro
					while(!PIR1bits.SSPIF){}
					PIR1bits.SSPIF = 0;
					ToSendDataBuffer[0] = I2C_NAK;
					USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
					return;//break; //No ACK from slave
				}
				SSPBUF = ReceivedDataBuffer[2]; //Slave Address Low byte
				while(!PIR1bits.SSPIF){}
				PIR1bits.SSPIF = 0;
				if(!SSPCON2bits.ACKSTAT == 0)
				{
					SSPCON2bits.PEN = 1; //Inicio secuencia de paro
					while(!PIR1bits.SSPIF){}
					PIR1bits.SSPIF = 0;
					ToSendDataBuffer[0] = I2C_NAK;
					USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
					return;//break;  //No ACK from slave
				}
			}

			//Tx
			for(int i = 0; i < ReceivedDataBuffer[4]; i++) //Zero based Index
			{
				SSPBUF = ReceivedDataBuffer[i + 7];
				while(!PIR1bits.SSPIF){}
				PIR1bits.SSPIF = 0;
				if(!SSPCON2bits.ACKSTAT == 0)
				{
					SSPCON2bits.PEN = 1; //Inicio secuencia de paro
					while(!PIR1bits.SSPIF){}
					PIR1bits.SSPIF = 0;
					ToSendDataBuffer[0] = I2C_NAK;
					return;//break; //No ACK from slave, out of FOR cycle
				}
			}
			if(ToSendDataBuffer[0] == I2C_NAK)
			{
				USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
				return; //break;
			}

			//En caso de reinicio el codigo debe de ir aqui antes de leer
			//Restart
			if(ReceivedDataBuffer[6] > 0)
			{
				SSPCON2bits.RSEN = 1; //inicio condicion de reinicio
				while(!PIR1bits.SSPIF){}
				PIR1bits.SSPIF = 0;
				//Address W/R byte
				if(ReceivedDataBuffer[3] == 7)//7 bit address
				{
					if(ReceivedDataBuffer[6] == 1)//Repeated Read
					{
						SSPBUF = ReceivedDataBuffer[2] | 0x01; //Slave Address Read bit
					}
					else if(ReceivedDataBuffer[6] == 2)
					{
						SSPBUF = ReceivedDataBuffer[2] & 0xFE; //Slave Address write bit
					}
					while(!PIR1bits.SSPIF){}
					PIR1bits.SSPIF = 0;
					if(!SSPCON2bits.ACKSTAT == 0)
					{
						SSPCON2bits.PEN = 1; //Inicio secuencia de paro
						while(!PIR1bits.SSPIF){}
						PIR1bits.SSPIF = 0;
						ToSendDataBuffer[0] = I2C_NAK;
						USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
						return;//break; //No ACK from slave
					}
				}
				else if(ReceivedDataBuffer[3] == 10)//10 bit address
				{
					if(ReceivedDataBuffer[6] == 1)//Repeated Read
					{
						SSPBUF = ReceivedDataBuffer[1] | 0x01; //Slave Address Read bit
					}
					else if(ReceivedDataBuffer[6] == 2)
					{
						SSPBUF = ReceivedDataBuffer[1] & 0xFE; //Slave Address write bit
					}
					while(!PIR1bits.SSPIF){}
					PIR1bits.SSPIF = 0;
					if(!SSPCON2bits.ACKSTAT == 0)
					{
						SSPCON2bits.PEN = 1; //Inicio secuencia de paro
						while(!PIR1bits.SSPIF){}
						PIR1bits.SSPIF = 0;
						ToSendDataBuffer[0] = I2C_NAK;
						USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
						return;//break; //No ACK from slave
					}
					SSPBUF = ReceivedDataBuffer[2]; //Slave Address Low byte
					while(!PIR1bits.SSPIF){}
					PIR1bits.SSPIF = 0;
					if(!SSPCON2bits.ACKSTAT == 0)
					{
						SSPCON2bits.PEN = 1; //Inicio secuencia de paro
						while(!PIR1bits.SSPIF){}
						PIR1bits.SSPIF = 0;
						ToSendDataBuffer[0] = I2C_NAK;
						USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
						return;//break;  //No ACK from slave
					}
				}
			}

			//Rx
			if(ReceivedDataBuffer[5] > 0)//Si se cumple la condici贸n quiere decir que hay datos que recibir
			{
				for(int i = 0; i < ReceivedDataBuffer[5]; i++)
				{
					while(SSPSTATbits.RW | SSPCON2bits.ACKEN | SSPCON2bits.SEN | SSPCON2bits.RSEN | SSPCON2bits.RCEN | SSPCON2bits.PEN){}
					SSPCON2bits.RCEN = 1;
					while(!PIR1bits.SSPIF){}//Rx complete?
					PIR1bits.SSPIF = 0;
					ToSendDataBuffer[i + 1] = SSPBUF;// ToSendDataBuffer[0] is reserved for NAK indicator
					if(i == (ReceivedDataBuffer[5] - 1))//No envio ACK en el ultimo byte recibido
					{
						SSPCON2bits.ACKDT = 1; //No envio ACK por que es el ultimo byte a leer
					}
					else
					{
						SSPCON2bits.ACKDT = 0; //Si envio ACK
					}
					SSPCON2bits.ACKEN = 1; //Inicio secuencia para enviar ACK al slave
					while(!PIR1bits.SSPIF){}
					PIR1bits.SSPIF = 0;
				}
			}
			SSPCON2bits.PEN = 1; //Inicio secuencia de paro
			while(!PIR1bits.SSPIF){}
			PIR1bits.SSPIF = 0;
		}
	}
	ToSendDataBuffer[0] = I2C_ACK;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
}

void CcpCfg()
{
	if(ReceivedDataBuffer[1] == 1) //CCP1
	{
		CCP1CON = ReceivedDataBuffer[2];
		if(ReceivedDataBuffer[2] > 7 || ReceivedDataBuffer[2] == 2) 
		{
			TRISCbits.RC2 = 0; //Compare or PWM
		}
		else
		{
			TRISCbits.RC2 = 1; //Capture
		}
	}
	else if(ReceivedDataBuffer[1] == 2) //CCP2
	{
		CCP2CON = ReceivedDataBuffer[2];
		if(ReceivedDataBuffer[2] > 7 || ReceivedDataBuffer[2] == 2)
		{
			TRISCbits.RC1 = 0; //Compare or PWM
		}
		else
		{
			TRISCbits.RC1 = 1; //Capture
		}

	}
}

void PwmFpwm()
{
	PR2 = ReceivedDataBuffer[1];
	//T2CON = ReceivedDataBuffer[2]; // Prescaler value
	T2CONbits.T2CKPS = ReceivedDataBuffer[2]; //Prescaer value
}

void PwmDc()
{
	if(ReceivedDataBuffer[1] == 1)
	{
		CCPR1L = ReceivedDataBuffer[3];
		CCP1CON = CCP1CON & ReceivedDataBuffer[2];
	}
	else if (ReceivedDataBuffer[1] == 2)
	{
		CCPR2L = ReceivedDataBuffer[3];
		CCP2CON = CCP2CON & ReceivedDataBuffer[2];
	}
}

void SfrReadBitValue()
{
	FSR0Ltemp = FSR0L;//Context saving
	FSR0Htemp = FSR0H;//Context saving
	FSR0H = 0x0F;//Siempre es el mismo valor para los SFR
	FSR0L = ReceivedDataBuffer[2];
	switch(ReceivedDataBuffer[3])
	{
		case 0:
			if((INDF0 & 0x01) == 0x01)
			{
				ToSendDataBuffer[1] = 1;
			}
			else
			{
				ToSendDataBuffer[1] = 0;
			}
			break;

		case 1:
			if((INDF0 & 0x02) == 0x02)
			{
				ToSendDataBuffer[1] = 1;
			}
			else
			{
				ToSendDataBuffer[1] = 0;
			}
			break;

		case 2:
			if((INDF0 & 0x04) == 0x04)
			{
				ToSendDataBuffer[1] = 1;
			}
			else
			{
				ToSendDataBuffer[1] = 0;
			}
			break;

		case 3:
			if((INDF0 & 0x08) == 0x08)
			{
				ToSendDataBuffer[1] = 1;
			}
			else
			{
				ToSendDataBuffer[1] = 0;
			}
			break;

		case 4:
			if((INDF0 & 0x10) == 0x10)
			{
				ToSendDataBuffer[1] = 1;
			}
			else
			{
				ToSendDataBuffer[1] = 0;
			}
			break;

		case 5:
			if((INDF0 & 0x20) == 0x20)
			{
				ToSendDataBuffer[1] = 1;
			}
			else
			{
				ToSendDataBuffer[1] = 0;
			}
			break;

		case 6:
			if((INDF0 & 0x40) == 0x40)
			{
				ToSendDataBuffer[1] = 1;
			}
			else
			{
				ToSendDataBuffer[1] = 0;
			}
			break;

		case 7:
			if((INDF0 & 0x80) == 0x80)
			{
				ToSendDataBuffer[1] = 1;
			}
			else
			{
				ToSendDataBuffer[1] = 0;
			}
			break;
	}
	FSR0L = FSR0Ltemp;
	FSR0H = FSR0Htemp;
	ToSendDataBuffer[0] = SFR_READ_BIT_VALUE;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
}

void EusartRx()
{

	//ReceivedDataBuffer[1] Continous or single reception
        //ReceivedDataBuffer[2] Number of bytes to be received
	///FIXME: SREN y CREN afectan diferente el modulo cuando es sincrono o asincrono tengo que resolver eso

    if(TXSTAbits.SYNC == 1)//Modo Sincrono
    {
        if(ReceivedDataBuffer[1] == 1)
        {
                RCSTAbits.SREN = 1; //Single receive
        }
        else if(ReceivedDataBuffer[1] == 0)
        {
                RCSTAbits.CREN = 1; //Continous Receive
        }
    }

    for(int n = 0; n <= ReceivedDataBuffer[2] - 1; n++)
    {
            while(!PIR1bits.RCIF);//Metodo bloqueante para el modo asincrono
            PIR1bits.RCIF = 0;
            ToSendDataBuffer[n] = RCREG;
    }

    if(TXSTAbits.SYNC == 1)  //Para modo sincron
    {
        RCSTAbits.CREN = 0;
    }

    //RCSTAbits.CREN = 0; //Disable continous reception
    USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
}

void EusartTx()
{
	//ReceivedDataBuffer[0] Command
	//ReceivedDataBuffer[1] Number of bytes to be TX
	//ReceivedDataBuffer[2-64] Data to be TX

	for(int n = 0; n <= ReceivedDataBuffer[1] - 1; n++)
	{
		TXREG = ReceivedDataBuffer[n+2];
		while(!TXSTAbits.TRMT); //TRMT == 1 buffer vacio
	}
}

void Test()
{
	ToSendDataBuffer[0] = 1;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 2;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 3;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 4;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 5;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 6;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 7;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 8;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 9;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 10;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 11;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 12;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 13;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 14;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 15;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 16;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 17;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 18;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 19;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
	while (HIDTxHandleBusy(USBInHandle)) { }
	ToSendDataBuffer[0] = 20;
	USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*) & ToSendDataBuffer[0], 64);
}
