// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2015 Omar Andrés Trevizo Rascón

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

/*******************************************************************************
 * User Defined Function (UDF) Manual.
 *
 * In this source file you can create "extra" functions that aren't covered in the
 * current AccessB firmware and that you may need in you application, all your
 * code must be writte inside the UDF function, there are some rules so please
 * read them before you start coding:
 *
 * RULES:
 *
 * 1.- Do not use High Priority Interruptions, High Priority Interruptions
 * are for Access B USB module use ONLY.
 * 2.- NEVER modify any of the USB module Special Function Registers (SFR),
 * doing that may cause unexpected USB operation.
 * 3.- Make sure that all you executable code remains inside the UDF,
 * this because the UDF has a reserved memory space from 0x3AEE - 0x7FFE and
 * that's the reason for why UDF is an absolute function. How I can
 * make code remain inside that memory range? the answer is simple, write all
 * your code inside UDF and DO NOT call others functions, exception to
 * this rule are functions from Access B firmware:
 *
 * EXAMPLE 1
 * You need to send/receive data from the USB module, you can use the functions
 * that are already on Access B firmware to send/receive data from USB without
 * problems.
 *
 * EXAMPLE 2
 * You have your own functions in a source C file and you want add them to this
 * project and call that functions from the UDF. In that case even when the
 * project compile without problems it would cause unexpected code execution,
 * that's because several reasons:
 *
 * When you add new functions that aren't inside UDF the compiler will reorder
 * all the AccessB firmware to fit the new functions but will not touch the
 * reserved program memory used for UDF, when you take the resulting hex file to
 * add the UDF to AccessB the software that do the programming will only extract
 * and program the reserved program memory space from the hex file (where the UDF resides)
 * but it will leave intact the rest of the program memory thus NOT programming the new
 * functions, then when you call the UDF and the code inside UDF call your
 * function the execution of the code will jump to a program memory location
 * where unknown code resides and leaving to unexpected behaivor.
 * There are only two solutions to this: one is programming PIC using
 * a PIC programmer like PicKit3 and program all the flash, the other is to make
 * your functions absolute and giving a position inside the reserverd program
 * memory space but withtout overlapping it with UDF or the others functions, for
 * this you will need to know the program memory size of each function and the UDF.
 *
 * When your code is ready, you must compile the entire AccessB project and then
 * go to your project folder named "production":
 *
 * MPLABXProjects
 * |-> AccessB.X
 *     |-> dist
 *         |-> 18F2550
 *             |-> production
 *
 * and search for AccessB.hex file, that file is required by the
 * AccessB UDF programmer sofware, simply connect your AccessB clikc on detect,
 * and open hex, select the hex file and then click GO!, and that's all.
 *
 * Calling your UDF is done using the Methods implemented in the AccessB class
 * file, so please refer to that file and read.
 *
 * XC8 linker options:
 * Extra Linker Options: -L-AAccB_Class=4280h-7EC0h -L-PAccB_Psect=AccB_Class
 * ROM range: default,-4280-7FFF
 *
 * **Maximum UDF program size = 7EC0h-4280h = 3C40h bytes = 15424 bytes
 *
 * *************************************************************************** /


/*********************Includes section******************************/
#include "xc.h"
#include <usb.h>
#include <usb_device_hid.h>
#include <system.h>

/***********************Define section******************************/


/**************************Global variables*************************/
extern volatile USB_HANDLE USBOutHandle;
extern volatile USB_HANDLE USBInHandle;
extern unsigned char ReceivedDataBuffer[64] @ HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS;
extern unsigned char ToSendDataBuffer[64] @ HID_CUSTOM_IN_DATA_BUFFER_ADDRESS;

/*********************************************************************
 * UDF: User Defined Function
 *
 * Here is where you place your own code to be programmed as
 * a new function of Access B.
 ********************************************************************/
void __section("AccB_Psect") UDF(void) //This function is placed in AccB_Psect psect, that is anywhere in the range of memory 0x3EAA-0x7FFE
{
    //Writte here your own code.
}
/********************************************************************
 * Writte here your UDF ISR, please remember that UDF ISR is low
 * priority interrupt ONLY, and you must not use High priority
 * interrupts because that are for USB module use ONLY.
 *
 * NOTE: Timer 0 don't have low priority interrupt :-(
 *
 * This is the Interrupt Service Rutine that is placed in AccB_Psect
 * psect.
 * ******************************************************************/
void __section("AccB_Psect") UDF_ISR(void) //This is the Interrupt Service Rutine that is placed in AccB_Psect psect.
{
    //Writte here your ISR.
}
