


#include <ModeSelect.h>
/**********************************************
 *This function check type of reset and do the next:
 * -If was WDT reset then disable WDT and execute USB2UART mode firmware
 * -If was a normal reset, first power on, power cycle, then enable WDT that is
 * preconfigured with 8 second reset timer and then go to execute AccessB mode firmware
 * 
 * The user is responsible for disable the WDT when want to use the AccessB mode on a
 * 8 seconds windows just before the board is powered on, if not it resets and changue to
 * USB2UART firmware mode. 
 ***********************************************/
void __section("MS_Psect") ModeSelect(void)
{
    //Check kind of reset
    if(RCONbits.NOT_TO == 0)
    {
        //WDT reset confirmed
        WDTCONbits.SWDTEN = 0; //WDT off
        asm("goto 0x3F00");//GOTO USB2UART firmware
    }
    else
    {
        //Power on/normal reset
        WDTCONbits.SWDTEN = 1; //WDT on, WDT conf 8 second reset
        asm("goto 0xA2");//GOTO AccessB firmware
    }
}