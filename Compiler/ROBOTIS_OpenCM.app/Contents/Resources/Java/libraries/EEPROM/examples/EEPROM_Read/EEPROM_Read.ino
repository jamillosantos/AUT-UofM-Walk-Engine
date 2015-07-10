/* EEPROM Read

 Demonstrates how to use flash memory as EEPROM in CM900, CM9.04 board.
 After write some data to flash memory using this example,
 execute Read example same category. it is better when power is off -> on

 New bootoader is needed to operate EEPROM library example.
 Because old bootloader always erase after flash address 0x08003000
 So, the data saved will be deleted after downloading new codes to cm9 board
 Please check the bootloader codes in gitHub and robotsource.org

                 Compatibility
 CM900                  O
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

#include "EEPROM.h"
int i;
EEPROM CM_EEPROM;
void setup(){  
  CM_EEPROM.begin();
}
void loop(){
  delay(1000);
  for(i=0;i < 10;i++){
    SerialUSB.print("EEPROM Example = "); 
    SerialUSB.println(CM_EEPROM.read(i));// read data from virtual address 0~9
  }
}


