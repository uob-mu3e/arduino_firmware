
//Test program to build functions for controlling HMP4040 Power Supply

// USES Arduino MEGA 2560
//HARDWARE:

// PC is connected to UART0 (built in USB connector on Arduino board)

// Rhode&Schwartz HMP4040 power supply is connected to UART 1
//(via MAX232A interface chip to generate RS232 voltage levels)

#include "HMP_4040.h"



                 //DEBUG mode will print lots of info out to the serial port while the system is running. 
#define DEBUG    // comment this line out to disable debug mode



void setup(){
  Serial.begin(9600);                 //Serial port for PC
  Serial1.begin(9600);                //Serial port for PSU
  delay(500); // let serial console settle
    #ifdef DEBUG
    Serial.println("SERIAL PORTS ENABLED");
    
    #endif
    PSU_Init();
    #ifdef DEBUG
    Serial.println("PSU INITIALISED");
    #endif
    delay(1000);
    PSU_Setup();
    #ifdef DEBUG
    Serial.println("PSU SETUP COMPLETE");
    #endif
    delay(1000);  
   
  }   //END SETUP



void loop() {
  
  PSU_Fan_Power_On();
  Serial1.println("SYST:BEEP");
  
  delay(1000);
  
  PSU_Heater_Power_On(10000);
  Serial1.println("SYST:BEEP");
  
  delay(10000);
    
  PSU_Heater_Power_Off();
  Serial1.println("SYST:BEEP");
  
  delay (1000);
  
  PSU_Fan_Power_Off();
  Serial1.println("SYST:BEEP");
  delay(5000);
  
  
}  // END MAIN LOOP
