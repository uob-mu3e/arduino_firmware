#include "Arduino.h"
#include "HMP_4040.h"

void PSU_Init(){
  // code here to initialise the PSU from a 'cold start'
  // checks the PSU is connected and responding
  // Initialises all channels to 0v and turns channel output off
  Serial1.println("*IDN?");
  while(Serial1.available()==0){}
  String identity = Serial1.readString();
  #ifdef DEBUG
  Serial.print("PSU identification: ");
  Serial.print(identity);
  #endif
  //turning the voltage on all channels to 0
  for (int i=1; i<5; i++) {
    Channel_Select(i);
    Serial1.println("OUTP 0");
    delay(100);
    Serial1.println("VOLT 0");
    delay(100);
    Serial1.println("CURR 0");
    delay(100);
  //checking if the output is really off  
    Serial1.println("OUTP?");
    while (Serial1.available()==0){}
    int output_init=Serial1.parseInt();
    if (output_init != 0) {
      #ifdef DEBUG
      Serial.println("Output error");
      #endif
      break;
    }
  }
}





void PSU_Setup(){
    Channel_Select(PSU_FAN_CHANNEL);
    Channel_Set(PSU_FAN_VOLTAGE, PSU_FAN_CURRENT_LIM);

    Channel_Select(PSU_MUPIX_CHANNEL);
    Channel_Set(PSU_MUPIX_VOLTAGE, PSU_MUPIX_CURRENT_LIM);
   }


void PSU_Heater_Power_On(double power){  //power is in mW

  float isquared;
  float i;

  isquared = (power/1000)/PSU_HEATERS_R;
  i = sqrt(isquared);

  #ifdef DEBUG
  Serial.print("Power: ");
  Serial.print(power);
  Serial.print(" I2: ");
  Serial.print(isquared);
  Serial.print(" I= ");
  Serial.print(i);
  Serial.println("A");
  #endif
  
  Channel_Select(PSU_HEATERS_CHANNEL);
  Channel_Set(PSU_HEATERS_VOLTAGE_LIM, i);
  #ifdef DEBUG
  Serial.print("Heater ON: ");
  #endif
  Serial1.println("OUTPT 1");
  delay(100);
  Serial1.println("OUTP?");
  while (Serial1.available()==0){}
  int output_init=Serial1.parseInt();
  if (output_init == 1) {
    #ifdef DEBUG
    Serial.println("OK");
    #endif
    } else {
      #ifdef DEBUG
      Serial.println("ERR");
      #endif
    }
}


void PSU_Heater_Power_Off(){
  Channel_Select(PSU_HEATERS_CHANNEL);
  #ifdef DEBUG
  Serial.print("Heater OFF: ");
  #endif
  Serial1.println("OUTPT 0");
  delay(100);
  Serial1.println("OUTP?");
  while (Serial1.available()==0){}
  int output_init=Serial1.parseInt();
  if (output_init == 0) {
    #ifdef DEBUG
    Serial.println("OK");
    #endif
    } else {
      #ifdef DEBUG
      Serial.println("ERR");
      #endif
    }
}


void PSU_Fan_Power_On(){
    Channel_Select(PSU_FAN_CHANNEL);
    #ifdef DEBUG
    Serial.print("Output ON: ");
    #endif
    Serial1.println("OUTPT 1");
    delay(100);
    Serial1.println("OUTP?");
    while (Serial1.available()==0){}
    int output_init=Serial1.parseInt();
    if (output_init == 1) {
      #ifdef DEBUG
      Serial.println("OK");
      #endif
      } else {
        #ifdef DEBUG
        Serial.println("ERR");
        #endif 
      }
    }
  

void PSU_Fan_Power_Off(){
    Channel_Select(PSU_FAN_CHANNEL);
    #ifdef DEBUG
    Serial.print("Output OFF: ");
    #endif
    Serial1.println("OUTPT 0");
    delay(100);
    Serial1.println("OUTP?");
    while (Serial1.available()==0){}
    int output_init=Serial1.parseInt();
    if (output_init == 0) {
      #ifdef DEBUG
      Serial.println("OK");
      #endif
      } else {
        #ifdef DEBUG
        Serial.println("ERR");
        #endif
      }
    }
  
       
  


//This function selects a channel on the HMP4040 
void Channel_Select(int n) {
  String cmd="INST OUT";
  String n_str=String(n);
  String tot=cmd+n_str;
  Serial1.println(tot);
  delay(500);
  #ifdef DEBUG
  Serial.print("Selected Channel: ");
  Serial.println(n); 
  #endif
  }



// This function transmits a voltage setpoint and current limit to the HMP4040
void Channel_Set (float voltage_setpoint, float current_limit_setpoint){

float volt_reading;
float current_reading;  
        
    Serial1.print("VOLT ");
    Serial1.print(voltage_setpoint);
    delay(100);
    Serial1.println("OUTP 0");
    delay(100);
    
    //check whether the output voltage=setpoint
    Serial1.println("VOLT?");
    while (Serial1.available()==0){}    //wait for reply from PSU
    volt_reading = Serial1.parseFloat();
    #ifdef DEBUG
    Serial.print("Output voltage: ");
    Serial.println(volt_reading);
    #endif
    if (volt_reading<(voltage_setpoint+PSU_VOLT_TOLERANCE) && volt_reading>(voltage_setpoint-PSU_VOLT_TOLERANCE)){
      #ifdef DEBUG
      Serial.println("Voltage OK");
      #endif
    }
    else{
      #ifdef DEBUG
      Serial.println("Voltage ERROR");
      #endif
    }

    delay(100);
    Serial1.print("CURR ");
    Serial1.println(current_limit_setpoint);
    delay(100);

    //check whether the output voltage=setpoint
    Serial1.println("CURR?");
    while (Serial1.available()==0){}    //wait for reply from PSU
    current_reading = Serial1.parseFloat();
    #ifdef DEBUG
    Serial.print("Output current: ");
    Serial.println(current_reading);
    #endif
    if (current_reading<current_limit_setpoint+PSU_CURRENT_TOLERANCE && current_reading>current_limit_setpoint-PSU_CURRENT_TOLERANCE){
      #ifdef DEBUG
      Serial.println("Current OK");
      #endif
    }
      else{
        #ifdef DEBUG
        Serial.println("Current ERROR");
        #endif
      }  

}
