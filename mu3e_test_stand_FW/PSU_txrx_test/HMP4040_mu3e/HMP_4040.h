
#include "Arduino.h"

//#define DEBUG   //if this is defined, debug data will be output to the serial port
                // comment out to disable

const int PSU_FAN_CHANNEL     = 1;        //define what is connected to each PSU channel
const int PSU_MUPIX_CHANNEL   = 2;
const int PSU_HEATERS_CHANNEL = 3;
const int PSU_SPARE_CHANNEL   = 4;

const float PSU_VOLT_TOLERANCE     = 0.05;   // +/- acceptable tolerance band for voltages read back from PSU vs requested value
const float PSU_CURRENT_TOLERANCE  = 0.05;   // "" "" for current

const float PSU_FAN_VOLTAGE         = 12;        //desired fixed supply voltages and current limits
const float PSU_FAN_CURRENT_LIM     = 2.5;
const float PSU_MUPIX_VOLTAGE       = 1;
const float PSU_MUPIX_CURRENT_LIM   = 0.5;
const float PSU_HEATERS_VOLTAGE_LIM = 12;

const float PSU_HEATERS_R     = 8;      //total resistance of dummy heater resistor chain

void PSU_Init();                  // Function to initialise power supply
void PSU_Setup();                 // Function to set up voltages and current limits on PSU
void PSU_Fan_Power_On();          //Turns ON the power to the fan
void PSU_Fan_Power_Off();         //"" "" OFF
void PSU_Heater_Power_On(double); // Turn on heaters and set to given value in miliWatts
void PSU_Heater_Power_Off();      // Turn off heaters
void Channel_Select(int);         // Selects a given channel on the Power Supply
void Channel_Set (float, float);   // Sets the currently selected channel to (voltage, current)
