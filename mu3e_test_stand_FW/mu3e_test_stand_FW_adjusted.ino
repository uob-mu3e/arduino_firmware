
#include <Wire.h>
#include <Adafruit_MAX31865.h> //from MAX31865 library from Adafruit
#include <HIH61xx.h>
#include <AsyncDelay.h>
#include <PID_v1.h>

// CRC calculation as per:
// https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/5_Mass_Flow_Meters/Sensirion_Mass_Flow_Meters_CRC_Calculation_V1.pdf


//HARDWARE:
//Sensirion SFM3300-D flowmeter
//    - I2C interface 
//    - some example code taken from https://github.com/MyElectrons/sfm3300-arduino  
//
// Red, Green and Yellow status LEDs 
//
//Fan is powered directly from a 12v supply
//      - it has a PWM control signal. The duty cycle of this controls the fan speed
//      - it has a tachometer output which outputs pulses which relate to fan speed. Not yet implemented here.
//
//
//MAX31865 temperature sensor is connected via SPI interface. Power from 5v & GND
//CLK - Pin 13
//MISO - Pin 12
//MOSI - Pin 11
//Chip Select - Pin 10

//Honeywell Humidity Sensor
//-also SPI, same pins as above
//Chip Select - Pin 7

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

double aggKp=1230, aggKi=28.6, aggKd=87.4;
double consKp=600, consKi=50, consKd=87;
//Specify the links and initial tuning parameters

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);

#define POLYNOMIAL 0x31     //P(x)=x^8+x^5+x^4+1 = 100110001   //magic number used in the CRC decoding for the flowmeter
#define sfm3300i2c 0x40     //I2C address of flowmeter

int FanPWMPin = 9;          //Fan PWM control signal connected to digital pin 9
const int redPin = 3;       //Red LED connected to Pin 3
const int greenPin = 5;     //Green LED connected to Pin 5
const int yellowPin = 6;    //Yello LED connected to Pin 6
const int humidityCS = 7;   //Chip Select for humidity sensor is connected to pin 7
int PWMValue = 50;          //initialise PWM duty cycle to 30/255. Fan doesn't run when this value is lower than mid-30s so use 30 as a minimum value.
bool human_readable = false;    //flag to set whether to transmit human or machine readable output
bool airflow_stable = false;    //flag to show if airflow control loop is stable
bool broadcast_flag = true;     //flag to enable/disable auto transmission of serial output on every measurement cycle

//---Stuff for MAX31865 temperature sensor
// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0
 
//Humidity Sensor HIH6131-021
HIH61xx<TwoWire> hih(Wire);
AsyncDelay samplingInterval;


void setup() {
  pinMode(FanPWMPin, OUTPUT);         //set up pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(humidityCS, OUTPUT);
  digitalWrite(humidityCS, LOW);    //TEMPORARY - assert this low to disable humidity sensor. - no code written for it yet.
  
  Wire.begin();                       //set up serial port and I2C
  Serial.begin(9600);
  delay(500); // let serial console settle

  // soft reset
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x20);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(1000);

#if 1
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x31);  // read serial number
  Wire.write(0xAE);  // command 0x31AE
  Wire.endTransmission();
  if (6 == Wire.requestFrom(sfm3300i2c,6)) {
    uint32_t sn = 0;
    sn = Wire.read(); sn <<= 8;
    sn += Wire.read(); sn <<= 8;
    Wire.read(); // CRC - omitting for now
    sn += Wire.read(); sn <<= 8;
    sn += Wire.read();
    Wire.read(); // CRC - omitting for now
    Serial.println(sn);
  } else {
    Serial.println("serial number - i2c read error");
  }
#endif

  // start continuous measurement
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x10);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(1000);
  /* // discard the first chunk of data that is always 0xFF
  Wire.requestFrom(sfm3300i2c,3);
  Wire.read();
  Wire.read();
  Wire.read();
  */

  analogWrite(FanPWMPin, PWMValue);   //initialise fan PWM to initial value
  analogWrite(yellowPin, 125);
  analogWrite(redPin, 125);
  //delay(3000);

  //setup for the MAX31865 temperature sensor
  thermo.begin(MAX31865_2WIRE);  // set to 2,3or4WIRE as necessary
  //thermo.begin(MAX31865_3WIRE);
  //thermo.begin(MAX31865_4WIRE);  

  //initialize the variables we're linked to
  Setpoint = 30;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //SetOutputLimits(30, 255);
  myPID.SetOutputLimits(30, 135);
  //compression heating after 145 pwm
}   //-----------------------------------END SETUP-----------------------------------

const unsigned mms = 100; // measurement interval in ms
unsigned long mt_prev = millis(); // last measurement time-stamp
float flow; // current flow value in slm
float vol;  // current volume value in (standard) cubic centimeters
bool flow_sign; // flow sign
bool flow_sp; // previous value of flow sign
bool crc_error; // flag
float flow_values[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //an array to hold historical flow values
int flow_array_index = 0;
float flow_moving_avg = 0;
float temperature = 0; //temperature which is calculated from reading the MAX31865 sensor
float rel_humidity;
float amb_temp;
bool printed = true;
unsigned long ms_prev = millis(); // timer for measurements "soft interrupts"
unsigned long ms_display = millis(); // timer for display "soft interrupts"
int i;
double setpoint_input;


void loop() { //--------------------------------------------MAIN LOOP--------------------------------------------------------------------------------------
  unsigned long ms_curr = millis();
  char command;
//  if (ms_curr - ms_prev >= dms) { // "soft interrupt" every dms milliseconds
//    ms_prev = ms_curr;
//    SFM_measure();
//  }
  if (ms_curr - ms_display >= mms) { // "soft interrupt" every mms milliseconds
    ms_display = ms_curr;
    SFM_measure();                    //read from the flowmeter
    //Serial.print("Temperature = "); 
    //Serial.println(thermo.temperature(RNOMINAL, RREF)); //read from the temperature sensor
    temperature = thermo.temperature(RNOMINAL, RREF);
    if (samplingInterval.isExpired() && !hih.isSampling()) {
      hih.start();
      printed = false;
      samplingInterval.repeat();
      //Serial.println("Sampling started (using Wire library)");
    }
    hih.process();
    hih.read(); // instruct the HIH61xx to take a measurement - blocks until the measurement is ready.
    if (hih.isFinished() && !printed) {
      printed = true;
      // Print saved values
      rel_humidity = hih.getRelHumidity() / 100.0;
      amb_temp = hih.getAmbientTemp() / 100.0;
    }
    Input = (double)temperature;
    double gap = abs(Setpoint-Input); //distance away from setpoint
    if (temperature>amb_temp+1){
      //  if(gap<0.05){
      //    myPID.Compute();
      //    if (temperature >Setpoint){ // If airflow is too low, increase fan power
      //      if(PWMValue < 135){     // prevent PWM value overflowing
      //        PWMValue += 1;
      //      }
      //      else PWMValue = 135;
      //      }
      //
      //    if (temperature < Setpoint){ // If airflow value is too high. decrease fan power
      //      if(PWMValue > 30){      //prevent pwm value underflowing. Fan does not run at all with very low PWM values so use 30/255 as a floor
      //        PWMValue -= 1;
      //      } 
      //      else PWMValue =30;
      //      }
      //    } 
      //  
      if(gap<0.1){
        //we're close to setpoint, use conservative tuning parameters
        myPID.SetTunings(consKp, consKi, consKd);
        myPID.Compute();
        PWMValue = Output;
      }
      else{
        //we're far from setpoint, use aggressive tuning parameters
        myPID.SetTunings(aggKp, aggKi, aggKd);
        myPID.Compute();
        PWMValue = Output;
      }
    }
    analogWrite(FanPWMPin, PWMValue);
    flow_values[flow_array_index]=flow;  //record the current flow value in the array
    flow_array_index++;
    if (flow_array_index == 10){
      flow_array_index = 0;
    }

    //calculate moving windowed average 
    flow_moving_avg = 0;
    for (i=0; i<10; i++){
      flow_moving_avg += flow_values[i];
    }
    flow_moving_avg /= 10;
    if (flow_moving_avg < (Setpoint+1)){
      if (flow_moving_avg > (Setpoint-1)){  //if average of last 10 flow readings is setpoint +/- 1 then   // turn the LED on to show stable airflow
        digitalWrite(redPin, LOW);            //turn off red LED
        analogWrite(greenPin, 125);           // turn the green LED on to show stable airflow
        airflow_stable = true;                //and flag the system as stable
      }
      else {
        digitalWrite(greenPin, LOW);    // If average flow is <(setpoint-1) then turn off green LED 
        analogWrite(redPin, 125);       //and light red LED
        airflow_stable = false;         //and flag system as not stable
      }
    }
    else {
      digitalWrite(greenPin, LOW);    //If average flow is <(setpoint+1) then turn off green LED 
      analogWrite(redPin, 125);       //and light red LED
      airflow_stable = false;         //and flag system as not stable
    }
    if (broadcast_flag){  
      transmit_data(); // output data via serial port
    }

  } // end measurement cycle

 while (Serial.available() > 0) {
  command = Serial.read();

  if(command == '?'){
    Serial.println(F("Commands:"));   //F stores the strings in Flash memory, saves using RAM space.
    Serial.println(F("?: Help"));
    Serial.println(F("v: produce (V)erbose human readable output"));
    Serial.println(F("m: produce compact (M)achine readable output"));
    Serial.println(F("s: new (S)etpoint. followed by integer. e.g. s35 for 35 l/min"));
    Serial.println(F("r: (R)un - begin closed loop control - not implememted yet"));
    Serial.println(F("x: Break - stop closed loop control and turn off fan - not implememted yet"));
    Serial.println(F("d: (D)isplay all measurements"));
    Serial.println(F("f: display (F}low measurement"));
    Serial.println(F("t: display (T)emperature measurement - not implememted yet"));
    Serial.println(F("h: display (H)umidity measurement - not implememted yet"));
    Serial.println(F("b: (B)roadcast measurements"));
    Serial.println(F("n: (N)o broadcasting"));
  }
    
  if (command == 'f'){
    //display_flow_volume(true);
    transmit_data();
   }

  if (command == 's'){
    Serial.print("New setpoint entered:");
    setpoint_input = (double)Serial.parseFloat();
    Serial.println(setpoint_input);
    Setpoint = constrain(setpoint_input, 0, 1000);
    Serial.println(Setpoint);
  }

  if (command == 'v'){
    human_readable = true;
  }

  if (command == 'm'){
    human_readable = false;
  }

  if (command == 'b'){
    broadcast_flag = true;
  }
  
   if (command == 'n'){
    broadcast_flag = false;
  }

  if (command == 'd'){
     transmit_data(); // output data via serial port
  }
  
  command = 0; //reset current command to null
 } // end while serial available
} //-------------------------------------end main loop---------------------------------


//-----Reads from Flowmeter and converts into SFM units
void SFM_measure() {
  if (3 == Wire.requestFrom(sfm3300i2c, 3)) {
    uint8_t crc = 0;
    uint16_t a = Wire.read();
    crc = CRC_prim (a, crc);
    uint8_t  b = Wire.read();
    crc = CRC_prim (b, crc);
    uint8_t  c = Wire.read();
    unsigned long mt = millis(); // measurement time-stamp
    if (crc_error = (crc != c)) // report CRC error
      return;
    a = (a<<8) | b;
    float new_flow = ((float)a - 32768) / 120;
    // an additional functionality for convenience of experimenting with the sensor
    flow_sign = 0 < new_flow;
    if (flow_sp != flow_sign) { // once the flow changed direction
      flow_sp = flow_sign;
    // display_flow_volume();    // display last measurements
      vol = 0;                  // reset the volume
    }
    flow = new_flow;
    unsigned long mt_delta = mt - mt_prev; // time interval of the current measurement
    mt_prev = mt;
    vol += flow/60*mt_delta; // flow measured in slm; volume calculated in (s)cc
    // /60 --> convert to liters per second
    // *1000 --> convert liters to cubic centimeters
    // /1000 --> we count time in milliseconds
    // *mt_delta --> current measurement time delta in milliseconds
  }
  else {
    // report i2c read error
  }
}


//-----Checks CRC of received flowmeter data.

uint8_t CRC_prim (uint8_t x, uint8_t crc) {
  crc ^= x;
  for (uint8_t bit = 8; bit > 0; --bit) {
    if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
    else crc = (crc << 1);
  }
  return crc;
}


//Humidity Sensor - control functions
void powerUpErrorHandler(HIH61xx<TwoWire>& hih){
  Serial.println("Error powering up HIH61xx device");
}


void readErrorHandler(HIH61xx<TwoWire>& hih){
  Serial.println("Error reading from HIH61xx device");
}


void display_flow_volume(bool force_d = false) {
  if (5 < abs(vol) || force_d) { // for convenience let's display only significant volumes (>5ml)
    Serial.print(flow);
    Serial.print("\t");
    Serial.print(vol);
    Serial.print("\t");
    Serial.print(PWMValue);
    Serial.print("\t");
    Serial.print(flow_moving_avg);
    Serial.println(crc_error?" CRC error":"");
  }
}


void transmit_data (void) {
  if (human_readable){
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Flow: ");
    Serial.print(flow);
    Serial.print("\t");
    //Serial.print("Volume: ");
    //Serial.print(vol);
    //Serial.print("\t");
    Serial.print("PWM Value: ");
    Serial.print(PWMValue);
    Serial.print("\t");
    Serial.print("Flow Avg: ");    
    Serial.print(flow_moving_avg);
    Serial.print("\t");
    Serial.print("Setpoint: ");
    Serial.print(Setpoint);
    Serial.print("\t");
    Serial.print("Relative humidity: ");
    Serial.print(rel_humidity);
    Serial.print("\t");
    Serial.print("Ambient temperature: ");
    Serial.print(amb_temp);
    Serial.print("/t");
    if (airflow_stable){
      Serial.print("Stable");
    }
    else{
      Serial.print("Setting...");
    }
    Serial.print("\t");
    Serial.println(crc_error?" CRC error":"");
  }

  else{
    Serial.print("T");
    Serial.print(temperature);
    Serial.print("F");
    Serial.print(flow);
    
    //Serial.print("V");
    //Serial.print(vol);
    //Serial.print("\t");
    Serial.print("P");
    Serial.print(PWMValue);
    
    Serial.print("A");    
    Serial.print(flow_moving_avg);
    
    Serial.print("S");
    Serial.print(Setpoint);

    Serial.print("RH");
    Serial.print(rel_humidity);

    Serial.print("AT");
    Serial.print(amb_temp);
    
    if (airflow_stable){
      Serial.print("K");
    }
    else{
      Serial.print("N");
    }
    Serial.println(crc_error?" CRC error":"");
  }
}
