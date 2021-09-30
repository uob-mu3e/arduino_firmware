
#include <Wire.h>
#include <Adafruit_MAX31865.h> //from MAX31865 library from Adafruit

// USES Arduino MEGA 2560

// CRC calculation as per:
// https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/5_Mass_Flow_Meters/Sensirion_Mass_Flow_Meters_CRC_Calculation_V1.pdf


//HARDWARE:

// PC is connected to UART0 (built in USB connector on Arduino board)

// Rhode&Schwartz HMP4040 power supply is connected to UART 1
//(via MAX232A interface chip to generate RS232 voltage levels)
#define FAN_PSU_CHANNEL  1
#define MUPIX_PSU_CHANNEL 2
#define HEATERS_PSU_CHANNEL 3
#define SPARE_PSU_CHANNEL 4


//Sensirion SFM3300-D flowmeter
//    - I2C interface 
//    - some example code taken from https://github.com/MyElectrons/sfm3300-arduino  
#define POLYNOMIAL 0x31     //P(x)=x^8+x^5+x^4+1 = 100110001   //magic number used in the CRC decoding for the flowmeter
#define sfm3300i2c 0x40     //I2C address of flowmeter


// Red, Green and Yellow status LEDs 
const int redPin = 3;       //Red LED connected to Pin 3
const int greenPin = 5;     //Green LED connected to Pin 5
const int yellowPin = 6;    //Yello LED connected to Pin 6

//Fan is powered directly from a 12v supply (HMP4040) Controlled via UART1
//      - fan has a PWM control signal. The duty cycle of this controls the fan speed
//      - it has a tachometer output which outputs pulses which relate to fan speed. Not yet implemented here.
int FanPWMPin = 9;          //Fan PWM control signal connected to digital pin 9


//MAX31865 temperature sensor is connected via SPI interface. Power from 5v & GND
//CLK - Pin 13
//MISO - Pin 12
//MOSI - Pin 11
//Chip Select - Pin 10
//---Stuff for MAX31865 temperature sensor
// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);   //Uncomment this to use Software SPI
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);                 //Uncomment this to use Hardware SPI (preferred)
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0


//Honeywell Humidity Sensor
//-also SPI, same CLK & Data pins as above
const int humidityCS = 7;   //Chip Select for humidity sensor is connected to pin 7



 




//Global Variables

const unsigned mms = 1000; // measurement interval in ms - 'system heartbeat'-how frequently to measure inputs, calculate action to take then implement new output from control loop 
unsigned long mt_prev = millis(); // last measurement time-stamp
unsigned long ms_prev = millis(); // timer for measurements "soft interrupts"
unsigned long ms_display = millis(); // timer for display "soft interrupts"
int i;
int setpoint_input;

bool airflow_stable = false;    //flag to show if airflow control loop is stable
bool broadcast_flag = true;     //flag to enable/disable auto transmission of serial output on every measurement cycle
bool human_readable = false;    //flag to set whether to transmit human or machine readable output

int FlowSetpoint = 30;      //desired setpoint for airflow
int PWMValue = 50;          //initialise fan PWM duty cycle to 50/255. Fan doesn't run when this value is lower than mid-30s so use 30 as a minimum value.

//Variables for flowmeter
float flow; // current flow value in slm
float vol;  // current volume value in (standard) cubic centimeters
bool flow_sign; // flow sign
bool flow_sp; // previous value of flow sign
bool crc_error; // flag
float flow_values[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //an array to hold historical flow values
int flow_array_index = 0;
float flow_moving_avg = 0;

//Variables for comms with PSU
float volt_setpoint;        // holds voltage to be requested from power supply
float volt_reading;         //"" actual voltage read from power supply
float volt_precision=0.01;  //precision with which compare actual voltage to setpoint
float current_setpoint;     //holds current to be requested from power supply
float current_reading;      //actual current read from the power supply
float current_limit=2;      //upper limit on the current
float curr_precision=0.01;  //precision with which compare actual current to setpoint
String command;             //strings used to send/receive commants to the PSU
String parameter;
String output;
int channel;                //Holds value for which channel of the PSU to select

float temperature = 0; //temperature which is calculated from reading the MAX31865 sensor



//---------------------------------------------------------------SETUP-------------------------------------------------------------
void setup() {
  pinMode(FanPWMPin, OUTPUT);         //set up pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(humidityCS, OUTPUT);
  digitalWrite(humidityCS, LOW);    //TEMPORARY - assert this low to disable humidity sensor. - no code written for it yet.
  
  Wire.begin();                       //set up serial ports and I2C
  Serial.begin(9600);                 //Serial port for PC
  Serial1.begin(9600);                //Serial port for PSU
  delay(500); // let serial console settle

   //initializing all power supply channels at 0
  channel_initialization();
  Serial.println("Select PSU channel: ");
  while (Serial.available()==0) {};
  channel = Serial.parseInt();
  channel_select(channel);
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.println(" selected.");


  // soft reset
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x20);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

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

  delay(100);
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
  
}   //-----------------------------------END SETUP-----------------------------------






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
    




    if (flow < FlowSetpoint){ // If airflow is too low, increase fan power
      if(PWMValue < 255){     // prevent PWM value overflowing
        PWMValue += 1;
      }
      else PWMValue = 255;
      }

    if (flow > FlowSetpoint){ // If airflow value is too high. decrease fan power
      if(PWMValue > 30){      //prevent pwm value underflowing. Fan does not run at all with very low PWM values so use 30/255 as a floor
        PWMValue -= 1;
      } 
      else PWMValue =30;
      }

    if(FlowSetpoint == 0){
      PWMValue = 30;
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

   if (flow_moving_avg < (FlowSetpoint+1)){
    if (flow_moving_avg > (FlowSetpoint-1)){  //if average of last 10 flow readings is setpoint +/- 1 then   // turn the LED on to show stable airflow
        digitalWrite(redPin, LOW);            //turn off red LED
        analogWrite(greenPin, 125);           // turn the green LED on to show stable airflow
        airflow_stable = true;                //and flag the system as stable
        
    }else {
      digitalWrite(greenPin, LOW);    // If average flow is <(setpoint-1) then turn off green LED 
      analogWrite(redPin, 125);       //and light red LED
      airflow_stable = false;         //and flag system as not stable
    }
   }else {
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
    Serial.println(F("p: Change (P)SU Parameter"));
    }
    
  if (command == 'f'){
    //display_flow_volume(true);
    transmit_data();
   }

  if (command == 's'){
    Serial.print("New setpoint entered:");
    setpoint_input = Serial.parseInt();
    Serial.println(setpoint_input);
    FlowSetpoint = constrain(setpoint_input, 0, 200);
    Serial.println(FlowSetpoint);
    
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

 if (command == 'p'){
     ChangePSUParameter();
  }

 
  
  command = 0; //reset current command to null
  
 } // end while serial available

  
} //-------------------------------------end main loop---------------------------------


//change a PSU parameter
void ChangePSUParameter(){
  
Serial.println("Select parameter to change (voltage/current): ");
while (Serial.available()==0) {}
parameter = Serial.readString();



//ask for input voltage
if (parameter =="voltage"){
   Serial.println("Set channel voltage: ");
   while (Serial.available()==0){}
   volt_setpoint = Serial.parseFloat();
   set_voltage(volt_setpoint);
   delay(500);
}


if (parameter =="current") {
   Serial.println("Set channel current: ");
   while (Serial.available()==0){}
   current_setpoint = Serial.parseFloat();
    while(current_setpoint>current_limit) {
      Serial.print("Current too high. Pick a value below ");
      Serial.print(current_limit);
      Serial.println(" amps:");
      while (Serial.available()==0){}
      current_setpoint = Serial.parseFloat();
    }
   set_current(current_setpoint);
   delay(500);  
}
}

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
  } else {
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
    Serial.print(FlowSetpoint);
    Serial.print("\t");
    if (airflow_stable){
      Serial.print("Stable");
    }else{Serial.print("Setting...");}
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
    Serial.print(FlowSetpoint);
    
    if (airflow_stable){
      Serial.print("K");
    }else{Serial.print("N");}
    
    
    Serial.println(crc_error?" CRC error":"");
   }
}





void set_voltage(float v) {
  String cmd="VOLT ";
  String volt=String(v);
  String tot=cmd+volt;
  Serial1.println(tot);
  delay(1000);

//check whether the output voltage=setpoint
  Serial1.println("VOLT?");
  while (Serial1.available()==0){}    //wait for reply from PSU
  volt_reading = Serial1.parseFloat();
  Serial.print("Output voltage: ");
  Serial.println(volt_reading);
  if (volt_reading<v+volt_precision || volt_reading>v-volt_precision){
    Serial.println("Voltage OK");
  }
  else{
    Serial.println("Voltage ERROR");
  }
}





void set_current(float a) {
  String cmd="CURR ";
  String curr=String(a);
  String tot=cmd+curr;
  Serial1.println(tot);
  delay(1000);

//check whether the output voltage=setpoint
  Serial1.println("CURR?");
  while (Serial1.available()==0){}    //wait for reply from PSU
  current_reading = Serial1.parseFloat();
  Serial.print("Output current: ");
  Serial.println(current_reading);
  if (current_reading<a+curr_precision || current_reading>a-curr_precision){
    Serial.println("Current OK");
  }
  else{
    Serial.println("Current ERROR");
  }  
}





void output_switch() {
  Serial.println("Output (on/off): ");
  while(Serial.available()==0) {}
  output = Serial.readString();
  if (output =="on"){
    Serial1.println("OUTPT 1");
    delay(100);
    Serial1.println("OUTP?");
    while (Serial1.available()==0){}
    int output_init=Serial1.parseInt();
    if (output_init == 1) {
      Serial.println("Output set up correctly.");
    }
  }
  if (output =="off"){
    Serial1.println("OUTPT 0");
    Serial1.println("OUTP?");
    while (Serial1.available()==0){}
    int output_init=Serial1.parseInt();
    if (output_init == 0) {
      Serial.println("Output set up correctly.");
    }
  }
  }

  



void channel_select(int n) {
  String cmd="INST OUT";
  String n_str=String(n);
  String tot=cmd+n_str;
  Serial1.println(tot);
  delay(500);
}






void channel_initialization(){
  Serial1.println("*IDN?");
  while(Serial1.available()==0){}
  String identity = Serial1.readString();
  Serial.print("PSU identification: ");
  Serial.print(identity);
  //turning the voltage on all channels to 0
  for (int i=1; i<5; i++) {
    channel_select(i);
    Serial1.println("VOLT 0");
    Serial1.println("OUTP 0");
    delay(100);
  //checking if the output is really off  
    Serial1.println("OUTP?");
    while (Serial1.available()==0){}
    int output_init=Serial1.parseInt();
    if (output_init != 0) {
      Serial.println("Output error");
      break;
    }
  }
}
