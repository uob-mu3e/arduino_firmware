
//Test program to build functions for controlling HMP4040 Power Supply

// USES Arduino MEGA 2560
//HARDWARE:

// PC is connected to UART0 (built in USB connector on Arduino board)

// Rhode&Schwartz HMP4040 power supply is connected to UART 1
//(via MAX232A interface chip to generate RS232 voltage levels)
#define PSU_FAN_CHANNEL     1        //define what is connected to each PSU channel
#define PSU_MUPIX_CHANNEL   2
#define PSU_HEATERS_CHANNEL 3
#define PSU_SPARE_CHANNEL   4

#define PSU_VOLT_TOLERANCE  0.01   // +/- acceptable tolerance band for voltages read back from PSU vs requested value
#define PSU_CURRENT_TOLERANCE  0.01   // "" "" for current

#define PSU_FAN_VOLTAGE       12        //desired fixed supply voltages and current limits
#define PSU_FAN_CURRENT_LIM   2
#define PSU_MUPIX_VOLTAGE     1
#define PSU_MUPIX_CURRENT_LIM 0.5


void PSU_Init(); // Function to initialise power supply
void PSU_Setup(); // Function to set up voltages and current limits on PSU


void setup() {
  
Serial.begin(9600);                 //Serial port for PC
Serial1.begin(9600);                //Serial port for PSU
delay(500); // let serial console settle
Serial.println("Serial Ports Enabled");
PSU_Init();
Serial.println("PSU Initialised");
delay(1000);
  

}

void loop() {
  // put your main code here, to run repeatedly:
 

  PSU_Setup();
  Serial.println("PSU Setup Complete");
  delay(1000000);
  
}




void PSU_Init(){
  // code here to initialise the PSU from a 'cold start'
  // checks the PSU is connected and responding
  // Initialises all channels to 0v and turns channel output off
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





void PSU_Setup(){
    channel_select(PSU_FAN_CHANNEL);
    channel_set(PSU_FAN_VOLTAGE, PSU_FAN_CURRENT_LIM);

    channel_select(PSU_MUPIX_CHANNEL);
    channel_set(PSU_MUPIX_VOLTAGE, PSU_MUPIX_CURRENT_LIM);
   }




//
//void set_voltage(float v) {
//  String cmd="VOLT ";
//  String volt=String(v);
//  String tot=cmd+volt;
//  Serial1.println(tot);
//  delay(1000);
//
////check whether the output voltage=setpoint
//  Serial1.println("VOLT?");
//  while (Serial1.available()==0){}    //wait for reply from PSU
//  volt_reading = Serial1.parseFloat();
//  Serial.print("Output voltage: ");
//  Serial.println(volt_reading);
//  if (volt_reading<v+volt_precision || volt_reading>v-volt_precision){
//    Serial.println("Voltage OK");
//  }
//  else{
//    Serial.println("Voltage ERROR");
//  }
//}



//
//
//void set_current(float a) {
//  String cmd="CURR ";
//  String curr=String(a);
//  String tot=cmd+curr;
//  Serial1.println(tot);
//  delay(1000);
//
////check whether the output voltage=setpoint
//  Serial1.println("CURR?");
//  while (Serial1.available()==0){}    //wait for reply from PSU
//  current_reading = Serial1.parseFloat();
//  Serial.print("Output current: ");
//  Serial.println(current_reading);
//  if (current_reading<a+curr_precision || current_reading>a-curr_precision){
//    Serial.println("Current OK");
//  }
//  else{
//    Serial.println("Current ERROR");
//  }  
//}
//
//
//
//
//
//void output_switch() {
//  Serial.println("Output (on/off): ");
//  while(Serial.available()==0) {}
//  output = Serial.readString();
//  if (output =="on"){
//    Serial1.println("OUTPT 1");
//    delay(100);
//    Serial1.println("OUTP?");
//    while (Serial1.available()==0){}
//    int output_init=Serial1.parseInt();
//    if (output_init == 1) {
//      Serial.println("Output set up correctly.");
//    }
//  }
//  if (output =="off"){
//    Serial1.println("OUTPT 0");
//    Serial1.println("OUTP?");
//    while (Serial1.available()==0){}
//    int output_init=Serial1.parseInt();
//    if (output_init == 0) {
//      Serial.println("Output set up correctly.");
//    }
//  }
//  }
//
//  
//
//
//


//This function selects a channel on the HMP4040 
void channel_select(int n) {
  String cmd="INST OUT";
  String n_str=String(n);
  String tot=cmd+n_str;
  Serial1.println(tot);
  delay(500);
  Serial.print("Selected Channel: ");
  Serial.println(n); 
  }



// This function transmits a voltage setpoint and current limit to the HMP4040
void channel_set (float voltage_setpoint, float current_limit_setpoint){

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
    Serial.print("Output voltage: ");
    Serial.println(volt_reading);
    if (volt_reading<voltage_setpoint+PSU_VOLT_TOLERANCE && volt_reading>voltage_setpoint-PSU_VOLT_TOLERANCE){
      Serial.println("Voltage OK");
    }
    else{
      Serial.println("Voltage ERROR");
    }

    delay(100);
    Serial1.print("CURR ");
    Serial1.println(current_limit_setpoint);
    delay(100);

    //check whether the output voltage=setpoint
    Serial1.println("CURR?");
    while (Serial1.available()==0){}    //wait for reply from PSU
    current_reading = Serial1.parseFloat();
    Serial.print("Output current: ");
    Serial.println(current_reading);
    if (current_reading<current_limit_setpoint+PSU_CURRENT_TOLERANCE && current_reading>current_limit_setpoint-PSU_CURRENT_TOLERANCE){
    Serial.println("Current OK");
  }
  else{
    Serial.println("Current ERROR");
  }  

}








//
//
//
//
//
//
//void channel_initialization(){
//  Serial1.println("*IDN?");
//  while(Serial1.available()==0){}
//  String identity = Serial1.readString();
//  Serial.print("PSU identification: ");
//  Serial.print(identity);
//  //turning the voltage on all channels to 0
//  for (int i=1; i<5; i++) {
//    channel_select(i);
//    Serial1.println("VOLT 0");
//    Serial1.println("OUTP 0");
//    delay(100);
//  //checking if the output is really off  
//    Serial1.println("OUTP?");
//    while (Serial1.available()==0){}
//    int output_init=Serial1.parseInt();
//    if (output_init != 0) {
//      Serial.println("Output error");
//      break;
//    }
//  }
//}
