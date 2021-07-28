float volt_setpoint;  //variable to hold voltage to be requested from power supply
float volt_reading;   //"" actual voltage read from power supply
float volt_precision=0.01;  //precision with which compare actual voltage to setpoint
float current_setpoint;   //variable to hold current to be requested from power supply
float current_reading;  //actual current read from the power supply
float current_limit=2;   //upper limit on the current
float curr_precision=0.01; //precision with which compare actual current to setpoint
String command;
String parameter;
String output;
int channel;          //select channel of the PSU



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  //delay(500); //let serial console settle (not sure if necessary)
  
 //initializing all channels at 0
  channel_initialization();
  Serial.println("Select PSU channel: ");
  while (Serial.available()==0) {};
  channel = Serial.parseInt();
  channel_select(channel);
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.println(" selected.");
}

 

void loop() {

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
