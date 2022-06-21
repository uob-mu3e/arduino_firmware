int volt_setpoint;  //variable to hold voltage to be requested from power supply
int volt_reading;   //"" actual voltage read from power supply
String command;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

  delay(500); //let serial console settle (not sure if necessary)
}

void loop() {
  // put your main code here, to run repeatedly:
   //Serial.println("INST OUT 1 \r");
   //Serial.println("VOLT 12 \r");
   //Serial.println("OUTP  1 \r");
   //Serial.println("OUTP  0  \r");
  
    //Serial1.write("INST OUT 1\n");
    //Serial.write("INST OUT 1\n");
    //delay(5000);
    //Serial1.write("VOLT 10\n");
    //Serial.write("VOLT 10\n");
    //delay(1000);
    //Serial1.write("OUTP 1\n");
    //Serial.write("OUTP 1\n");
    //delay(1000);
    //Serial1.write("OUTP 0\n");
    //Serial.write("OUTP 0\n");
 
   
   Serial.println("Set power supply voltage: ");
   while (Serial.available()==0){}
   volt_setpoint = Serial.parseInt();
   set_voltage(volt_setpoint);
   
   Serial.println("VOLT?");
   Serial1.println("VOLT?");
    while (Serial1.available()==0){}    //wait for reply from PSU
    volt_reading = Serial1.parseInt();
    Serial.println(volt_reading);
    if (volt_setpoint == volt_reading){
      Serial.println("Voltage OK");
    }
    else{
      Serial.println("Voltage ERROR");
    }
      
  
   
   
   
   
  // float output_volt = Serial1.parseFloat();
   //String output = Serial1.readString();
   //Serial.println(output_volt);
   //Serial.println(output);
   
}



void set_voltage(int v) {
  String cmd="VOLT ";
  String tot;
  String volt=String(v);
  //String endl="\n";
  tot=cmd+volt;
  Serial1.println(tot);
  Serial.println(tot);
  delay(1000);

  //Serial1.println("VOLT?");
  //Serial.println("VOLT?");
  //String output_voltage=Serial1.readString;
  //if (output_voltage==volt)return 0;
  //if (output_voltage!=volt){
    //println("error");
    
  //}
  
}

//void settings {
  //Serial1.write("INST OUT 1\n");  //select channel 1
  //delay(1000);
//}
