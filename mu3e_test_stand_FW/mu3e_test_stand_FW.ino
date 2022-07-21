// ===[ABSTRACT]===
// CRC calculation as per:
// -
// https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/5_Mass_Flow_Meters/Sensirion_Mass_Flow_Meters_CRC_Calculation_V1.pdf
// HARDWARE:
// - Uses Arduino MEGA 2560
// - PC is connected to UART0 (built in USB connector on Arduino board)
// - Rhode&Schwartz HMP4040 power supply is connected to UART 1
// - (via MAX232A interface chip to generate RS232 voltage levels)

#include <Adafruit_MAX31865.h>
#include <AsyncDelay.h>
#include <HIH61xx.h>
#include <PID_v1.h>
#include <Wire.h>

// ===[MACRO BEGINS]===
// ---PSU Channel---
#define FAN_PSU_CHANNEL 1
#define MUPIX_PSU_CHANNEL 2
#define HEATERS_PSU_CHANNEL 3
// Sensirion SFM3300-D flowmeter
// - I2C interface
// - some example code taken from
// https://github.com/MyElectrons/sfm3300-arduino
#define SPARE_PSU_CHANNEL 4

// ---OTHER VALUES---
// P(x)=x^8+x^5+x^4+1 = 100110001, magic number used in the CRC
// decoding for the flowmeter
#define POLYNOMIAL 0x31
// I2C address of flowmeter
#define sfm3300i2c 0x40
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 4300.0
// The 'nominal' 0-degrees-C resistance of the sensor, 100.0 for PT100, 1000.0
// for PT1000
#define RNOMINAL 1000.0
// ===[MACRO ENDS]===

// ---------------------------------------

// ===[GLOBAL BEGINS]===
// ---PID INITIALISATION---
double Setpoint = 30, Input = 0, Output = 0;
double aggKp = 1230, aggKi = 28.6, aggKd = 87.4;
double consKp = 600, consKi = 50, consKd = 87;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);

// ---ARDUINO STATUS LED TO PIN CONSTANTS---
const int redPin = 3;
const int greenPin = 5;
const int yellowPin = 6;

// ---HARDWARE VARIABLES---
// -FAN PWM-
// Fan is powered directly from a 12v supply (HMP4040) Controlled via UART1
// - fan has a PWM control signal. The duty cycle of this controls the fan
// speed.
// - it has a tachometer output which outputs pulses which relate to fan speed.
// Not yet implemented here. Fan PWM control signal connected to digital pin 9
int FanPWMPin = 9;

// -MAX31865 TEMPERATURE SENSOR-
// is connected via SPI interface. Power from 5v
// & GND CLK - Pin 13 MISO - Pin 12 MOSI - Pin 11 Chip Select - Pin 10
// Stuff for MAX31865 temperature sensor
// Use software SPI: CS, DI, DO, CLK
// Either...
// Option 1: Uncomment below to use Software SPI use hardware SPI, just pass in
// the CS pin Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// ...or
// Option 2: Uncomment below to use Hardware SPI (preferred)
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// -HONEYWELL HUMIDITY SENSOR-
// Also SPI, same CLK & Data pins as above
// Chip Select for humidity sensor is connected to pin 7
const int humidityCS = 7;

// -HUMIDITY SENSOR HIH6131-021-
HIH61xx<TwoWire> hih(Wire);
AsyncDelay samplingInterval;

// -OTHER-
// measurement interval in ms; how frequently to measure inputs
const unsigned mms = 1000;
// last measurement time-stamp
unsigned long mt_prev = millis();
// timer for measurements "soft interrupts"
unsigned long ms_prev = millis();
// timer for display "soft interrupts"
unsigned long ms_display = millis();
int setpoint_input;
float rel_humidity = 0;
// initialise fan PWM duty cycle to 50/255. Fan doesn't run when this value is
// lower than mid-30s
int PWMValue = 50;

// -FLAGS-
// show if airflow control loop is stable
bool airflow_stable = false;
// auto transmission of serial output every measurement cycle
bool broadcast_flag = true;
// set transmit human or machine readable output
bool human_readable = false;
bool printed = true;

// -FLOWMETER-
// current flow value in slm
float flow;
// current volume value in (standard) cubic centimeters
float volume;
bool flow_sign;
bool flow_sign_previous;
// flag??
bool crc_error;
// an array to hold historical (10 previous) flow values
float flow_values[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int flow_array_index = 0;
float flow_moving_avg = 0;
int flow_setpoint = 30;

// -PSU COMMUNICATION-
// holds voltage to be requested from power supply
float volt_setpoint;
// actual voltage read from power supply
float volt_reading;
// precision with which compare actual voltage to setpoint
float volt_precision = 0.01;
// actual current read from the power supply
float current_reading;
// upper limit on the current
float current_limit = 2;
// holds current to be requested from power supply
float current_setpoint = current_limit + 1;
// precision with which compare actual current to setpoint
float curr_precision = 0.01;
// strings used to send/receive commands to the PSU
String command;
String parameter;
String output;
// Holds value for which channel of the PSU to select
int channel;
float amb_temp = 0;
// temperature which is calculated from reading the MAX31865 sensor
float temperature = 0;
// char to hold command character received from PC
char pc_command;
// ===[GLOBAL ENDS]===

// ---------------------------------------

// ===[SETUP BEGINS]===
void setup() {
    // set up pins
    pinMode(FanPWMPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(yellowPin, OUTPUT);
    pinMode(humidityCS, OUTPUT);

    // TEMPORARY - assert this low to disable humidity sensor. NOTE: no code
    // written for it yet.
    digitalWrite(humidityCS, LOW);

    // set up serial ports and I2C, serial=PC, serial1=PSU
    Wire.begin();
    Serial.begin(9600);
    Serial1.begin(9600);
    // let serial console settle
    delay(500);

    // initializing all power supply channels at 0
    channel_initialization();

    // user input select PSU channel
    Serial.println("Select PSU channel: ");
    while (Serial.available() == 0) {
    };
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
    delay(1000);

#if 1
    Wire.beginTransmission(sfm3300i2c);
    Wire.write(0x31);  // read serial number
    Wire.write(0xAE);  // command 0x31AE
    Wire.endTransmission();
    if (6 == Wire.requestFrom(sfm3300i2c, 6)) {
        uint32_t sn = 0;
        sn = Wire.read();
        sn <<= 8;
        sn += Wire.read();
        sn <<= 8;
        Wire.read();  // CRC - omitting for now
        sn += Wire.read();
        sn <<= 8;
        sn += Wire.read();
        Wire.read();  // CRC - omitting for now
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

    // discard the first chunk of data that is always 0xFF
    /*
    Wire.requestFrom(sfm3300i2c,3);
    Wire.read();
    Wire.read();
    Wire.read();
    */

    // initialise fan PWM to initial value
    analogWrite(FanPWMPin, PWMValue);
    analogWrite(yellowPin, 125);
    analogWrite(redPin, 125);
    // delay(3000);

    // setup for the MAX31865 temperature sensor
    thermo.begin(MAX31865_2WIRE);  // set to 2, 3 or 4WIRE as necessary
    // thermo.begin(MAX31865_3WIRE);
    // thermo.begin(MAX31865_4WIRE);

    // turn the PID on
    myPID.SetMode(AUTOMATIC);
    // compression heating after 145 pwm, must set limits
    myPID.SetOutputLimits(30, 135);
}
// ===[SETUP ENDS]===

// ---------------------------------------

// ===[MAIN LOOP BEGINS]===
void loop() {
    unsigned long ms_curr = millis();

    // "soft interrupt" every mms milliseconds
    if (ms_curr - ms_display >= mms) {
        ms_display = ms_curr;
        // read from the flowmeter TODO: how does it know to write to `flow`????
        SFM_measure();

        temperature = thermo.temperature(RNOMINAL, RREF);
        if (samplingInterval.isExpired() && !hih.isSampling()) {
            hih.start();
            printed = false;
            samplingInterval.repeat();
        }

        // instruct the HIH61xx to take a measurement - blocks until the
        // measurement is ready.
        hih.process();
        hih.read();
        if (hih.isFinished() && !printed) {
            printed = true;
            // Print saved values
            rel_humidity = hih.getRelHumidity() / 100.0;
            amb_temp = hih.getAmbientTemp() / 100.0;
        }
        Input = (double)temperature;

        // gap = distance away from setpoint
        double gap = abs(Setpoint - Input);
        if (temperature > amb_temp + 1) {
            // we're close to setpoint, use conservative tuning parameters
            if (gap < 0.1) {
                myPID.SetTunings(consKp, consKi, consKd);
                myPID.Compute();
                PWMValue = Output;
            }

            // we're far from setpoint, use aggressive tuning parameters
            else {
                myPID.SetTunings(aggKp, aggKi, aggKd);
                myPID.Compute();
                PWMValue = Output;
            }
        }

        analogWrite(FanPWMPin, PWMValue);

        // record the current flow value in the array
        flow_values[flow_array_index] = flow;
        flow_array_index++;
        if (flow_array_index == 10) {
            flow_array_index = 0;
        }

        // calculate moving windowed average
        flow_moving_avg = 0;
        int i;
        for (i = 0; i < 10; i++) {
            flow_moving_avg += flow_values[i];
        }
        flow_moving_avg /= 10;

        // if average of last 10 flow readings is setpoint +/- 1 then turn the
        // LED on to show stable airflow and flag as stable, otherwise light red
        // led and flag as NOT stable.
        //
        // UNSTABLE conditions
        if (flow_moving_avg > (Setpoint + 1))
            || (flow_moving_avg < (Setpoint - 1)) {
                digitalWrite(greenPin, LOW);
                analogWrite(redPin, 125);
                airflow_stable = false;
            }
        // STABLE conditions
        else {
            digitalWrite(redPin, LOW);
            analogWrite(greenPin, 125);
            airflow_stable = true;
        }

        if (broadcast_flag) {
            transmit_data();  // output data via serial port
        }
    }

    // Read and perform commands while serial port is available
    while (Serial.available() > 0) {
        pc_command = Serial.read();

        if (pc_command == '?') {
            // F() stores the strings in Flash memory, saves using RAM space.
            Serial.println(F("Commands:"));
            Serial.println(F("?: Help"));
            Serial.println(F("v: produce (V)erbose human readable output"));
            Serial.println(F("m: produce compact (M)achine readable output"));
            Serial.println(
                F("s: new (S)etpoint. followed by integer. e.g. s35 for 35 "
                  "l/min"));
            Serial.println(F(
                "r: (R)un - begin closed loop control - not implememted yet"));
            Serial.println(
                F("x: Break - stop closed loop control and turn off fan - not "
                  "implememted yet"));
            Serial.println(F("d: (D)isplay all measurements"));
            Serial.println(F("f: display (F}low measurement"));
            Serial.println(F(
                "t: display (T)emperature measurement - not implememted yet"));
            Serial.println(
                F("h: display (H)umidity measurement - not implememted yet"));
            Serial.println(F("b: (B)roadcast measurements"));
            Serial.println(F("n: (N)o broadcasting"));
            Serial.println(F("p: Change (P)SU Parameter"));
        }

        // TODO: f is literally the same as d??
        if (pc_command == 'f') {
            // display_flow_volume(true);
            transmit_data();
        }

        if (pc_command == 's') {
            Serial.print("New setpoint entered:");
            setpoint_input = (double)Serial.parseFloat();
            Setpoint = constrain(setpoint_input, 0, 1000);
            Serial.println(Setpoint);
        }

        if (pc_command == 'v') {
            human_readable = true;
        }

        if (pc_command == 'm') {
            human_readable = false;
        }

        if (pc_command == 'b') {
            broadcast_flag = true;
        }

        if (pc_command == 'n') {
            broadcast_flag = false;
        }

        if (pc_command == 'd') {
            transmit_data();
        }

        if (pc_command == 'p') {
            ChangePSUParameter();
        }

        pc_command = 0;  // reset current command to null
    }
}
// ===[MAIN LOOP ENDS]===

// ---------------------------------------

// ===[OTHER FUNCTIONS]===

// ChangePSUParameter()
// - Called when command "p" is parsed from loop()
// - Reads the parameter to change (voltage/current)
// - Asks for value
// - Sets the value to the corresponding parameter
void ChangePSUParameter() {
    Serial.println("Select parameter to change (voltage/current): ");
    while (Serial.available() > 0) {
        parameter = Serial.readString();
    }

    // -VOLTAGE-
    if (parameter == "voltage") {
        Serial.println("Set channel voltage: ");
        while (Serial.available() == 0) {
            // do nothing to wait for 1 keypress
        }
        volt_setpoint = Serial.parseFloat();
        set_voltage(volt_setpoint);
        delay(500);
    }

    // -CURRENT-
    if (parameter == "current") {
        Serial.println("Set channel current: ");
        // Current value input not okay: loop until it's under the limit
        while (true) {
            while (Serial.available() == 0) {
                // do nothing to wait for 1 keypress
            }
            current_setpoint = Serial.parseFloat();
            if current_setpoint
                <= current_limit{break} Serial.print(
                       "Current too high. Pick a value below ");
            Serial.print(current_limit);
            Serial.println(" amps:");
        }
        set_current(current_setpoint);
        delay(500);
    }
}

// SFM_measure()
// - Reads from Flowmeter
// - Converts into SFM units
void SFM_measure() {
    if (3 == Wire.requestFrom(sfm3300i2c, 3)) {
        uint8_t crc = 0;
        uint16_t a = Wire.read();
        crc = CRC_prim(a, crc);
        uint8_t b = Wire.read();
        crc = CRC_prim(b, crc);
        uint8_t c = Wire.read();
        // measurement time-stamp
        unsigned long mt = millis();
        // report CRC error
        if (crc_error = (crc != c)) return;
        a = (a << 8) | b;
        float new_flow = ((float)a - 32768) / 120;

        // an additional functionality for convenience of experimenting with the
        // sensor
        flow_sign = 0 < new_flow;

        // once the flow changed direction reset the volume
        if (flow_sign_previous != flow_sign) {
            flow_sign_previous = flow_sign;
            // display_flow_volume();
            volume = 0;
        }

        flow = new_flow;

        unsigned long mt_delta =
            mt - mt_prev;  // time interval of the current measurement
        mt_prev = mt;

        // flow measured in slm; volume calculated in (s)cc
        // /60 --> convert to liters per second
        // *1000 --> convert liters to cubic centimeters
        // /1000 --> we count time in milliseconds
        // *mt_delta --> current measurement time delta in milliseconds
        volume += flow / 60 * mt_delta;
    } else {
        // TODO:??
        // report i2c read error
    }
}

// Checks CRC of received flowmeter data.

uint8_t CRC_prim(uint8_t x, uint8_t crc) {
    crc ^= x;
    for (uint8_t bit = 8; bit > 0; --bit) {
        if (crc & 0x80)
            crc = (crc << 1) ^ POLYNOMIAL;
        else
            crc = (crc << 1);
    }
    return crc;
}

// Humidity Sensor - control functions
void powerUpErrorHandler(HIH61xx<TwoWire>& hih) {
    Serial.println("Error powering up HIH61xx device");
}

void readErrorHandler(HIH61xx<TwoWire>& hih) {
    Serial.println("Error reading from HIH61xx device");
}

void display_flow_volume(bool force_d = false) {
    if (5 < abs(volume) || force_d) {  // for convenience let's display only
                                       // significant volumes (>5ml)
        Serial.print(flow);
        Serial.print("\t");
        Serial.print(volume);
        Serial.print("\t");
        Serial.print(PWMValue);
        Serial.print("\t");
        Serial.print(flow_moving_avg);
        Serial.println(crc_error ? " CRC error" : "");
    }
}

void transmit_data(void) {
    if (human_readable) {
        Serial.print("Temp: ");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("Flow: ");
        Serial.print(flow);
        Serial.print("\t");
        // Serial.print("Volume: ");
        // Serial.print(volume);
        // Serial.print("\t");
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
        if (airflow_stable) {
            Serial.print("Stable");
        } else {
            Serial.print("Setting...");
        }
        Serial.print("\t");
        Serial.println(crc_error ? " CRC error" : "");
    }

    else {
        Serial.print("T");
        Serial.print(temperature);
        Serial.print("F");
        Serial.print(flow);

        // Serial.print("V");
        // Serial.print(volume);
        // Serial.print("\t");
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

        if (airflow_stable) {
            Serial.print("K");
        } else {
            Serial.print("N");
        }
        Serial.println(crc_error ? " CRC error" : "");
    }
}

void set_voltage(float v) {
    String cmd = "VOLT ";
    String volt = String(v);
    String tot = cmd + volt;
    Serial1.println(tot);
    delay(1000);

    // check whether the output voltage=setpoint
    Serial1.println("VOLT?");
    while (Serial1.available() == 0) {
    }  // wait for reply from PSU
    volt_reading = Serial1.parseFloat();
    Serial.print("Output voltage: ");
    Serial.println(volt_reading);
    if (volt_reading < v + volt_precision ||
        volt_reading > v - volt_precision) {
        Serial.println("Voltage OK");
    } else {
        Serial.println("Voltage ERROR");
    }
}

void set_current(float a) {
    String cmd = "CURR ";
    String curr = String(a);
    String tot = cmd + curr;
    Serial1.println(tot);
    delay(1000);

    // check whether the output voltage=setpoint
    Serial1.println("CURR?");
    while (Serial1.available() == 0) {
    }  // wait for reply from PSU
    current_reading = Serial1.parseFloat();
    Serial.print("Output current: ");
    Serial.println(current_reading);
    if (current_reading < a + curr_precision ||
        current_reading > a - curr_precision) {
        Serial.println("Current OK");
    } else {
        Serial.println("Current ERROR");
    }
}

void output_switch() {
    Serial.println("Output (on/off): ");
    while (Serial.available() == 0) {
    }
    output = Serial.readString();
    if (output == "on") {
        Serial1.println("OUTPT 1");
        delay(100);
        Serial1.println("OUTP?");
        while (Serial1.available() == 0) {
        }
        int output_init = Serial1.parseInt();
        if (output_init == 1) {
            Serial.println("Output set up correctly.");
        }
    }
    if (output == "off") {
        Serial1.println("OUTPT 0");
        Serial1.println("OUTP?");
        while (Serial1.available() == 0) {
        }
        int output_init = Serial1.parseInt();
        if (output_init == 0) {
            Serial.println("Output set up correctly.");
        }
    }
}

void channel_select(int n) {
    String cmd = "INST OUT";
    String n_str = String(n);
    String tot = cmd + n_str;
    Serial1.println(tot);
    delay(500);
}

void channel_initialization() {
    Serial1.println("*IDN?");
    while (Serial1.available() == 0) {
    }
    String identity = Serial1.readString();
    Serial.print("PSU identification: ");
    Serial.print(identity);
    // turning the voltage on all channels to 0
    for (int i = 1; i < 5; i++) {
        channel_select(i);
        Serial1.println("VOLT 0");
        Serial1.println("OUTP 0");
        delay(100);
        // checking if the output is really off
        Serial1.println("OUTP?");
        while (Serial1.available() == 0) {
        }
        int output_init = Serial1.parseInt();
        if (output_init != 0) {
            Serial.println("Output error");
            break;
        }
    }
}
