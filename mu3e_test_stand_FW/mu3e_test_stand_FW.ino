// vim:set ft=cpp:
// ===[ABSTRACT]===
// CRC calculation as per:
// https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/5_Mass_Flow_Meters/Sensirion_Mass_Flow_Meters_CRC_Calculation_V1.pdf
// HARDWARE:
// - Uses Arduino MEGA 2560
// - PC is connected to UART0 (built in USB connector on Arduino board)
// - Rhode&Schwartz HMP4040 power supply is connected to UART 1
// - (via MAX232A interface chip to generate RS232 voltage levels)
// - Sensirion SFM3300-D flowmeter
//     - I2C interface
//     - some example code taken from
// https://github.com/MyElectrons/sfm3300-arduino
// Line ending: "No line ending"

#include <Adafruit_MAX31865.h>
#include <ArduinoSTL.h>
#include <AsyncDelay.h>
#include <HIH61xx.h>
#include <PID_v1.h>
#include <Wire.h>

#include <string>

// ===[MACRO BEGINS]===
// ### PSU Channels ###
#define FAN_PSU_CHANNEL 1
#define MUPIX_PSU_CHANNEL 2
#define HEATERS_PSU_CHANNEL 3
#define SPARE_PSU_CHANNEL 4

// ### OTHER ###
// - POLYNOMIAL: P(x)=x^8+x^5+x^4+1 = 100110001, magic number used in the CRC
//   decoding for the flowmeter
// - sfm3300i2c: I2C address of flowmeter
// - RREF: The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for
//   PT1000
// - RNOMINAL: The 'nominal' 0-degrees-C resistance of the sensor, 100.0 for
//   PT100, 1000.0 for PT1000
#define POLYNOMIAL 0x31
#define sfm3300i2c 0x40
#define RREF 4300.0
#define RNOMINAL 1000.0
// ===[MACRO ENDS]===

// #######################################

// ===[GLOBAL BEGINS]===
// ### ARDUINO STATUS LEDS ###
const int red_pin = 3;
const int green_pin = 5;
const int yellow_pin = 6;

// ### HARDWARE ###
// -FAN PWM-
// Fan is powered directly from a 12v supply (HMP4040) Controlled via UART1
// - fan has a PWM control signal. The duty cycle of this controls the fan
// speed.
// - it has a tachometer output which outputs pulses which relate to fan speed.
// Not yet implemented here. Fan PWM control signal connected to digital pin 9
const int fan_pwm_pin = 9;

// # MAX31865 TEMPERATURE SENSOR #
// - is connected via SPI interface. Power from 5v
// - & GND CLK - Pin 13 MISO - Pin 12 MOSI - Pin 11 Chip Select - Pin 10
// - Stuff for MAX31865 temperature sensor
// - Use software SPI: CS, DI, DO, CLK
// - Either:
//     - Option 1: Uncomment below to use Software SPI use hardware SPI, just pass in
//       the CS pin Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// - Or:
//     - Option 2: Uncomment below to use Hardware SPI (preferred)
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// # HONEYWELL HUMIDITY SENSOR #
// Also SPI, same CLK & Data pins as above
// Chip Select for humidity sensor is connected to pin 7
const int humidity_cs = 7;

// # HUMIDITY SENSOR HIH6131-021 #
HIH61xx<TwoWire> hih(Wire);
AsyncDelay samplingInterval;

// # PSU #
// *_precision: precision with which compare actual voltage/current to setpoint
// current_limit: upper limit on the current
const float volt_precision = 0.01;
const float curr_precision = 0.01;
const float current_limit = 3;

// ### VARIABLES ###
// initialise these---best left in global scope
// since accesses to them are a bit complicated
float flow_history[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int flow_array_idx = 0;
float rel_humidity = 0;
int pwm_value = 50;
float amb_temp = 0;
float temperature = 0;
float flow = 0;
float flow_moving_avg = 0;
float volume;

// ### FLAGS ###
bool airflow_stable = false;
bool broadcast_flag = true;
bool human_readable = false;
bool crc_error;

// ### PID INITIALISATION ###
// ms_prev: timer for measurements "soft interrupts"
// ms_display: timer for display "soft interrupts"
// mms: measurement interval in ms; how frequently to measure inputs
const unsigned mms = 1000;
double temp_setpoint = 30, input = 0, output = 0;
double agg_kp = 1230, agg_ki = 28.6, agg_kd = 87.4;
double cons_kp = 600, cons_ki = 50, cons_kd = 87;
PID myPID(&input, &output, &temp_setpoint, cons_kp, cons_ki, cons_kd, REVERSE);
unsigned long ms_prev = millis();
unsigned long ms_display = millis();

// ===[GLOBAL ENDS]===

// #######################################

// ===[OTHER FUNCTIONS BEGIN]===
// measure_flow_convert_sfm()
// - Reads from Flowmeter
// - Converts into SFM units
void measure_flow_convert_sfm() {
    bool flow_sign, flow_sign_previous;
    if (3 == Wire.requestFrom(sfm3300i2c, 3)) {
        uint8_t crc = 0;
        uint16_t a = Wire.read();
        crc = crc_prim(a, crc);
        uint8_t b = Wire.read();
        crc = crc_prim(b, crc);
        uint8_t c = Wire.read();

        // measurement time-stamp
        unsigned long mt = millis();

        // report CRC error
        if (crc_error = (crc != c)) return;
        a = (a << 8) | b;
        float new_flow = ((float)a - 32768) / 120;
        flow = new_flow;

        // an additional functionality for convenience of experimenting with the
        // sensor
        flow_sign = 0 < new_flow;

        // once the flow changed direction reset the volume
        if (flow_sign_previous != flow_sign) {
            flow_sign_previous = flow_sign;
            // display_flow_volume();
            volume = 0;
        }

        // time interval of the current measurement
        // mt_prev: last measurement time-stamp
        // mt_delta: difference between mt and mt_prev
        unsigned long mt_prev = millis();
        unsigned long mt_delta = mt - mt_prev;
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
    return;
}
// power_up_error_handler()
// read_error_handler()
// display_flow_volume()
// - Functions to control humidity sensor
// - For convenience display only significant volumes (>5ml)
void power_up_error_handler(HIH61xx<TwoWire>& hih) {
    Serial.println("Error powering up HIH61xx device");
    return;
}

void read_error_handler(HIH61xx<TwoWire>& hih) {
    Serial.println("Error reading from HIH61xx device");
    return;
}

void display_flow_volume(bool force_d = false) {
    if (5 < abs(volume) || force_d) {  
        Serial.print(flow);
        Serial.print("\t");
        Serial.print(volume);
        Serial.print("\t");
        Serial.print(pwm_value);
        Serial.print("\t");
        Serial.print(flow_moving_avg);
        Serial.println(crc_error ? " CRC error" : "");
    }
    return;
}

// loop_pid()
// - PID control loop logic
// - Outputs variables every loop
void loop_pid(unsigned long ms_curr) {
    // not sure about the role of this
    bool printed = true;

    // "soft interrupt" every mms milliseconds
    if (ms_curr - ms_display >= mms) {
        ms_display = ms_curr;
        measure_flow_convert_sfm();
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
            // print saved values
            printed = true;
            rel_humidity = hih.getRelHumidity() / 100.0;
            amb_temp = hih.getAmbientTemp() / 100.0;
        }
        input = (double)temperature;
        double gap = abs(temp_setpoint - input);
        if (temperature > amb_temp + 1) {
            // ### CLOSE: Conservative tuning parameters ###
            if (gap < 0.1) {
                myPID.SetTunings(cons_kp, cons_ki, cons_kd);
                myPID.Compute();
                pwm_value = output;
            }
            // ### FAR: aggressive tuning parameters ###
            else {
                myPID.SetTunings(agg_kp, agg_ki, agg_kd);
                myPID.Compute();
                pwm_value = output;
            }
        }
        analogWrite(fan_pwm_pin, pwm_value);

        flow_history[flow_array_idx] = flow;
        flow_array_idx++;
        flow_array_idx %= 10;

        flow_moving_avg = calculate_flow_avg(flow_history);
        if (broadcast_flag) transmit_data();
    }
    return;
}

// loop_command_input()
// - Reads and performs commands while serial port is available
// - After every command, reset current command to 0
// - The weird effect of this is that there is always character '0' in the buffer
//   so we must set line ending to "No line ending", or else the input of some variables
//   will be intercepted as 0.0 (for Serial.parseFloat())
// - Currently, it is set to '\0'
void loop_command_input() {
    char pc_command;
    while (Serial.available() > 0) {
        pc_command = Serial.read();
        if (pc_command == '?') print_help();
        if (pc_command == 'f') display_flow_volume(true);
        if (pc_command == 's') get_setpoint();
        if (pc_command == 'r') human_readable = toggle_flag(human_readable);
        if (pc_command == 'b') broadcast_flag = toggle_flag(broadcast_flag);
        if (pc_command == 'd') transmit_data();
        if (pc_command == 'p') get_channel();
        if (pc_command == 'o') toggle_selected_channel();
        if ((pc_command == 'v') || (pc_command == 'c')) {
            get_psu_parameter(pc_command);
        }
        pc_command = '\0';  
    }
    return;
}

// toggle_flag()
// - Negates the parameter
bool toggle_flag(bool flag) { return !flag; }

// print_help()
// - Displays list of commands
// - F() stores the strings in Flash memory, saves RAM space.
void print_help() {
    Serial.println(F("Commands:"));
    Serial.println(F("?: Help"));
    Serial.println(F("r: Toggle verbose human (R)eadable output"));
    Serial.println(F("s: Change temperature (S)etpoint."));
    Serial.println(F("d: (D)isplay all measurements"));
    Serial.println(F("f: display (F)low measurement"));
    Serial.println(F("b: Toggle (B)roadcast measurements"));
    Serial.println(F("v{value}: Change PSU (v)oltage"));
    Serial.println(F("c{value}: Change PSU (c)urrent"));
    Serial.println(F("p{value}: Select a (P)SU channel"));
    Serial.println(F("o: Toggle selected PSU channel (O)n/(O)ff"));
    Serial.println(
        F("t: display (T)emperature measurement - not implememted "
          "yet"));
    Serial.println(
        F("h: display (H)umidity measurement - not implememted yet"));
    Serial.println(
        F("x: Break - stop closed loop control and turn off fan - "
          "not "
          "implememted yet"));
    Serial.println(
        F("l: Run - begin closed (l)oop control - not implememted "
          "yet"));
    return;
}

// control_arduino_leds()
// - Switch leds on corresponding to flow_average
// - If average of last 10 flow readings is setpoint +/- 1 then turn the
//   LED on to show stable airflow and flag as stable, otherwise light red
//   LED and flag as NOT stable.
void control_arduino_leds(float flow_val) {
    // ### UNSTABLE conditions ###
    if ((flow_val > (temp_setpoint + 1)) || (flow_val < (temp_setpoint - 1))) {
        digitalWrite(green_pin, LOW);
        analogWrite(red_pin, 125);
        airflow_stable = false;
    }
    // ### STABLE conditions ###
    else {
        digitalWrite(red_pin, LOW);
        analogWrite(green_pin, 125);
        airflow_stable = true;
    }
    return;
}

// get_setpoint()
// - Asks for and set the temperature setpoint
void get_setpoint() {
    int setpoint_input;
    Serial.print("Enter new setpoint: \n");
    setpoint_input = (double)Serial.parseFloat();
    temp_setpoint = constrain(setpoint_input, 0, 1000);
    Serial.println("New setpoint: " + String(temp_setpoint));
    delay(500);
    return;    
}

int calculate_flow_avg(float values[10]) {
    float avg = 0;
    for (int idx = 0; idx < 10; idx++) {
        avg += values[idx];
    }
    avg /= 10;
    return avg;
}

// get_psu_parameter()
// - Called when command "p" is parsed from loop()
// - Reads the parameter to change (voltage/current)
// - Asks for value
// - Sets the value to the corresponding parameter
void get_psu_parameter(char parameter) {
    // ### VOLTAGE ###
    if (parameter == 'v') {
        Serial.println("Enter channel voltage: ");
        float volt_setpoint = Serial.parseFloat();
        set_voltage(volt_setpoint);
        delay(500);
    }
    // ### CURRENT ###
    // initialise to outside the limit
    float current_setpoint = current_limit + 1;
    if (parameter == 'c') {
        Serial.println("Enter channel current: ");
        while (true) {
            current_setpoint = Serial.parseFloat();
            if (current_setpoint <= current_limit) break;
            Serial.print("Current too high. Pick a value below ");
            Serial.print(current_limit);
            Serial.println(" amps: ");
        }
        set_current(current_setpoint);
        delay(500);
    }
    return;
}

// crc_prim()
// - Checks CRC of received flowmeter data.
uint8_t crc_prim(uint8_t x, uint8_t crc) {
    crc ^= x;
    for (uint8_t bit = 8; bit > 0; --bit) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ POLYNOMIAL;
        } else {
            crc = (crc << 1);
        }
    }
    return crc;
}

// transmit_data()
// - Print out data stream in two formats
// - 1. Human readable and 2. Non-human readable
void transmit_data(void) {
    String stable, setting, title, delimiter;

    std::vector<String> stream_keys = {
        "Temp",          "Flow",     "PWM Value",   "Flow Avg",
        "Humidity", "Ambient Temp"};
    std::vector<String> stream_values = {
        String(temperature),     String(flow),          String(pwm_value),
        String(flow_moving_avg), String(temp_setpoint), String(rel_humidity),
        String(amb_temp)};

    for (int idx = 0; idx < stream_keys.size(); ++idx) {
        if (human_readable) {
            title = stream_keys[idx] + ": ";
            stable = "Stable";
            setting = "Setting";
            delimiter = "\t";
        } else {
            title = stream_keys[idx][0];
            stable = "K";
            setting = "N";
            delimiter = "";
        }
        Serial.print(title);
        Serial.print(stream_values[idx]);
        Serial.print(delimiter);
    }
    if (airflow_stable) {
        Serial.print(stable);
    } else {
        Serial.print(setting);
    }
    Serial.println(crc_error ? " CRC error" : "");
    return;
}

// set_voltage()
// - Sets the PSU voltage
// - Checks whether the output voltage=setpoint
void set_voltage(float v_target) {
    String cmd = "VOLT " + String(v_target);
    Serial1.println(cmd);
    delay(1000);
    Serial1.println("VOLT?");
    float volt_reading = Serial1.parseFloat();
    Serial.print("Output voltage: ");
    Serial.println(volt_reading);
    if (volt_reading < v_target + volt_precision ||
        volt_reading > v_target - volt_precision) {
        Serial.println("Voltage OK");
    } else {
        Serial.println("Voltage ERROR");
    }
    Serial.println("New voltage: ");
    Serial.println(String(v_target));
    return;
}

// set_current()
// - Sets PSU current
// - Checks whether the output current=setpoint
void set_current(float c_target) {
    String cmd = "CURR ";
    String curr = String(c_target);
    String tot = cmd + curr;
    Serial1.println(tot);
    delay(1000);
    Serial1.println("CURR?");
    float current_reading = Serial1.parseFloat();
    Serial.print("Output current: ");
    Serial.println(current_reading);
    if (current_reading < c_target + curr_precision ||
        current_reading > c_target - curr_precision) {
        Serial.println("Current OK");
    } else {
        Serial.println("Current ERROR");
    }
    return;
}

// output_switch()
// TODO: this not being used for anything
void output_switch() {
    Serial.println("Output (on/off): ");
    while (Serial.available() == 0) {}
    String psu_output = Serial.readString();
    if (psu_output == "on") {
        Serial1.println("OUTPT 1");
        delay(100);
        Serial1.println("OUTP?");
        int output_init = Serial1.parseInt();
        if (output_init == 1) {
            Serial.println("Output set up correctly.");
        }
    }
    if (psu_output == "off") {
        Serial1.println("OUTPT 0");
        delay(100);
        Serial1.println("OUTP?");
        int output_init = Serial1.parseInt();
        if (output_init == 0) {
            Serial.println("Output set up correctly.");
        }
    }
    return;
}

// get_channel()
// - asks for a PSU channel and selects it
void get_channel() {
    Serial.println("Select PSU channel: ");
    int channel = Serial.parseInt(); 
    select_channel(channel);
    String printout = "Channel " + String(channel) + " selected.";
    Serial.print(printout);
    return;
}

// toggle_selected_channel()
// - Send a query
// - Switch on selected channel if off
// - Switch off selected channel if on
void toggle_selected_channel() {
    String cmd = "OUTP 0";
    String status = "Selected channel switched off.";
    Serial1.println("OUTP?");
    int output_state = Serial1.parseInt();
    delay(100);
    if (output_state == 0) {
        cmd = "OUTP 1";
        String status = "Selected channel switched on.";
    }
    Serial1.println(cmd);
    Serial.print(status);
    delay(500);
    return;
}

// select_channel()
// - Send command to set the PSU channel
void select_channel(int channel) {
    String cmd = "INST OUT" + String(channel);
    Serial1.println(cmd);
    delay(500);
    return;
}

// initialize_channels()
// - Runs at start to initialize all PSU channels
// - Turns the voltage on all channels to 0
void initialize_channels() {
    int idx;
    Serial1.println("*IDN?");
    while (Serial1.available() == 0) {}
    String identity = Serial1.readString();
    Serial.print("PSU identification: ");
    Serial.print(identity);
    for (int idx = 1; idx < 5; idx++) {
        select_channel(idx);
        Serial1.println("VOLT 0");
        Serial1.println("OUTP 0");
        delay(100);
        Serial1.println("OUTP?");
        while (Serial1.available() == 0) {}
        int output_init = Serial1.parseInt();
        if (output_init != 0) {
            Serial.println("Output error");
            break;
        }
    }
    return;
}


// ===[OTHER FUNCTIONS END]===

// #######################################

// ===[SETUP BEGINS]===
// - set up pins
// - disable humidity sensor by setting to LOW: Note: not implemented
// - set up serial port baud rate, serial = PC, serial1 = PSU
// - initialise all power supply channels at 0V
// - select channel 1 by default
void setup() {
    pinMode(fan_pwm_pin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(red_pin, OUTPUT);
    pinMode(green_pin, OUTPUT);
    pinMode(yellow_pin, OUTPUT);
    pinMode(humidity_cs, OUTPUT);

    digitalWrite(humidity_cs, LOW);

    Wire.begin();
    Serial.begin(9600);
    Serial1.begin(9600);
    delay(500);

    initialize_channels();

    // user input select PSU channel
    // Serial.println("Select PSU channel: ");

    // holds value for which channel of the PSU to select
    int channel = 1;
    select_channel(channel);

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

    // initialise fan PWM duty cycle to 50/255. Fan doesn't run when this value
    // is lower than mid-30s initialise other pins
    int pwm_value;
    analogWrite(fan_pwm_pin, pwm_value = 50);
    analogWrite(yellow_pin, 125);
    analogWrite(red_pin, 125);

    // setup for the MAX31865 temperature sensor
    // set to 2, 3 or 4WIRE as necessary
    thermo.begin(MAX31865_2WIRE);  

    // turn the PID & set limits as compression heating after 145 pwm
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(30, 135);

    return;
}
// ===[SETUP ENDS]===

// #######################################

// ===[MAIN LOOP BEGINS]===
void loop() {
    // ms_curr: current time
    unsigned long ms_curr = millis();
    loop_pid(ms_curr);
    loop_command_input();
    return;
}
// ===[MAIN LOOP ENDS]===
