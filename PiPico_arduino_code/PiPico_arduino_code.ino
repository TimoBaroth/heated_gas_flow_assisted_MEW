#include <ArduinoJson.h>
#include <Adafruit_MAX31865.h>
#include <spi.h>
#include <Adafruit_SHT4x.h>
#include <PID_v1.h>

// PT100 temperature measurement - MAX31865
Adafruit_MAX31865 pt100 = Adafruit_MAX31865(13, &SPI1);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000 // TAKEN from example code 
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

// Ambient Temp and Humidity measurement    // TAKEN from example code 
Adafruit_SHT4x sht45 = Adafruit_SHT4x();
int ambient_WindowSize = 1000; // lets take a measurement approx every 1 second 
unsigned long ambient_windowStartTime;


// variables to hold input/output data
JsonDocument doc;
double N2_temp_set = 0;
double N2_temp_act = 0;
double P_gas_heater = 0;
float ambient_temp_act = 0;
float ambient_humidity_act = 0;
double PID_P = 1; // PID proportional term
double PID_I = 1; // PID integral term
double PID_D = 0; // PID derivative term
double PID_output = 0;
bool N2_heater_run = false;
bool decon_ambient_sensor = false;
//pin heater control 
const int heater = 18; 

// PID 
PID h_pid(&N2_temp_act, &PID_output, &N2_temp_set, PID_P, PID_I, PID_D, DIRECT);
int WindowSize = 500; 
unsigned long windowStartTime;

//output timing
int out_WindowSize = 100; // maybe every 100 ms
unsigned long out_windowStartTime;


void setup() {
  //start serial communication
  Serial.begin(9600);
  delay(500); //give it some time to init serial

  // SPI bus pin set
  SPI1.setRX(12);
  SPI1.setCS(13);
  SPI1.setSCK(10);
  SPI1.setTX(11); 
  //start PT100 readout module
  pt100.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
  //start ambient sensor module 
  sht45.begin();
  // set heater output pins
  pinMode(heater, OUTPUT);
  digitalWrite(heater, LOW);
  //pid setup
  windowStartTime = millis();
  h_pid.SetOutputLimits(0, WindowSize);
  h_pid.SetMode(AUTOMATIC);
  // timer setup ambient
  ambient_windowStartTime = windowStartTime; 
  out_windowStartTime = windowStartTime;
  //give it some time to finish everything
  delay(500);
}


void loop() {
  // adapted from: https://arduinojson.org/v6/how-to/do-serial-communication-between-two-boards/
  // get input data 
  if (Serial.available()){
    StaticJsonDocument<300> doc;

    DeserializationError err = deserializeJson(doc, Serial);

    if (err == DeserializationError::Ok){
      N2_temp_set = doc["N2_temp_set"];
      N2_heater_run = doc["N2_heater_run"];
      decon_ambient_sensor = doc["decon_ambient_sensor"];
      if (decon_ambient_sensor == true){  // Only temp control when ambient sensor not in decon => adafruit library blocks and would slow down code execution, no good for temp control
        N2_heater_run = false;
      }

    }
  }

  // read pt 100 temp
  N2_temp_act = pt100.temperature(RNOMINAL, RREF);
  // Check if any faults
  uint8_t fault = pt100.readFault();
  if (fault) {
    N2_heater_run = false; // deactivate N2_heater_run state
    digitalWrite(heater, LOW); // make sure heate is turned off
  }

  // check if we want to activate heater / temp control 
  if (N2_heater_run == true){ 
    bool pid_state =  h_pid.Compute();  // compute pid
    if (!pid_state) { // if it returns false, set it to automatic mode to enable compute
      h_pid.SetMode(AUTOMATIC);
    }
    // quasi software PWM, as we want slow switching 
    /************************************************ see https://github.com/br3ttb/Arduino-PID-Library/tree/master/examples/PID_RelayOutput
    * turn the output pin on/off based on pid output
    ************************************************/
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if (PID_output < millis() - windowStartTime){
      digitalWrite(heater, LOW);
    } else {
      digitalWrite(heater, HIGH);
    }

  } else { // if we don't want to activate heater, set ot manual (so we can set back to auto with propre init)
      h_pid.SetMode(MANUAL);
      digitalWrite(heater, LOW); // make sure heate is turned off
      PID_output = 0;
    }

  // ambient temp/humidity sensor, every one second
  if (millis() - ambient_windowStartTime > ambient_WindowSize) { //time to shift the Relay Window
    ambient_windowStartTime += ambient_WindowSize;
    if (decon_ambient_sensor == true){ 
      sht45.setHeater(SHT4X_HIGH_HEATER_1S); 
    } else {
      sht45.setHeater(SHT4X_NO_HEATER); 
    }
    sensors_event_t humidity, temp;
    sht45.getEvent(&humidity, &temp);
    ambient_temp_act = temp.temperature;
    ambient_humidity_act = humidity.relative_humidity;
    JsonDocument outdoc;
    outdoc["ambient_humidity_act"] = ambient_humidity_act;
    outdoc["ambient_temp_act"] = ambient_temp_act;
    serializeJson(outdoc, Serial);
    Serial.println(); // empty line, makes it easier to sort data on pyhton side
  }



  //send info back
  if (millis() - out_windowStartTime > out_WindowSize){
      out_windowStartTime += out_WindowSize;
      JsonDocument outdoc;
      outdoc["N2_temp_act"] = N2_temp_act; // pt100 temp reading
      outdoc["N2_heater_power"] = PID_output/WindowSize; // heater output scaled 0..1
      outdoc["N2_heater_run"] = N2_heater_run; // N2_heater_run state
      outdoc["decon_ambient_sensor"] = decon_ambient_sensor;
      serializeJson(outdoc, Serial);
      Serial.println(); // empty line, makes it easier to sort data on pyhton side
  }

}