#include <Wire.h>
#include <math.h>

class Rail { //Voltage rail class. designed for LM2596 and ACS712

  int enablePin; //connected to EN pin from LM2596
  int voltageSense; //connected to output, via voltage divider if needed
  int currentSense; //connected to OUT pin from ACS712
  float railVoltage; //desired output voltage
  float dividerScaling; //voltage divider for voltage sensing scaling constant

  public: //constructor, set all variables
  Rail(int enable, int voltage, int current, float rail, float scale){
    enablePin = enable;
    voltageSense = voltage;
    currentSense = current;
    railVoltage = rail;
    dividerScaling  = scale; //set object variables
  }

  void initialize() {
    pinMode(enablePin, OUTPUT); //set enable pin as output
    digitalWrite(enablePin, LOW); //start with disabled regulator
  }

  float getCurrent() { //returns current reading from ACS712
    float current = (analogRead(currentSense)* 3.3 / 1023) * 0.1; //set for 20A model. Change the 0.1 for different outputs
    return current;
  }

  float getVoltage() { //returns rail voltage
    float inputVoltage = analogRead(voltageSense) * 3.3 / 1023; //gets input voltage from ADC
    float voltage = inputVoltage / dividerScaling; //applies voltage divider scaling
    return voltage;
  }

  void enable(bool action) {
    digitalWrite(enablePin, action); //turns rail on or off
  }
};

class LTC { //LTC4162 class
  int SMBAlertPin; //pin used as SMBALERT interrupt
  const int PROGMEM address = 0x68; //address for LTC4162.

  float vbat;
  float vin;
  float vout;
  float ibat;
  float iin;
  float die;
  float ntc;
  float bsr; //data from LTC4162. Same name as in datasheet

  public:
  LTC(int interrupt) {
    SMBAlertPin = interrupt;
  }

  void initialize() {
    pinMode(SMBAlertPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(SMBAlertPin),alert , FALLING); //setup interrupt for SMBALERT

    Wire.begin(); //begin I2C communication as bus master
  }

  static void alert() {

  }

  int read16bitVal(int addr) { //returns 16 bit value sent from IC
    Wire.requestFrom(addr,2);

    byte LSB = Wire.read();
    byte MSB = Wire.read();

    int out = ((MSB << 8) | LSB);
    return out;
  }

  float thermistorConversion(int ntcVal){ //converts register value into temperature
    const PROGMEM float Aconst = 0.0;
    const PROGMEM float Bconst = 0.0;
    const PROGMEM float Cconst = 0.0; //constants for Steinhart-Hart equation
     
    float Rntc = (100000 * ntcVal) / (21829 - ntcVal); //Conversion formula from datasheet. Solves for NTC resistance.
    float temp = Aconst  + (Bconst * log(Rntc)) + (Cconst * pow(log(Rntc), 3)); //Steinhart-Hart equation. Solves for temperature.
    return temp;
  }

  float measure() {
    vbat = read16bitVal(0x3A) * 0.0001924; //battery voltage
    vin = read16bitVal(0x3B) * 0.0001649; //input voltage
    vout = read16bitVal(0x3C) * 0.0001653; //output voltage
    ibat = read16bitVal(0x3D) * 0.0001466; //battery current
    iin = read16bitVal(0x3E) * 0.0001466; //input current
    die = read16bitVal(0x3F) * 0.0215 - 264.4; //die temperature
    ntc = thermistorConversion(read16bitVal(0x40)); //NTC temperature
    bsr = read16bitVal(0x41) * 0.000004; //battery series resistance
  }

};

class statusLED { //status LED class
  int RPin;
  int GPin;
  int BPin; //LED pin variables

  public:
  statusLED(int red, int green, int blue) {
    RPin = red;
    GPin = green;
    BPin = blue;
  }
  void start() { //initializes pin modes and turns LED OFF. To be run once at startup
    pinMode(RPin, OUTPUT);
    pinMode(GPin, OUTPUT);
    pinMode(BPin, OUTPUT);

    digitalWrite(RPin, LOW);
    digitalWrite(GPin, LOW);
    digitalWrite(BPin, LOW);
  }

  void errorUpdate(char state) { //updates status LED error being displayed
    if (state == "OFF") { //off status. LED turns off
      digitalWrite(RPin, LOW);
      digitalWrite(GPin, LOW);
      digitalWrite(BPin, LOW);
    }
    else if (state == "OOK") { //OK status. LED turns green
      digitalWrite(RPin, LOW);
      digitalWrite(GPin, HIGH);
      digitalWrite(BPin, LOW);
    }
    else if (state == "STR") { //startup status. LED turns blue
      digitalWrite(RPin, LOW);
      digitalWrite(GPin, LOW);
      digitalWrite(BPin, HIGH);
    }
    else if (state == "WRN") { //warning status. LED turns yellow
      digitalWrite(RPin, HIGH);
      digitalWrite(GPin, HIGH);
      digitalWrite(BPin, LOW);
    }
    else if (state == "ERR") { //error status. LED turns red
      digitalWrite(RPin, HIGH);
      digitalWrite(GPin, LOW);
      digitalWrite(BPin, LOW);
    }
  }
};

Rail R5V0(2, 0, 1, 5.0, 0.33);
Rail R3V3(3, 2, 3, 3.3, 1);
Rail R1V5(4, 4, 5, 1.476, 1); //define all rail objects

LTC BATT(7);

statusLED STAT(7, 8, 9);

void setup() {
  STAT.start();
  STAT.errorUpdate("SRT"); //initialize status LED and set it to start mode

  R5V0.initialize();
  R3V3.initialize();
  R1V5.initialize(); //initialize voltage rails

  BATT.initialize(); //initialize battery management IC
}

void loop() {
  BATT.measure();
}
