/*
  extra elementen die nodig zijn 
     - een TARE knop 
     - alle communicatie die nu serieel gebeurt zal moeten gebeuren via een lcd scherm
     - de wanted value moet kunnen worden ingesteld
     - de klep moet nog kunnen werken
     - de tijd dat de pomp aanligt moet nog instelbaar zijn 
     - ERROR handeling van bepaalde noodsituaties (bv.) 
        - als het gewicht wegvalt dan zou er een error moeten zijn 
        - ...
     - ...
*/

#include <Wire.h> // library voor LCD
#include <LiquidCrystal_I2C.h> // library voor LCD
#include <HX711_ADC.h> // library voor ADC converter LOADCELL
#include <EEPROM.h> // opslaan van cali val voor LOADCELL
#include <Stepper.h> // 

// pin def
const uint8_t stopPin = 2; 
const uint8_t startPin = 4; 
const uint8_t HX711_dout = 6;
const uint8_t HX711_sck = 7;
const uint8_t SP1 =  9; 
const uint8_t SP2 = 10;
const uint8_t SP3 = 11;
const uint8_t SP4 = 12;
const uint8_t joystickPin = A1; // Y 
const uint8_t pumpPin = 3; // (NC)
const uint8_t valvePin = 99;  

// global objects
LiquidCrystal_I2C lcd(0x27,16,2);
HX711_ADC LoadCell(HX711_dout, HX711_sck);
#define STEPS 200
Stepper stepper(STEPS, SP1, SP2, SP3, SP4);

// global var
const int serialPrintInterval = 50; 
const float newtonMax = 10.0; 

float newtonValue = 0;
float wantedValue = 10.0;

// pump related
unsigned long pumpStartTime = 0;
unsigned long pumpTime = 15000; // 15 sec
bool isPumpRunning = false;
// valve related
bool isValveOpen = false;
// loadcell related
unsigned long t = 0; 
static boolean newDataReady = 0;

//status var
volatile bool automaticMode = false;

void setupPins() {
  pinMode(joystickPin, INPUT);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(13,OUTPUT);
  
  digitalWrite(pumpPin, LOW);
  digitalWrite(valvePin, LOW);
  
  attachInterrupt(digitalPinToInterrupt(stopPin), ISR_BREAK, FALLING);
}

void setup() {
  Serial.begin(57600);
  setupPins();
  setupLoadCell();
}

void setupLoadCell(){
  LoadCell.begin();
  float calibrationValue = 231.72; 
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Startup is complete");
  }
}

void loop() {
  if (digitalRead(startPin) == LOW && !automaticMode){
    automaticMode = true;
    Serial.println("automatic mode has started");
    automatic();
  }
  if(!automaticMode){
    manual();
    digitalWrite(pumpPin, LOW); 
    digitalWrite(valvePin, LOW);
  }
}

void ISR_BREAK(){
  automaticMode = false;
}
void manual(){

  int joystickValue = analogRead(joystickPin); 
  int deadZone = 50; 
  int speed = map(abs(joystickValue - 512), 0, 512, 50, 200); 

  readLoadCell();

  Serial.println(newtonValue);
  if (newtonValue < newtonMax){
    if (joystickValue < 512 - deadZone) {
      Serial.println("Motor gaat omlaag");
      stepper.setSpeed(speed);
      stepper.step(50); // Beweeg de motor omhoog
    }
    else if (joystickValue > 512 + deadZone) {
      Serial.println("Motor omhoog");
      stepper.setSpeed(speed);
      stepper.step(-50); 
    }

    else {
        Serial.println("move joystick");
    }
  }
  else {
    Serial.print("Maximum value reached : ");
    Serial.println(newtonMax); 
    // als max dan mag je hem omhoog laten gaan
    if (joystickValue > 512 + deadZone) {
      Serial.println("Motor omhoog");
      stepper.setSpeed(speed);
      stepper.step(-50); // Beweeg de motor omhoog
    }
  }
}
void automatic(){
  if (newtonValue < wantedValue){
    while(newtonValue <= wantedValue/10 && automaticMode) {
      readLoadCell();
      stepper.setSpeed(100);
      stepper.step(50);
      Serial.println(newtonValue);
    }
    while((newtonValue > 1 && newtonValue < wantedValue) && automaticMode){
      readLoadCell();
      stepper.setSpeed(60);
      stepper.step(50);
      Serial.println(newtonValue);
    }
  }
  if (newtonValue >= wantedValue) {
    if (!isPumpRunning) {
      isPumpRunning = true;
      pumpStartTime = millis();
      digitalWrite(pumpPin, HIGH); 
      Serial.println("pomp aan");
    }
  }
 while (isPumpRunning && !isValveOpen) {

    Serial.print("Verstreken tijd: ");
    Serial.println(millis() - pumpStartTime);
    Serial.print("Ingestelde tijd: ");
    Serial.println(pumpTime);

/*  dit werkt niet ik weet niet welke pin we hier voor moeten gaan gebruiken ?  */

    if (millis() - pumpStartTime >= pumpTime) {
     
      Serial.println("Timer verlopen! POMP UIT!");
      digitalWrite(pumpPin, LOW);
      isPumpRunning = false;
      digitalWrite(valvePin, HIGH);
      isValveOpen = true;
      Serial.println("Klep geopend");
    }
  }

/* 
hier zou de motor automatish(misshien op een OKE) naar boven gaan 
   na dit is de analyse gedaan 

*/

}
void readLoadCell() {
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t) {
      float i = LoadCell.getData();
      newtonValue = i / 1000 * -9.81;
      newDataReady = 0;
      t = millis();
    }
  }
}
