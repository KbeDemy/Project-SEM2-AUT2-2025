#include <HX711_ADC.h>  // library voor ADC converter LOADCELL
#include <EEPROM.h>     // opslaan van cali val voor LOADCELL
#include <AccelStepper.h>
 

// DEBUG Toggle
#define DEBUG 1  // 0 = alleen loadcell waarden, 1 = full debugmode
#if DEBUG 
  #define Debug_println(x) Serial.println(x)
  #define Debug_print(x) Serial.print(x)
#else 
  #define Debug_print(x)
  #define Debug_println(x)
#endif

// Pin-definities
const uint8_t stopPin = 2;
const uint8_t HX711_dout = 6;
const uint8_t HX711_sck = 7;
const uint8_t STEP_PIN = 9;      // Step pin
const uint8_t DIR_PIN = 10;       // Direction pin
 
// Microstepping control pins
const uint8_t MS1_PIN = 12;
const uint8_t MS2_PIN = 11;

// HX711 en stepper
HX711_ADC LoadCell(HX711_dout, HX711_sck);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
const float stepsPerRevolution = 200;
int microstepSetting = 2;
// Loadcell waarde
const float newtonMax = 10.0;
float newtonValue = 0;
float wantedValue = 10.0;

unsigned long t = 0;
static boolean newDataReady = 0;
volatile bool Noodstop = false;

void setupPins() {
  pinMode(stopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stopPin), ISR_BREAK, FALLING);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
 
  digitalWrite(MS1_PIN, LOW);  
  digitalWrite(MS2_PIN, LOW);
}

void setupLoadCell() {
  LoadCell.begin();
  float calibrationValue = 231.72;
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    Debug_println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    LoadCell.setCalFactor(calibrationValue);
    Debug_println("Startup is complete");
  }
}

void setup() {
  Serial.begin(57600);
  setupPins();
  setupLoadCell();

  
         
}

void loop() {
  while (!Noodstop) {
    for (int i = 1; i < 6; i++) {
      wantedValue = i * 5;
      Serial.print("---- nieuwe grafiek begint hier met als wantedValue: ");
      Serial.print(wantedValue);
      Serial.println(" ----");
      Serial.println("wantedValue , newtonValue");

      for (int j = 1; j < 11; j++) {
        readLoadCell();
        float integralMax = 50.0;

        if (newtonValue < wantedValue) {
          while (newtonValue < wantedValue && !Noodstop) {
            readLoadCell(); 
            Serial.print(wantedValue);
            Serial.print (" , ");
            Serial.println(newtonValue);
                      
            float desiredRPM = 250; // Set the desired speed in rpm (revolutions per minute)
            float MaxRPM = 300; // Set max speed in rpm (revolutions per minute)
          
            float speedStepsPerSec = (microstepSetting * stepsPerRevolution*desiredRPM) / 60.0;
            float Max_Speed_StepsPerSec = microstepSetting * stepsPerRevolution * MaxRPM / 60; // Specify max speed in steps/sec (converted from RPM)
            stepper.setMaxSpeed(Max_Speed_StepsPerSec); // negatief is naar beneden!!!
            stepper.setSpeed(-speedStepsPerSec);

            stepper.runSpeed();
          }
        }
        while (newtonValue >= wantedValue && !Noodstop) {
            readLoadCell(); 
            Serial.print(wantedValue);
            Serial.print (" , ");
            Serial.println(newtonValue);
           
          float desiredRPM = 5000; // Set the desired speed in rpm (revolutions per minute)
          float MaxRPM = 6000; // Set max speed in rpm (revolutions per minute)
        
          float speedStepsPerSec = (microstepSetting * stepsPerRevolution*desiredRPM) / 60.0;
          float Max_Speed_StepsPerSec = microstepSetting * stepsPerRevolution * MaxRPM / 60; // Specify max speed in steps/sec (converted from RPM)
          stepper.setMaxSpeed(Max_Speed_StepsPerSec);
          stepper.setSpeed(speedStepsPerSec);

          stepper.runSpeed();
        }
      }
    }
    delay(1000);
  }
}

void readLoadCell() {
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t) {
      float i = LoadCell.getData();
      newtonValue = (i / 1000 * -9.81)  ;
      newDataReady = 0;
      t = millis();
    }
  }
}

void ISR_BREAK() {
  Noodstop = true;
}
