#include <HX711_ADC.h>  // library voor ADC converter LOADCELL
#include <EEPROM.h>     // opslaan van cali val voor LOADCELL
#include <Stepper.h>    // library voor aansturen stappenmotor

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
const uint8_t SP1 = 9;
const uint8_t SP2 = 10;
const uint8_t SP3 = 11;
const uint8_t SP4 = 12;

// HX711 en stepper
HX711_ADC LoadCell(HX711_dout, HX711_sck);
#define STEPS 200
Stepper stepper(STEPS, SP1, SP2, SP3, SP4);

// PID-parameters
float Kp = 0.02;
float Ki = 0.005;
float Kd = 1;

float integral = 0;
float previousError = 0;
unsigned long previousTime = millis();

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
      Serial.println("deltaTime , Error , integral , Derivative , output , wantedValue , newtonValue");

      for (int j = 1; j < 11; j++) {
        readLoadCell();
        float integralMax = 50.0;

        if (newtonValue < wantedValue) {
          integral = 0;
          previousError = 0;
          previousTime = millis();

          while (newtonValue < wantedValue && !Noodstop) {
            readLoadCell();

            unsigned long currentTime = millis();
            float deltaTime = (currentTime - previousTime) / 1000.0;
            previousTime = currentTime;

            float error = wantedValue - newtonValue;
            float derivative = (error - previousError) / deltaTime;
            previousError = error;

           
            if (abs(error) < 10) {
              integral += error * deltaTime;
              integral = constrain(integral, -integralMax, integralMax);
            }

            float output = Kp * error + Ki * integral + Kd * derivative;

            if (abs(output) < 1) output = (output > 0 ? 1 : -1);

            int speed = constrain(abs(output), 40, 100);
            int stepSize = constrain(map(abs(error), 0, wantedValue, 1, 100), 1, 100);

            stepper.setSpeed(speed);
            stepper.step(stepSize * (output > 0 ? 1 : -1));

            
            Debug_print("t=");
            Debug_print(currentTime / 1000.0);
            Debug_print("s, Error=");
            Debug_print(error);
            Debug_print(", Int=");
            Debug_print(integral);
            Debug_print(", Deriv=");
            Debug_print(derivative);
            Debug_print(", Out=");
            Debug_print(output);
            Debug_print(", F=");
            Debug_println(newtonValue);

            if (abs(error) < 1) break;
          }
        }

        // Reset na test: trek kracht terug
        while (newtonValue >= wantedValue && !Noodstop) {
          readLoadCell();
          stepper.setSpeed(100);
          stepper.step(-20);
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
      newtonValue = i / 1000 * -9.81;
      newDataReady = 0;
      t = millis();
    }
  }
}


void ISR_BREAK() {
  Noodstop = true;
}

