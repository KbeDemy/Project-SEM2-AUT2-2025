/*
  extra elementen die nodig zijn 
     - een TARE knop 
     - de klep moet nog kunnen werken 
     - ERROR handeling van bepaalde noodsituaties (bv.) 
        - als het gewicht wegvalt dan zou er een error moeten zijn 
        - ...
     - een DEBUG ON/OFF switch 
     - een TARE KNOP
     -...
*/

#include <Wire.h> // library voor LCD
#include <LiquidCrystal_I2C.h> // library voor LCD
#include <HX711_ADC.h> // library voor ADC converter LOADCELL
#include <EEPROM.h> // opslaan van cali val voor LOADCELL

#include <AccelStepper.h>

// DEBUG Toggle
#define DEBUG 0  // 0 = alleen loadcell waarden, 1 = on full debugmode
#if DEBUG 
  #define Debug_println(x) Serial.println(x)
  #define Debug_print(x) Serial.print(x)
#else 
  #define Debug_print(x)
  #define Debug_println(x)
#endif

// pin def
const uint8_t stopPin = 2; 
const uint8_t startPin = 4; 
const uint8_t confirmPin = 5;

const uint8_t HX711_dout = 6;
const uint8_t HX711_sck = 7;
const uint8_t STEP_PIN = 9;
const uint8_t DIR_PIN = 10;
const uint8_t MS1_PIN = 12;
const uint8_t MS2_PIN = 11;
const uint8_t joystickPin = A1; // Y 
const uint8_t pumpPin = 3; // (NC)
const uint8_t valvePin = 8; 


// global objects
LiquidCrystal_I2C lcd(0x27,16,2);
HX711_ADC LoadCell(HX711_dout, HX711_sck);
#define STEPS 200
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
const float stepsPerRevolution = 200;
int microstepSetting = 1;
// global var
// Weight related
const float maxWeight = 10000; 
float weightValue = 0;
float wantedValue = 100.0;

// pump related
unsigned long pumpStartTime = 0;
unsigned long pumpTime = 1000; // 15 sec
const unsigned long pumpTimeMax = 120000; // 2 minuten in ms
bool isPumpRunning = false;
// valve related
bool isValveOpen = false;
// loadcell related
unsigned long t = 0; 
static boolean newDataReady = 0;
// LCD related
unsigned long printTimeLCD = 0;
const int LCDPrintTime = 300;

//status var
volatile bool automaticMode = false;

void setupPins() {
  pinMode(joystickPin, INPUT);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(confirmPin, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(13,OUTPUT);
  
  digitalWrite(pumpPin, LOW);
  digitalWrite(valvePin, LOW);
  
  attachInterrupt(digitalPinToInterrupt(stopPin), ISR_BREAK, FALLING);
}

void setup() {
  Serial.begin(57600);
  printTimeLCD = millis();
  init_LCD();
  setupPins();
  setupLoadCell();
  // show the PMD4U label
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("PMD4U");
  lcd.setCursor(0,1);
  lcd.print("Ready!");
  delay(500);
}

void setupLoadCell(){
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

void init_LCD(){ 
  lcd.init();                    // initialize the lcd 
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("PMD4U");
  lcd.setCursor(0,1);
  lcd.print("Opstarten...");
  delay(1000);
}

void loop() {
  Serial.println(weightValue);
  if (digitalRead(startPin) == LOW && !automaticMode){
    automaticMode = true;
    Debug_println("automatic mode");
    automatic();
  }
  if(!automaticMode){
    Debug_println("manual mode");
    moveManual();
    digitalWrite(pumpPin, LOW); 
    digitalWrite(valvePin, LOW);
  }
}

void ISR_BREAK(){
  automaticMode = false;
}

void moveManual()
{
  int joystickValue = analogRead(joystickPin);
  const int deadzone = 50; // Maak een dode zone rond 512
  int maxSpeed = 3000;
  int speed = map(abs(joystickValue - 512), 0 , 512 , 300, maxSpeed);
  stepper.setMaxSpeed(maxSpeed);

  readLoadCell(); 
  printValue(weightValue);

  if(weightValue <= maxWeight){
    if (joystickValue > 512 + deadzone) {
      stepper.setSpeed(speed); // positieve snelheid = omhoog
      stepper.runSpeed();
    }
    else if (joystickValue < 512 - deadzone) {
      stepper.setSpeed(-speed); // negatieve snelheid = omlaag
      stepper.runSpeed();
    }
    else {
      stepper.setSpeed(0); // stop in de deadzone
      stepper.runSpeed();
    }
  } else {
    Debug_print("Maximum value reached: ");
    Debug_println(maxWeight);

    if (joystickValue > 512 + deadzone) {
      stepper.setSpeed(speed); // positieve snelheid = omhoog
      stepper.runSpeed();
    } else {
      Debug_println("Motor geblokkeerd (te veel gewicht)");
    }
  }
}

void automatic(){

  askWeightValue();
  askPumpTime();

  autoMoveDown();
  
  if (weightValue >= wantedValue) {
    if (!isPumpRunning) {
      isPumpRunning = true;
      pumpStartTime = millis();
      digitalWrite(pumpPin, HIGH); 
      Debug_println("pomp aan");
    }
  }
 while (isPumpRunning && !isValveOpen) {

    Debug_print("Verstreken tijd: ");
    Debug_println(millis() - pumpStartTime);
    Debug_print("Ingestelde tijd: ");
    Debug_println(pumpTime);

  /*  dit werkt niet ik weet niet welke pin we hier voor moeten gaan gebruiken ?  */

    if (millis() - pumpStartTime >= pumpTime) {
     
      Debug_println("Timer verlopen! POMP UIT!");
      digitalWrite(pumpPin, LOW);
      isPumpRunning = false;
      digitalWrite(valvePin, HIGH);
      isValveOpen = true;
      Debug_println("Klep geopend");

      Debug_println("motor omhoog");
        
      float terugRPM = 600; // gewenste terug-snelheid
      float speedStepsPerSec = (microstepSetting * stepsPerRevolution * terugRPM) / 60.0;
      float maxSpeed = microstepSetting * stepsPerRevolution * 2000 / 60.0;
      stepper.setMaxSpeed(maxSpeed);
      stepper.setSpeed(speedStepsPerSec); // omhoog is positieve snelheid

      unsigned long startTime = millis();
      while (millis() - startTime < 2000) {  // 2 seconden omhoog
        stepper.runSpeed();
      }

      
      Debug_println("Test compleet.");
      Debug_println("automatishe mode gestopt");
      automaticMode = false;

      digitalWrite(valvePin,LOW);
      isValveOpen = false;
    }
  }
}

void readLoadCell() {
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t) {
      float i = LoadCell.getData();
      weightValue = -i;
      newDataReady = 0;
      t = millis();
    }
  }
}

void printValue(float value){
  if(millis() - printTimeLCD >= LCDPrintTime)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Kracht: ");
    lcd.setCursor(8, 0);
    lcd.print(abs(value), 1); // abs omdat - niet kan met lcd (of werkt niet goed toen ik dit deed (42454541)
    lcd.print(" gr");   
    printTimeLCD = millis();
  }
}

void askWeightValue() {
  float delta = 0.0;
  float stapgrootte = 0.1;

  unsigned long lastUpdateTime = 0;
  unsigned long updateInterval = 100; // joystick aanpassingstijd

  while (true) {
    int joyValue = analogRead(joystickPin);
    delta = 0,0;
    // Serial.print("joyvalue : ");
    // Serial.println(joyValue);

    // waarden aanpassen met joyValue
    if (millis() - lastUpdateTime >= updateInterval) {
      if (joyValue < 490) {
        delta = (joyValue - 490) / 490.0;
        delta *= stapgrootte;
      } else if (joyValue > 530) {
        delta = (joyValue - 530) / (1023.0 - 530.0);
        delta *= stapgrootte;
      }
      // waarden limiteren
      wantedValue += delta;
      if (wantedValue < 0.0) wantedValue = 0.0;
      if (wantedValue > maxWeight) wantedValue = maxWeight;

      lastUpdateTime = millis();
    }

    // LCD update 
    if (millis() - printTimeLCD >= LCDPrintTime) {
      lcd.clear();

      lcd.setCursor(0, 0);
      lcd.print("Max-g: ");

      lcd.setCursor(7, 0);
      lcd.print(wantedValue, 1);

      
      lcd.print(" gr");

      lcd.setCursor(0, 1);
      lcd.print("Confirm ->");
      printTimeLCD = millis();
    }
    //confirm
    if (digitalRead(confirmPin) == LOW) {
      while (digitalRead(confirmPin) == LOW) {
        delay(10); 
      }
      
      break; // ga uit lus = confirmed
    }
  }
}

void askPumpTime() {
  float delta = 0.0;
  float stapgrootte = 1.0; // 1 seconde per stap

  unsigned long lastUpdateTime = 0;
  unsigned long updateInterval = 100; // joystick aanpassingstijd

  while (true) {
    int joyValue = analogRead(joystickPin);
    delta = 0.0;

    // joystick input verwerken
    if (millis() - lastUpdateTime >= updateInterval) {
      if (joyValue < 490) {
        delta = (joyValue - 490) / 490.0;
        delta *= stapgrootte * 1000.0; // seconden omzetten naar milliseconden
      } else if (joyValue > 530) {
        delta = (joyValue - 530) / (1023.0 - 530.0);
        delta *= stapgrootte * 1000.0;
      }

      pumpTime += (long)delta;

      // beperken tussen 0 en PumpTimeMAx minuten
      if (pumpTime < 0) pumpTime = 0;
      if (pumpTime > pumpTimeMax) pumpTime = pumpTimeMax;

      lastUpdateTime = millis();
    }

    // LCD updaten
    if (millis() - printTimeLCD >= LCDPrintTime) {
      lcd.clear();

      lcd.setCursor(0, 0);
      lcd.print("Tijd: ");

      lcd.setCursor(6, 0);
      lcd.print(pumpTime / 1000); // seconden weergeven

      lcd.setCursor(10, 0);
      lcd.print("s");

      lcd.setCursor(0, 1);
      lcd.print("Start ->");

      printTimeLCD = millis();
    }

    // bevestiging met knop
    if (digitalRead(confirmPin) == LOW) {
      while (digitalRead(confirmPin) == LOW) {
        delay(10); 
      }
      Debug_print("break Pump");
      break; // bevestiging -> uit de lus
    }
  }
}

void autoMoveDown() {
  // Lcd message 
  if (millis() - printTimeLCD >= LCDPrintTime) {
      lcd.clear();

      lcd.setCursor(0, 0);
      lcd.print("fqsjf ");

      printTimeLCD = millis();
    }
  unsigned long startTime = millis();
  unsigned long timeout = 300000;  // Max 5 min proberen

  while (millis() - startTime < timeout) {
    readLoadCell();
    if (weightValue <= wantedValue) {
      float error = wantedValue - weightValue;
      float Kp = 100.0;
      float output = Kp * error;
      float desiredRPM = constrain(output, 50, 500);

      float speedStepsPerSec = (microstepSetting * stepsPerRevolution * desiredRPM) / 60.0;
      float maxSpeed = microstepSetting * stepsPerRevolution * 2000 / 60.0;

      stepper.setMaxSpeed(maxSpeed);
      stepper.setSpeed(-speedStepsPerSec); // omlaag
      stepper.runSpeed();
      Serial.println(weightValue);
    }else {
      stepper.setMaxSpeed(0);
      stepper.setSpeed(0);
      stepper.runSpeed();
      break;
    }
  } 

  Debug_print("behaalde waarde is :");
  Debug_println(weightValue);
  delay(5000);
}
