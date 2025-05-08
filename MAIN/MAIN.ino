#include <Wire.h> // library voor LCD
#include <LiquidCrystal_I2C.h> // library voor LCD
#include <HX711_ADC.h> // library voor ADC converter LOADCELL
#include <AccelStepper.h>

// DEBUG Toggle
#define DEBUG 0  // 0 = off (alleen loadcell waarden), 1 = on (full debugmode)
#if DEBUG 
  #define Debug_println(x) Serial.println(x)
  #define Debug_print(x) Serial.print(x)
#else 
  #define Debug_print(x)
  #define Debug_println(x)
#endif

// pin def
const uint8_t stopPin     =  2; 
const uint8_t startPin    =  4; 
const uint8_t confirmPin  =  5;

const uint8_t HX711_dout  =  6;
const uint8_t HX711_sck   =  7;

const uint8_t STEP_PIN    =  9;
const uint8_t DIR_PIN     = 10;
const uint8_t MS1_PIN     = 12;
const uint8_t MS2_PIN     = 11;

const uint8_t pumpPin     =  3; // (NC)
const uint8_t valvePin    =  8; 

const uint8_t joystickPin = A1; // Y 

// global objects
LiquidCrystal_I2C lcd(0x27,16,2);
HX711_ADC LoadCell(HX711_dout, HX711_sck);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// global var
// stepper related
const float stepsPerRevolution = 200; 
const uint8_t microstepSetting =   1;

// Weight related
const float maxWeight = 10000; 
float weightValue     =     0;

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

void setupLCD(){ 
  // initialize the lcd 
  lcd.init();                    
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("PMD4U");
  lcd.setCursor(0,1);
  lcd.print("Opstarten...");
  delay(1000);
  // show the PMD4U label
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("PMD4U");
  lcd.setCursor(0,1);
  lcd.print("Ready!");
  delay(500);
}

void setup() {
  Serial.begin(57600);
  setupLCD();
  setupPins();
  setupLoadCell();
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

void moveManual(){
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
  bool isPumpRunning = false;
  unsigned long pumpStartTime = 0;
  bool isValveOpen = false;


  float wantedValue = askWeightValue();
  unsigned long pumpTime = askPumpTime() ;
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

    if (millis() - pumpStartTime >= pumpTime) {
      Debug_println("Timer verlopen! POMP UIT!");
      digitalWrite(pumpPin, LOW);
      isPumpRunning = false;
      digitalWrite(valvePin, HIGH); // dit werkt niet ik weet niet welke pin we hier voor moeten gaan gebruiken ?  
      isValveOpen = true;
      delay(500);
      digitalWrite(valvePin,LOW);
      isValveOpen = false;
      // motor omhoog 
      Debug_println("Klep geopend");
      autoMoveUp();
      Debug_println("moveUp");
      // ga uit deze lus 
      Debug_println("Test compleet.");
      Debug_println("automatishe mode gestopt");
      automaticMode = false;
    }
  }
}

void readLoadCell() {
  if (LoadCell.update()) {
    weightValue = -LoadCell.getData();
  }
}

void printValue(float value){
  static unsigned long printTimeLCD = 0;
  const uint8_t LCDPrintTime = 255;
  if(millis() - printTimeLCD >= LCDPrintTime)
  {
    lcd.setCursor(0, 0);
    lcd.print("Kracht: ");
    lcd.setCursor(0, 1);
    lcd.print("             "); // niet getest , geen clear gebruiken maar spaties om flikkering te voorkomen ? 
    lcd.setCursor(0, 1);
    lcd.print(abs(value), 1); // abs omdat - niet kan met lcd (of werkt niet goed toen ik dit deed (42454541)
    lcd.setCursor(0,14);
    lcd.print(" gr");   
    printTimeLCD = millis();
  }
}

float askValue(String label, float startValue, float stepSize, float minValue, float maxValue, String unit) {
  float delta = 0.0;
  float value = startValue;
  unsigned long lastUpdateTime = 0;
  unsigned long updateInterval = 100;

  static unsigned long printTimeLCD = 0;
  const uint8_t LCDPrintTime = 255;

  while (true) {
    int joyValue = analogRead(joystickPin);
    delta = 0.0;

    // Joystick uitlezen en aanpassen
    if (millis() - lastUpdateTime >= updateInterval) {
      if (joyValue < 490) {                                      
        delta = (joyValue - 490) / 490.0 * stepSize; 
      } else if (joyValue > 530) {
        delta = (joyValue - 530) / (1023.0 - 530.0) * stepSize;
      }

      value += delta;
      if (value < minValue) value = minValue;
      if (value > maxValue) value = maxValue;

      lastUpdateTime = millis();
    }

    // LCD update
    if (millis() - printTimeLCD >= LCDPrintTime) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(label + ": ");
      lcd.setCursor(label.length() + 2, 0);
      lcd.print(value, (stepSize < 1.0) ? 1 : 0); // Toon 1 decimaal als het nodig is
      lcd.print(" " + unit);
      lcd.setCursor(0, 1);
      lcd.print("Confirm ->");
      printTimeLCD = millis();
    }

    // Bevestiging
    if (digitalRead(confirmPin) == LOW) {
      while (digitalRead(confirmPin) == LOW) delay(10); // debounce
      return value;
    }
  }
}

float askWeightValue() {
  return askValue("Max-g", 100.0, 0.1, 0.0, maxWeight, "gr");
}

unsigned long askPumpTime() {
  return (unsigned long)(askValue("Tijd", 1.0, 1.0, 0.0, 120.0, "s") * 1000); // seconden → ms 
}

void autoMoveDown() {
  unsigned long printTimeLCD = 0;   
  const uint8_t LCDPrintTime = 255;             // refresh-interval in ms (max waarde)
  unsigned long startTime = millis();
  unsigned long timeout = 300000;               // Max 5 min proberen
  float wantedValue = askWeightValue();
 

  if (millis() - printTimeLCD >= LCDPrintTime) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("running ... ");
    printTimeLCD = millis();
  }

  while (millis() - startTime < timeout) {
    readLoadCell();
    if (weightValue <= wantedValue) {
      float error = wantedValue - weightValue;
      float Kp = 100.0;                         // proportionele versterkingsfactor 
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
}

void autoMoveUp(){
  float terugRPM = 600; // gewenste terug-snelheid
  float speedStepsPerSec = (microstepSetting * stepsPerRevolution * terugRPM) / 60.0;
  float maxSpeed = microstepSetting * stepsPerRevolution * 2000 / 60.0;
  stepper.setMaxSpeed(maxSpeed);
  stepper.setSpeed(speedStepsPerSec); // omhoog is positieve snelheid
    
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {  // 2 seconden omhoog
    stepper.runSpeed();
  }
}
