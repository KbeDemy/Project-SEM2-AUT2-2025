#include <Wire.h> // library voor LCD
#include <LiquidCrystal_I2C.h> // library voor LCD
#include <HX711_ADC.h> // library voor ADC converter LOADCELL
#include <EEPROM.h> // opslaan van cali val voor LOADCELL



// pin def
const uint8_t stepPin  = 9; 
const uint8_t dirPin = 10; 

const uint8_t joystickPin = A1; // Y 
  
const uint8_t startPin = 4; 
const uint8_t stopPin = 2; 

const uint8_t HX711_dout = 6;
const uint8_t HX711_sck = 7;

const uint8_t pumpPin = 11; // (NC)
const uint8_t valvePin = 12; 



// global objects
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// global var
const int serialPrintInterval = 50; // increase value to slow down serial print activity
const float newtonMax = 10.0; // Max force

unsigned long t = 0;
static boolean newDataReady = 0;

float newtonValue = 0;
unsigned long pumpStartTime = 0;
unsigned long pumpTime = 15000; // 1 min in millisec


//status var
volatile bool automaticMode = false;

void setupPins() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(joystickPin, INPUT);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(13,OUTPUT);
  // Start met pomp en klep uit
  digitalWrite(pumpPin, HIGH); // NC 
  digitalWrite(valvePin, LOW);
  
  attachInterrupt(digitalPinToInterrupt(stopPin), ISR_BREAK, FALLING);
}

void setupLoadCell(){
    float calibrationValue = 231.72; // measert with 1kg (.5 acc)
  LoadCell.begin();
  LoadCell.start(1000, true); // time, do tare 
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("ERROR");
  }
  else {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
}


void setup() {
  Serial.begin(115200);
  
  setupPins();
  setupLoadCell();

  lcd.init();      // initialize the LCD                
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Starting up ...");
}

void loop() {
  Serial.print(automaticMode);
  if (digitalRead(startPin) == LOW && !automaticMode){
    automaticMode = true;
    Serial.println("automatic mode has started");
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("auto");
  }

  if(!automaticMode){
     // Serial.println("manual mode");
    manual();
    digitalWrite(pumpPin, HIGH); // Zet pomp UIT
    digitalWrite(valvePin, LOW); // Zet klep dicht
  }
}

void ISR_BREAK(){
  automaticMode = false;
}


void manual(){
  int joystickValue = analogRead(joystickPin); 
  int deadZone = 50; 

  readLoadCell();
  Serial.println(newtonValue);
  if (newtonValue < newtonMax){
    if (joystickValue > (512 + deadZone)) {  
      int stepDelay = map(joystickValue, 512 + deadZone, 1023, 1000, 100);  // verder-> sneller
      Serial.print("UP: "); 
      Serial.println(stepDelay);
      moveMotor(HIGH, stepDelay);
    } 
    else if (joystickValue < (512 - deadZone)) {
      int stepDelay = map(joystickValue, 0, 512 - deadZone, 1000, 100); //
      
      Serial.print("   DOWN: "); 
      Serial.println(stepDelay);
      moveMotor(LOW, stepDelay);
    } 
    else {
      Serial.println("move joystick");

    }
  } 
  else {
    Serial.print("Maximum value reached : ");
    Serial.println(newtonMax);

    
  }
}

void moveMotor(bool direction, int stepDelay){
  digitalWrite(dirPin, direction);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelay);
}

void readLoadCell(){
  if (LoadCell.update()) newDataReady = true;  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float GramValue = LoadCell.getData();
      newtonValue = GramValue / 1000 * -9.81; // waarde in gram / 1000 * 9.81 () 
      newDataReady = 0;
      t = millis();
    }
  }
}



