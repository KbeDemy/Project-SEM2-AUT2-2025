#include <Wire.h> // library voor LCD
#include <LiquidCrystal_I2C.h> // library voor LCD
#include <HX711_ADC.h> // library voor ADC converter LOADCELL
#include <EEPROM.h> // opslaan van cali val voor LOADCELL

#define stepPin 2 // step pin
#define dirPin 3 // dir pin
#define downButtonPin 8   // Knop voor omhoog
#define upButtonPin 9 // Knop voor omlaag
#define HX711_dout 5 //mcu > HX711 dout pin
#define HX711_sck 6 //mcu > HX711 sck pin
#define startButtonPin 7 // Start knop voor automatische modus
#define stopButtonPin 12 // Stop knop voor automatische modus
#define pumpPin 11 // Pin voor de pomp (NC geschakeld)
#define valvePin 10 // Pin voor de klep

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_calVal_eepromAdress = 0;
unsigned long t = 0;
static boolean newDataReady = 0;
const int serialPrintInterval = 50; // increase value to slow down serial print activity
float NewtonValue = 0;
int stepDelay = 1000;

// Nieuwe variabelen voor automatische modus
const float newtonMax = 10.0; // Maximale kracht in Newton
bool isAutomaticMode = false;
bool isPumpRunning = false;
bool isValveOpen = false;
unsigned long pumpStartTime = 0;
const unsigned long PUMP_DURATION = 15000; // 1 minuut in milliseconds

void setup() {
  Serial.begin(115200);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(upButtonPin, INPUT_PULLUP);  
  pinMode(downButtonPin, INPUT_PULLUP);
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(13,OUTPUT);
  // Start met pomp en klep uit
  digitalWrite(pumpPin, HIGH); // NC geschakeld, dus HIGH is uit
  digitalWrite(valvePin, LOW);

  // initialize the loadcell
  float calibrationValue = 231.72; // calibration value
  LoadCell.begin();
  LoadCell.start(1000, true); // functie vraag een "stabilizatie tijd en en "do tare" 
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("ERROR");
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  // initialize the lcd
  lcd.init();                      
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Starting up ...");
}

void loop() {
  // Check voor start/stop knoppen
  if (digitalRead(startButtonPin) == LOW && !isAutomaticMode) {
    isAutomaticMode = true;
    Serial.println("Automatische modus gestart");
  }
  
  if (digitalRead(stopButtonPin) == LOW && isAutomaticMode) // interrupt maken die programma direct stopt
  { 
    isAutomaticMode = false;
    isPumpRunning = false;
    isValveOpen = false;
    digitalWrite(pumpPin, HIGH); // Zet pomp uit
    digitalWrite(valvePin, LOW); // Zet klep dicht
    Serial.println("Automatische modus gestopt");
  }

  if (isAutomaticMode) {
    automatish();
  } else {
    // Bestaande handmatige besturing
    if (NewtonValue < 0) NewtonValue = -NewtonValue;
      
    stepDelay = 100 + (abs(NewtonValue) * 190);  // lineare herschaling naarmate dat de belasting stijgt
    
    if (stepDelay < 100) stepDelay = 100;     // Maximum snelheid
    if (stepDelay > 1000) stepDelay = 1000;   // Minimum snelheid

    //Serial.println(stepDelay);

    if (digitalRead(downButtonPin) == LOW) { 
      Serial.println("downbutton"); 
      digitalWrite(dirPin, HIGH);           // omhoog
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay); // de snelheid 
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay);
    } 
    else if (digitalRead(upButtonPin) == LOW) { 
      Serial.println("upbutton"); 
      digitalWrite(dirPin, LOW);                   // omlaag
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay); // de snelheid 
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay);
    }
  }
  
  // Load cell data reading
  if (LoadCell.update()) newDataReady = true;  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float GramValue = LoadCell.getData();
      NewtonValue = GramValue / 1000 * -9.81; // waarde in gram / 1000 * 9.81 () 
      /*
      Serial.print("Load_cell output val in gram: ");
      Serial.print(GramValue);
      Serial.print("      Load_cell output in N: ");
      Serial.println(NewtonValue);
      */
      UpdateLCD(NewtonValue);
      newDataReady = 0;
      t = millis();
    }
  }
}

void automatish() {
  // Motor naar beneden laten gaan
  //digitalWrite(dirPin, HIGH);
  //digitalWrite(stepPin, HIGH);
  //delayMicroseconds(stepDelay);
  //digitalWrite(stepPin, LOW);
  //delayMicroseconds(stepDelay);
  if(NewtonValue <= newtonMax)
  digitalWrite(13,HIGH);
  // Check of maximale kracht is bereikt
  if (NewtonValue >= newtonMax && !isPumpRunning) {
    // Stop de motor
    //digitalWrite(stepPin, LOW);
      digitalWrite(13,LOW); // error moet optreden indien gewicht wegvalt tijdens process.
    // Start de pomp
    isPumpRunning = true;
    digitalWrite(pumpPin, LOW); // NC geschakeld, dus LOW is aan
    pumpStartTime = millis();
    Serial.println("Maximale kracht bereikt, pomp gestart");
  }

  // Check of pomp lang genoeg heeft gedraaid
  if (isPumpRunning && !isValveOpen) {
    if (millis() - pumpStartTime >= PUMP_DURATION) {
      // Zet pomp uit en open klep
      digitalWrite(pumpPin, HIGH); // Zet pomp uit
      isPumpRunning = false;
      digitalWrite(valvePin, HIGH); // Open klep
      isValveOpen = true;
      Serial.println("Pomp gestopt, klep geopend");
    }
  }

  // Debug informatie
  if (LoadCell.update()) {
    if (millis() > t + serialPrintInterval) {
      Serial.print("Kracht: ");
      Serial.print(NewtonValue);
      Serial.print(" N | Pomp: ");
      Serial.print(isPumpRunning ? "AAN" : "UIT");
      Serial.print(" | Klep: ");
      Serial.println(isValveOpen ? "OPEN" : "DICHT");
      t = millis();
    }
  }
}

void UpdateLCD(float kracht){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("kracht : ");
  lcd.setCursor(0,1);
  lcd.print(kracht);
}
