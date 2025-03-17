#include <Wire.h> // library voor LCD
#include <LiquidCrystal_I2C.h> // library voor LCD
#include <HX711_ADC.h> // library voor ADC converter LOADCELL
#include <EEPROM.h> // opslaan van cali val voor LOADCELL

#define stepPin 2 // step pin
#define dirPin 3 // dir pin
#define upButtonPin 8   // Knop voor omhoog
#define downButtonPin 9 // Knop voor omlaag
#define HX711_dout 5 //mcu > HX711 dout pin
#define HX711_sck 6 //mcu > HX711 sck pin

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_calVal_eepromAdress = 0;
unsigned long t = 0;
static boolean newDataReady = 0;
const int serialPrintInterval = 50; // increase value to slow down serial print activity
float NewtonValue = 0;
int stepDelay = 1000;

void setup() {
  Serial.begin(115200);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(upButtonPin, INPUT_PULLUP);  
  pinMode(downButtonPin, INPUT_PULLUP); 

   // initialize the loadcell
  float calibrationValue = 231.72; // calibration value
  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
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
  // Motor control
  if (NewtonValue < 0) NewtonValue = -NewtonValue; // Absolute waarde gebruiken
      
  stepDelay = 100 + (abs(NewtonValue) * 190);  // lineare herschaling naarmate dat de belasting stijgt
    
  if (stepDelay < 100) stepDelay = 100;     // Maximum snelheid
  if (stepDelay > 1000) stepDelay = 1000;   // Minimum snelheid

  Serial.println(stepDelay);

  if (digitalRead(upButtonPin) == LOW) {  
    digitalWrite(dirPin, HIGH);           // omhoog
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay); // de snelheid 
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  } 
  else if (digitalRead(downButtonPin) == LOW) {  
    digitalWrite(dirPin, LOW);                   // omlaag
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay); // de snelheid 
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  
  // Load cell data reading
  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;
  // get smoothed value from the dataset:
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
 /* 
  we kunnen dit doen dan met een knop

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }
  
  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
  */
}


void UpdateLCD(float kracht){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("kracht : ");
  lcd.setCursor(0,1);
  lcd.print(kracht);
}
