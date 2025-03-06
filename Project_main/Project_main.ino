#include <Wire.h> // library voor LCD
#include <LiquidCrystal_I2C.h> // library voor LCD
#include <HX711_ADC.h> // library voor ADC converter LOADCELL
#include <EEPROM.h> // opslaan van cali val voor LOADCELL
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


// denk niet dat dit nodig is !!
#if defined(ESP8266)|| defined(ESP32) || defined(AVR) 
#endif

#define stepPin 4
#define dirPin 7
#define potPin A0 

const int HX711_dout = 5; //mcu > HX711 dout pin
const int HX711_sck = 6; //mcu > HX711 sck pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_calVal_eepromAdress = 0;
unsigned long t = 0;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
 
  Serial.begin(57600); 
  Serial.println();
  Serial.println("Starting...");

  float calibrationValue = 231.72; // calibration value
   // calibrationValue = 231.72; // uncomment this if you want to set this value in the sketch

  LoadCell.begin();
  //LoadCell.setReverseOutput();
  // TESTEN
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

  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Starting up ...");
}


void loop() {
   int potValue = analogRead(potPin); // Lees de potentiometerwaarde (0-1023)
 
   int stepDelay;

  if (potValue > 530) {  // Beweeg omhoog
    digitalWrite(dirPin, LOW);
    stepDelay = map(potValue, 530, 1023, 1000, 100);
    moveStepper(stepDelay);
  } 
  else if (potValue < 490) {  // Beweeg naar beneden
    digitalWrite(dirPin, HIGH);
    stepDelay = map(potValue, 490, 0, 1000, 100); 
    moveStepper(stepDelay);
  }

  static boolean newDataReady = 0;
  const int serialPrintInterval = 500; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      UpdateLCD(i);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}


void UpdateLCD(int kracht){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("kracht : ");
  lcd.setCursor(0,1);
  lcd.print(kracht);
}

void moveStepper(int stepDelay) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelay);
}
