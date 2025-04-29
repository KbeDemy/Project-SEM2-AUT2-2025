#include <HX711_ADC.h> // library voor ADC converter LOADCELL
#include <EEPROM.h> // opslaan van cali val voor LOADCELL
#include <Stepper.h> // 

// DEBUG Toggle
#define DEBUG 1  // 0 = alleen loadcell waarden, 1 = on full debugmode
#if DEBUG 
  #define Debug_println(x) Serial.println(x)
  #define Debug_print(x) Serial.print(x)
#else 
  #define Debug_print(x)
#endif

const uint8_t stopPin = 2; 

const uint8_t HX711_dout = 6;
const uint8_t HX711_sck = 7;
const uint8_t SP1 =  9; 
const uint8_t SP2 = 10;
const uint8_t SP3 = 11;
const uint8_t SP4 = 12;

 
HX711_ADC LoadCell(HX711_dout, HX711_sck);
#define STEPS 200
Stepper stepper(STEPS, SP1, SP2, SP3, SP4);

// global var
// newton related
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
    Debug_println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    LoadCell.setCalFactor(calibrationValue);
    Debug_println("Startup is complete");
  }
}

void ISR_BREAK(){
  Noodstop = true;
}

void loop(){
  while(!Noodstop){
    for(int i = 1; i < 6; i++){
      
      
      wantedValue = i * 5;
      Serial.print("---- nieuwe grafiek begint hier met als wantedValue :");
      Serial.print(wantedValue);
      Serial.println("  ----");
         readLoadCell();

      for(int j = 1; j < 11; j++){
        
           readLoadCell();
        if (newtonValue < wantedValue){
          while(newtonValue <= 1 ) {
            readLoadCell();
            stepper.setSpeed(100);
            stepper.step(50);
            Serial.print("    wantedValue ; ");
            Serial.print(wantedValue);
            Serial.print(";");
            Serial.print("    newtonValue ; ");
            Serial.println(newtonValue);
          }
          while((newtonValue > 1 && newtonValue < wantedValue) ){
            readLoadCell();
            stepper.setSpeed(60);
            stepper.step(50);
            Serial.print("    wantedValue ; ");
            Serial.print(wantedValue);
            Serial.print(";");
            Serial.print("    newtonValue ; ");
            Serial.println(newtonValue);
          }
        }
        while(newtonValue >= wantedValue) {
          readLoadCell();
          stepper.setSpeed(100);
          stepper.step(-50);
        }
      }
      delay(1000);
    }
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
