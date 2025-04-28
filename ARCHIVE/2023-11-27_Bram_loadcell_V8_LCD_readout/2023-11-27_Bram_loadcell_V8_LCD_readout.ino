//combination of working loadcell with LCD_readout
//
const int weight = 2000; //force in grams
//-------------------------------------------------------------------------------------------------------------------
#include <HX711_ADC.h>

#include <EEPROM.h>
#include <Wire.h>//?
#include <LiquidCrystal_I2C.h>//to communicatie ith LCD


/*pins:
mcu > HX711 d out pin proberen op een andere pin aan te sluiten 
dan kan alles op 1 arduino gezet worden.of kijken of we het me 2 doen.
mcu > HX711 sck pin
*/
const int HX711_dout = 4;  //mcu > HX711 dout pin
const int HX711_sck = 5;   //mcu > HX711 sck pin
const int trigerPin = 2;

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
//hardware adress I2C device
LiquidCrystal_I2C lcd(0x27,16,2);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

void setup() {
  pinMode(trigerPin, OUTPUT);
  //pinMode(trigerPin,HIGH);
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");  //BRAM-dees laten staan is gewoon handig ook op einden niet weg doen voor als ik er aan moet voort werken
  lcd.init();
  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  float calibrationValue;     // calibration value (see example file "Calibration.ino")
  //calibrationValue = 429.88;  // 5 kg load cell with large platform  uncomment this if you want to set the calibration value in the sketch
  calibrationValue = -231.3;  // 10 kg load cell with large platform  uncomment this if you want to set the calibration value in the sketch to a different loadcell

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } 
  else {
    LoadCell.setCalFactor(calibrationValue);  // set calibration value (float) BRAM-hier gaan we afblijven tot dat alles marcheert stel dat het niet nodig is / werkt proberen uitlaten want calibratie word in geheugen van de arduino op geslagen en is dus microcontroler specifiek.
    Serial.println("Startup is complete");
  }
}

void loop() {
  static boolean newDataReady = 0;    // BRAM-deze code kan eventueel ook worden weg gewerkt maar dan moet de new dataReady waarde standaard op aan worden gezet is beetje risky
  const int serialPrintInterval = 0;  //increase value to slow down serial print activity BRAM- deze lijn kan in de finale code worden weg gelaten om klucht te laten vallen

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();  //BRAM- dit is de waarde die nodig is om de steppers aan te sturen en te doen stoppen. ook om de limieten op in te stellen
      //Serial.print("Load_cell output val: ");
      Serial.println(i);
       //added data for LCD
       lcd.backlight();
       //Nachricht ausgeben
       lcd.setCursor(0,0);
       lcd.print("Downward force");
       lcd.setCursor(0,1);
       lcd.print(i);
       lcd.setCursor(8,1);
       lcd.print("gr");
       //added data for set weight
       lcd.setCursor(11,1);
       lcd.print(weight);
      //-----Hardware Adressierung-----
      //added data for LCD

      if (i >= weight) {
        Serial.println("trigerPin, HIGH");
        digitalWrite(trigerPin, LOW);
      } else {
        digitalWrite(trigerPin, HIGH);
      }
      newDataReady = 0;
      t = millis();
    }
  }

  //BRAM-dees gaan we waarschijnlijk niet van doen hebben is om te resetten op 0
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
