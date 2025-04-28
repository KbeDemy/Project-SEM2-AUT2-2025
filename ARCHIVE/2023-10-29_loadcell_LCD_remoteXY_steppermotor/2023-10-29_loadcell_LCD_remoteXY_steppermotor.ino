/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.
   If you have other time consuming code running (i.e. a graphical LCD), consider calling update() from an interrupt routine,
   see example file "Read_1x_load_cell_interrupt_driven.ino".

   This is an example sketch on how to use this library
*/

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#include <Wire.h>               //?
#include <LiquidCrystal_I2C.h>  //to communicatie ith LCD
#endif

//pins:
const int HX711_dout = 5;  //mcu > HX711 dout pin
const int HX711_sck = 4;   //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

//hardware adress I2C device
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

/*   
 *   Basic example code for controlling a stepper without library
 *      
 *   by Dejan, https://howtomechatronics.com
 */

// defines pins for steppermotors
#define stepPin 6
#define dirPin 7 


/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.11 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.11.4 or later version;
     - for iOS 1.9.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>
#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_SERIAL_RX 2
#define REMOTEXY_SERIAL_TX 3
#define REMOTEXY_SERIAL_SPEED 9600


// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 39 bytes
  { 255, 2, 0, 11, 0, 32, 0, 16, 31, 1, 4, 0, 43, 15, 10, 60, 2, 26, 1, 0,
    6, 18, 12, 12, 123, 31, 108, 101, 100, 0, 67, 1, 38, 5, 20, 5, 24, 26, 11 };

// this structure defines all the variables and events of your control interface
struct {

  // input variables
  int8_t slider_1;   // =0..100 slider position
  uint8_t button_1;  // =1 if button pressed, else =0

  // output variables
  char slider[11];  // string UTF8 end zero

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define PIN_BUTTON_1 9

void setup() {
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");
  lcd.init();
  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  float calibrationValue;     // calibration value (see example file "Calibration.ino")
  calibrationValue = 429.88;  // uncomment this if you want to set the calibration value in the sketch
#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  } else {
    LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
    Serial.println("Startup is complete");
  }
  {
    RemoteXY_Init();
    Serial.begin(9600);
    pinMode(PIN_BUTTON_1, OUTPUT);
  }
  {
    // Sets the two steppermotor pins as Outputs
    pinMode(stepPin,OUTPUT); 
    pinMode(dirPin,OUTPUT);
  }
    // TODO you setup code
  
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;  //increase value to slow down serial print activity

  {
    RemoteXY_Handler();

    digitalWrite(PIN_BUTTON_1, (RemoteXY.button_1 == 0) ? LOW : HIGH);
    Serial.print("slider value = ");
    Serial.println(RemoteXY.slider_1);
    // TODO you loop code
    // use the RemoteXY structure for data transfer
    // do not call delay(), use instead RemoteXY_delay()

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      /*added data for LCD
      lcd.backlight();
      //Nachricht ausgeben
      lcd.setCursor(0,0);
      lcd.print("Downward force");
      lcd.setCursor(0,1);
      lcd.print(i);
      lcd.setCursor(8,1);
      lcd.print("gr");
      //-----Hardware Adressierung-----
      //added data for LCD
      */
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

    //added data for LCD
    lcd.backlight();
    //Nachricht ausgeben
    lcd.setCursor(0, 0);
    lcd.print(RemoteXY.slider_1 *100.0);
    lcd.setCursor(7, 0);
    lcd.print("=setvalue");
    lcd.setCursor(0, 1);
    lcd.print(LoadCell.getData());
    lcd.setCursor(7, 1);
    lcd.print("gr");
    //added data for LCD
    digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 800; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(7000/(RemoteXY.slider_1 + 0.01) );    
    /* by changing this time delay between the steps we can change the rotation speed
    higher number slower speed 7000
    lower number higher speed 70
    */
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(70); 
  }
  //delay(1000); // One second delay
  /*
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 1600; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(500);
  }
  */
  delay(1000);
  }
}
