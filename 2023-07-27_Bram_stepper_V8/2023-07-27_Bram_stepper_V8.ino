// Constants
const unsigned long timerDuration = 360000;  // Timer duration in milliseconds
//------------------------------------------------------------------------------
// defines (pins 4 and 7 are for Z axis)
#define stepPin 4
#define dirPin 7

// Pin numbers for valve and pump control
const int valvePin = 3;
const int pumpPin = 5;

const int trigerPin = 2;  
//trigerPinsignaal pin die aangeeft dat de correcte waarde aan kracht bereikt is in de loadcell
int togleValue = 0;       //0 is naar beneden 1 is naar omhoog

// Variables
unsigned long startTime = 0;    // Time when the button was pressed
unsigned long elapsedTime = 0;  // Elapsed time since the button press

// Constants
//const unsigned long timerDuration = 5000;  // Timer duration in milliseconds

void setup() {
  Serial.begin(57600);
  //initial settings for the stepper motors
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  //initial setting to make sure the trigerpin reads the value
  //from the arduino that inidcates if the correct presure value has been reached
  pinMode(trigerPin, INPUT);
  //initial settings for the valve and pump
  pinMode(valvePin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  digitalWrite(valvePin, LOW);  // Initially closed
  digitalWrite(pumpPin, LOW);   // Initially off
}
void loop() {

  if (digitalRead(trigerPin) == 0) {
    if (startTime == 0) {
      // Start the timer
      startTime = millis();
      //Close the valve and start the pump
      digitalWrite(valvePin, HIGH);
      digitalWrite(pumpPin, HIGH);
    }
  } else {
    // de triger pin is nog niet afgegaan
    if (startTime != 0) {
      //als de timer bezig is en de trigger pin is nog niet afgegaan zet hij alles op nul
      startTime = 0;
      elapsedTime = 0;
      //Close the valve and start the pump
      digitalWrite(valvePin, LOW);
      digitalWrite(pumpPin, LOW);
    }
  }
  if (startTime != 0) {
    elapsedTime = millis() - startTime;
    if (elapsedTime >= timerDuration) {
      // Timer duration reached, switch the rotation of the stepper motors by setting the togle value to 1
      togleValue = 1;
      startTime = 0;    //de timer is afgegaan dus alle meet waardes zoals start tijd worden terug naar nul gezet
      elapsedTime = 0;  // detimer is afgegaan dus alle meetwaardes zoals tijd worden terug op nul gezet

      //de pomp word afgezet en de valve word ook geopend
      digitalWrite(valvePin, LOW);
      digitalWrite(pumpPin, LOW);
    }
  }
  Serial.print("toglevalue = ");
  Serial.println(togleValue);
  digitalWrite(dirPin, togleValue);  // Enables the motor to move in a particular direction HIGH AND LOW forward backwards
  // Makes 200 pulses for making one full cycle rotation
  if (digitalRead(trigerPin) || togleValue) {
    for (int x = 0; x < 2; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);  // by changing this time delay between the steps we can change the rotation speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
    delay(500);  // One second delay
    Serial.println(digitalRead(trigerPin));
  }
}