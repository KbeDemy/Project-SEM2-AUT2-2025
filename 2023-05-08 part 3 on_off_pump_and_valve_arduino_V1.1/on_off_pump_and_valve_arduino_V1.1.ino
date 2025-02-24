// Define the pins
// pin 8 is set to activate relay 1 control vacuumvalve (normally closed) HIGH opens the valve
// pin 9 is set to activate relay 2 control vacuumpump
// activate Arduino IDE
// activate serial

// data 0 implemented in serial monitor will deactivate relays
// data 1 implemented in serial monitor will activate relay 1 (opens the valve)
// data 2 implemented in serial monitor will activate relay 2 (starts the pump)
// data 3 implemented in serial monitor will activate relay 1 and afterwards 2 for a est time of loops set interval
int IN1 = 8;
int IN2 = 9;
// minutes defines the time used for generating droplets with a loop function passing through 60x 1000 milliseconds
// can be changed to seconds if 60x is changed to 1x but interval is more 155 sec for 1 minute!!
int minutes = 6;

void setup() {
  // put your setup code here, to run once:
  // Initialize the serial communication
  // synchronize the serial monitor to the same baud rate as in Serial.begin
  Serial.begin(9600);

  // Set the pin as an output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}
void loop() {
  // Check if there is any incoming data on the serial port
  if (Serial.available() > 0) {
    // Read the incoming command
    char command = Serial.read();
    // Check the command and switch the pin on or off accordingly
    if (command == '0') {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    } else if (command == '1') {
      // open the vacuumvalve IN1
      digitalWrite(IN1, HIGH);
      delay(1000);
    } else if (command == '2') {
      // pump on IN2
      digitalWrite(IN2, HIGH);
      delay(10000);
      // pump off
      digitalWrite(IN2, LOW);
    } else if (command == '3') {
      // Start pump for set time, open vacuumvalve, stop pump, wait for a set time, deactivate vacuumvalve, shut down system
      digitalWrite(IN2, HIGH);
      //delay(interval);
      for (int x = 0; x <= minutes; x++) {
        for (int y = 0; y < 60; y++) {
          delay(1000);
        }
      }
      // keep pumping and break vacuum
      digitalWrite(IN1, HIGH);
      // pump off
      digitalWrite(IN2, LOW);
      delay(1000);
      digitalWrite(IN1, LOW);
    } else if (command == '4') {
      // pump off
      digitalWrite(IN2, LOW);
      delay(1000);
      digitalWrite(IN1, HIGH);
      
    }
  }
}
