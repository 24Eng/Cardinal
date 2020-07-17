// 24 Hour Engineer --- This whole tab is from Adafruit example code //////////////

void printEvent(sensors_event_t* event) {
  Serial.println();
  Serial.print(event->type);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  Serial.print(": x= ");
  Serial.print(x);
  Serial.print(" | y= ");
  Serial.print(y);
  Serial.print(" | z= ");
  Serial.println(z);
}



void serialCommands(){
  // Check for user input
  char inputs[BUFSIZE+1];

  if ( getUserInput(inputs, BUFSIZE) ){
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response status
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  // Some data was found, it's in the buffer
  Serial.print(F("[Recv] "));
//  incomingMessage[0] = (ble.buffer);
//  Serial.println(incomingMessage);
  // When the system receives the signal that button one (1)
  // was pressed on Adafruit's Bluefruit Connect program
  // zero reading to reorient with the current heading as
  // forward.
  if (strcmp(ble.buffer, "!B11:") == 0) {
    boostBTPower();
    delay(3);
    freshZero = true;
  }
  // Button two (2) in Adafruit's Bluefruit Connect program
  // will boost the BT power for one minute to allow for
  // more stable connection. Usually BT does this automatically
  // but why would anything be easy?
  if (strcmp(ble.buffer, "!B219") == 0) {
    boostBTPower();
    delay(3);
    ble.print("AT+BLEUARTTX=");
    ble.println("Bluetooth max power\t\t");
  }
  // Button three (3) in Adafruit's Bluefruit Connect program
  // will request to see the current heading.
  if (strcmp(ble.buffer, "!B318") == 0) {
    boostBTPower();
    delay(3);
    printHeading();
  }
  // Button four (4) in Adafruit's Bluefruit Connect program
  // will request to see the current heading.
  if (strcmp(ble.buffer, "!B417") == 0) {
    boostBTPower();
    delay(3);
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    Serial.print("Battery Voltage: " );
    Serial.println(measuredvbat);
    
    ble.print("AT+BLEUARTTX=");
    ble.print("Battery Voltage: ");
    ble.print(measuredvbat);
    ble.println("\t");
    ble.println();
  }
  // Up button in Adafruit's Bluefruit Connect program
  // will increase the intensity of the onboard LED.
  if (strcmp(ble.buffer, "!B516") == 0) {
    boostBTPower();
    LEDToyIntensity++;
    if(LEDToyIntensity>10){
      LEDToyIntensity = 10;
    }
    LEDToyFrequency = 0;
    noTone(indicatorLED);
    LEDToy(LEDToyIntensity);
    ble.print("AT+BLEUARTTX=");
    ble.print("LED Brightness: ");
    ble.print(LEDToyIntensity);
    ble.println("\t");
  }
  // Down button in Adafruit's Bluefruit Connect program
  // will decrease the intensity of the onboard LED.
  if (strcmp(ble.buffer, "!B615") == 0) {
    boostBTPower();
    LEDToyIntensity--;
    if(LEDToyIntensity<0){
      LEDToyIntensity = 0;
    }
    LEDToyFrequency = 0;
    noTone(indicatorLED);
    LEDToy(LEDToyIntensity);
    ble.print("AT+BLEUARTTX=");
    ble.print("LED Brightness: ");
    ble.print(LEDToyIntensity);
    ble.println("\t");
  }
  // Right button in Adafruit's Bluefruit Connect program
  // will increase the strobing frequency of the onboard LED.
  if (strcmp(ble.buffer, "!B813") == 0) {
    boostBTPower();
    LEDToyFrequency++;
    if(LEDToyFrequency>60){
      LEDToyFrequency = 60;
    }
    LEDToyIntensity = 0;
    LEDToy(LEDToyIntensity);
    LEDToyStrobe(LEDToyFrequency);
    ble.print("AT+BLEUARTTX=");
    ble.print("LED Hz: ");
    ble.print(LEDToyFrequency);
    ble.println("\t");
  }
  // Left button in Adafruit's Bluefruit Connect program
  // will decrease the strobing frequency of the onboard LED.
  if (strcmp(ble.buffer, "!B714") == 0) {
    boostBTPower();
    LEDToyFrequency--;
    if(LEDToyFrequency<0){
      LEDToyFrequency = 0;
    }
    LEDToyIntensity = 0;
    LEDToy(LEDToyIntensity);
    LEDToyStrobe(LEDToyFrequency);
    ble.print("AT+BLEUARTTX=");
    ble.print("LED Hz: ");
    ble.print(LEDToyFrequency);
    ble.println("\t");
  }
  Serial.println(ble.buffer);
  ble.waitForOK();
}

bool getUserInput(char buffer[], uint8_t maxSize){
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}
