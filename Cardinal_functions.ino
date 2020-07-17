// 24 Hour Engineer --- I wrote these functions for the Cardinal project
// I usually start my FUNction names with the prefix "fun"

int acceptableWindow(int funBigWindow, int funBigTime, long funLastActive){
  // Take in the max angle, max time, and the last buzz to
  // calculate how close we can swing from the origin before
  // vibrating again.
  int funAcceptableAngle = 0;
  buzzElapsedTime = (millis() - lastBuzzTime);
  funAcceptableAngle = map(buzzElapsedTime, 0, funBigTime, funBigWindow, 0);
  funAcceptableAngle = constrain(funAcceptableAngle, 0, 90);
  return(funAcceptableAngle);
}


void headingNotification(){
  // Make regular checks to see if it's time to
  // turn off the vibrator.
  if (millis() > headingNotificationTimer){
    digitalWrite(vibrationMotor, LOW);
  }
}

void LEDToy(int funIntensity){
  // This is just for turning the LED up or down.
  funIntensity = constrain(funIntensity, 0, 10);
  analogWrite(indicatorLED, pow(2, funIntensity)-1);
}

void LEDToyStrobe(int funFrequency){
  // This is just for blinking the LED faster or slower.
  funFrequency = constrain(funFrequency, 0, 60);
  tone(indicatorLED, funFrequency);
  if(funFrequency==0){
    noTone(indicatorLED);
  }
}

void relativeLED(int funHeading){
  // Based on the heading, make the LED brighter or dimmer.
  int funIntensity = 0;
  if (funHeading<=180){
    funIntensity = map(funHeading, 0, 180, 9, 0);
    funIntensity = constrain(funIntensity, 0, 8);
    analogWrite(indicatorLED, pow(2, funIntensity)-1);
  }else{
    funIntensity = map(funHeading, 180, 360, 0, 9);
    funIntensity = constrain(funIntensity, 0, 8);
    analogWrite(indicatorLED, pow(2, funIntensity)-1);
  }
  Serial.print("Intensity: ");
  Serial.println(pow(2, funIntensity)-1);
}

void startupVibration(int funOutput){
  // Play a few vibrations of increasing power
  int delayPeriod = 200;
  for (int i=90; i<255; i=i+40){
    analogWrite(funOutput, i);
    delay(delayPeriod);
    analogWrite(funOutput, 0);
    delay(delayPeriod);
  }
}

void boostBTPower(){
  // Set the Bluetooth power to max (4) and
  // set a timer for sixty seconds.
  ble.println("AT+BLEPOWERLEVEL=4");
  powerBoosting = millis() + 60000;
}

void bluetoothPowerMaintain(){
  // Check if sixty seconds have passed since the Bluetooth
  // power was boosted. If so, turn it back to minimum.
  if (millis() > powerBoosting){
    ble.println("AT+BLEPOWERLEVEL=-16");
    powerBoosting = 2147483647;
    ble.print("AT+BLEUARTTX=");)
    ble.println("Bluetooth low power");
//    Serial.println("Bluetooth low power");
    
//    LEDToy(0);                                // Reactivating these lines will deactivate the onboard LED after one minute
//    noTone(indicatorLED);
//    LEDToyIntensity = 0;
//    LEDToyFrequency = 0;
  }
}

void printHeading(){
  // Print the heading to the phone UART.
  // The heading will print as a value between -180.00 and 180.00
  ble.print("AT+BLEUARTTX=");
  ble.print("Heading: ");
  float funHeading = floatHeading;
  if(funHeading > 180.0){
    funHeading = (360 - funHeading);
    ble.print("-");
  }else{
    ble.print("+");
  }
  if(funHeading<100){
    ble.print("0");
  }
  if(funHeading<10){
    ble.print("0");
  }
  ble.print(funHeading);
  ble.print('\t');
  ble.println();
}
