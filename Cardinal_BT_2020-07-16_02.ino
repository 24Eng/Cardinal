#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

//////////////////////24 Hour Engineer//////////////////////
// My code starts here. The above is from examples and//////
// may have been slightly modified./////////////////////////

int vibrationMotor = 11;                // Pin 11 supports PWM
int indicatorLED = 13;                  // Onboard LED, also support PWM
long headingNotificationTimer = 0;      // Used to timeout the vibrator
int notificationTimeout = 250;          // This is how long the vibrator says active
int IMUResetPin = 25;                   // Not currently used. Would be used to send a hardware reset to the IMU
int maximumAngleOfInterest = 60;        // This is the maximum angle from the origin we have to turn before it will vibrate again
bool buzzAgain = true;                  // Buzz again is a bit so we know that it's acceptable to vibrate if we see an origin crossing
int maximumTimeOfInterest = 10000;      // This is the maximum time we will wait before buzzAgain goes high. Even if we face the origin the whole time
long lastBuzzTime = 0;                  // We need to record when we last vibrated to know how long it has been
int buzzElapsedTime = 0;                // How many milliseconds SINCE the last vibration
bool trendingClockwise = false;         // A bit that indicates which side of the origin we are facing
bool oldTrendClockwise = false;         // What side of the origin we were at the last check
bool outOfRange = false;                // When this is high, we are facing at least 60° from the origin
float floatHeading = 0.0;               // Float version of the heading which we'll get from the IMU
int intHeading = floatHeading;          // In version of the heading which we'll get from the IMU
float headingCorrection = 0.0;          // When we reset the origin throught the controller, this is the variable where we store it
bool freshZero = false;                 // This bit goes high when we're resetting the origin
long powerBoosting = 60000;             // A timer for keeping the BT powered on high
int VBATPIN  = A9;                      // Analog pin 9 is connected to the battery
int LEDToyIntensity = 0;                // For fun, we can change the intensity of the onboard LED
int LEDToyFrequency = 0;                // For fun, we can change the strobe frequency of the onboard LED

void setup(void){
//  while (!Serial);  // required for Flora & Micro
  delay(500);
  Serial.begin(115200);
  
    /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) ){
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE ){
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  ble.verbose(false);  // debug info is a little annoying after this point!

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ){
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  
  }
//////////////////////24 Hour Engineer//////////////////////
// Below is the code I added. Above setup() code is from 
// example code sketches.

  pinMode(vibrationMotor, OUTPUT);            // Set the motor pin as an output
  pinMode(indicatorLED, OUTPUT);              // Set the onboard LED as an output
  pinMode(IMUResetPin, OUTPUT);               // Set the reset pin as an output
  digitalWrite(IMUResetPin, HIGH);            // Keep this high. Bringing it low would reset the IMU if it were connected
  
  if (!bno.begin()){
    Serial.print("No BNO055 detected");
    while (1);
  }
  startupVibration(vibrationMotor);           // This function buzzes three times so we know when it's booted.
  delay(1000);
}

void loop(void){
  serialCommands();                           // Serial example code was moved to a function
  
  unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

//  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {        // This is what the example code used to check the IMU, but we needed something faster
  if (true) {
    //enough iterations have passed that we can print the latest data
    floatHeading = orientationData.orientation.x;
    if(freshZero){
      printHeading();
      headingCorrection = floatHeading;
      freshZero = false;
    }
    floatHeading = (floatHeading - headingCorrection);
    if (floatHeading<0){
      floatHeading = floatHeading+360.0;
    }
    intHeading = floatHeading;
    float invFloatHeading = 360.0 - floatHeading;
    int invIntHeading = 360 - intHeading;
    if (intHeading < invIntHeading){
      trendingClockwise = true;
    }else{
      trendingClockwise = false;
    }
    
//    ble.print("AT+BLEUARTTX=");           // If you uncomment these lines, you will get the heading reading printed to the phone each cycle
//    ble.print("Heading: ");
//    if(floatHeading<100){
//      ble.print("0");
//    }
//    if(floatHeading<10){
//      ble.print("0");
//    }
//    ble.print(floatHeading);
//    ble.print('\t');
//    ble.println();
    
    
    
//    Serial.print("Heading: ");           // If you uncomment these lines, you will get the heading reading printed to the computer each cycle
//    Serial.println(floatHeading);
//    Serial.print("Position: ");
//    Serial.print(xPos);
//    Serial.print(" , ");
//    Serial.println(yPos);
//    Serial.print("Speed: ");
//    Serial.println(headingVel);
//    Serial.println("-------");
    
    
    
    if(!buzzAgain){
      // If the heading is outside the MAXIMUM window size, don't
      // bother calculating the shrinking window.
      if ((invIntHeading > maximumAngleOfInterest) && !trendingClockwise){
        buzzAgain = true;
        oldTrendClockwise = trendingClockwise;
        outOfRange = true;
      }
      if ((intHeading > maximumAngleOfInterest) && trendingClockwise){
        buzzAgain = true;
        oldTrendClockwise = trendingClockwise;
        outOfRange = true;
      }
    }
    
    // Know when the user is facing outside the range of interest.
    if (((invIntHeading > maximumAngleOfInterest) && !trendingClockwise) || ((intHeading > maximumAngleOfInterest) && trendingClockwise)){
      outOfRange = true;
    }

    if(!buzzAgain){
      // If the heading is within a shrinking window, check for origin
      // crossings again.
      int refinedWindow = acceptableWindow(maximumAngleOfInterest, maximumTimeOfInterest, lastBuzzTime);
      
      if ((invIntHeading > refinedWindow) && !trendingClockwise){
        buzzAgain = true;
        oldTrendClockwise = trendingClockwise;
      }
      if ((intHeading > refinedWindow) && trendingClockwise){
        buzzAgain = true;
        oldTrendClockwise = trendingClockwise;
      }
    }
    
    // If we're inside the acceptable window and cross the
    // origin, give a buzz.
    if (buzzAgain && ((invIntHeading < maximumAngleOfInterest) || (intHeading < maximumAngleOfInterest))){
      if(outOfRange){
        oldTrendClockwise = trendingClockwise;
        outOfRange = false;
      }
      if(oldTrendClockwise != trendingClockwise){
        digitalWrite(vibrationMotor, HIGH);
        headingNotificationTimer = millis() + notificationTimeout;
        lastBuzzTime = millis();
        buzzAgain = LOW;
      }
      oldTrendClockwise = trendingClockwise;
    }
    headingNotification();
    bluetoothPowerMaintain();
//    relativeLED(intHeading);                // This function will intensify the LED as you get closer to the origin
    
    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }
  
}
