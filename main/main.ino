// Written by Jonas Nichols for Helios: Cansat 2 Month Project 2026
// TODO:
//      add ESPNOW Transmission
//      add ESPNOW Reception
//      add sd card system
//      find stack usage of async function with printf included inside and adjust the allocated bytes
//
// EXTRAS???
//       add music
//
// groud station soon

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <String.h>
#include <utility/imumaths.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Servo.h>


// Barometer and Temperature sensor
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
Adafruit_BMP3XX bmpSensor; // I2C var for Barometer

// Accelometer and Rotation sensor
Adafruit_BNO055 bnoSensor = Adafruit_BNO055(55, 0x28, &Wire)

// GPS
SFE_UBLOX_GNSS GNSS;


// Sensor pins
const int SCL = 5;
const int SDA = 4;

// Other pins
const int operationLight = 27;
const int buzzer = 26;
const int releasePin = 25;
const int panelPin = 24;

Servo releaseServo;
const releaseExtensionAmount = 45;
Servo panelServo;
const panelExtensionAmount = 45;

float altitude = 0;
int velocity;
int packetCount = 0;
int temperature;
float initalPressure;

float orientationX;
float orientationY;
float orientationZ;

float accelerationX;
float accelerationY;
float accelerationZ;

long lastGPSsample;
long latitude;
long longitude;


void sampleSensors();
void saveData();            // TODO: add serial sd card connection
void transmitTelemetry();   // TODO: add ESPNOW protocol things

void release();             // TODO: Probably connect a servo

void onReceive();           // TODO: when command is received, appropiate action is taken; async trigger function; ESPNOW


String stage = 'launchPad';


int setup() {

  xTaskCreatePinnedToCore(
    startExtensionTimer,    // function
    "startExtensionTimer",  // name
    2500,                   // Stack size
    NULL,                   // parameters
    1                       // task priority
    NULL,                   // Task handle !!!may be necessasry
    1                       // Core to use
  )

    Wire.begin();

    // Set up bmp Sensor (Barometer and temperature)
    bmpSensor.begin_I2C();

    // Set up oversampling and filter initialization
    bmpSensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmpSensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmpSensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmpSensor.setOutputDataRate(BMP3_ODR_50_HZ);

    // First reading is inaccurate; throw away values
    sampleSensors();

    // Set pressure of current (lowest) altitude in Hpa
    sampleSensors();
    initalPressure = (bmpSensor.pressure / 100);

    // Set up bno Sensor (orintation and accelometer)
    bnoSensor.begin();

    // Start GPS
    GNSS.begin();

    GNSS.setI2COutput(COM_TYPE_UBX); // Sets output to UBX only instead of the standard NMEA
    GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

    // Start buzzer
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);

    // set up servos
    releaseServo.attach(releasePin);
    panelServo.attach(panelPin);

}


int loop() {
  stage = 'launchPad';

  while (stage == 'launchPad') {

    sampleSensors();
    saveData();
    transmitTelemetry();

    if (altitude >= 10) {
      stage = 'ascent';
    }

  }

  while (stage == 'ascent') {

    sampleSensors();
    saveData();
    transmitTelemetry();

    // when at desired height
    if (altitude >= 530) {
      stage = 'apogee';
    }

  }

  while (stage == 'apogee') {

    release();
    startExtensionTimer();

    stage = 'decent';

  }

  while (stage == 'decent') {

    // receive commands
    sampleSensors();
    saveData();
    transmitTelemetry();

    // when stops falling
    if (altitude <= 10 || velocity < 1.8) {

      digitalWrite(buzzer, HIGH);
      stage = 'landed';

    }
  }

  while (stage == 'landed') {

    // receive commands
    sampleSensors();
    saveData();
    transmitTelemetry();

  }
}


void sampleSensors() {

  // Sample pressure and temperature sensors
  bmpSensor.performReading();
  temperature = bmpSensor.temperature;
  altitude = bmpSensor.readAltitude(initalPressure);


  // Sample rotation and acceleration sensors
  sensors_event_t orientationData, accelerometerData;
  bnoSensor.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bnoSensor.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  orientationX = orientationData->orientation.x;
  orientationY = orientationData->orientation.y;
  orientationZ = orientationData->orientation.z;

  accelerationX = accelerometerData->acceleration.x;
  accelerationY = accelerometerData->acceleration.y;
  accelerationZ = accelerometerData->acceleration.z;

  // Only sample GPS again if it has been 1 second
  if (millis() - lastGPSsample > 1000) {

    // sample GPS
    longitude = GNSS.getLongitude();
    latitude = GNSS.getLatitude();
    
    lastGPSsample = millis();
  
  }

}


void release() {
  releaseServo.write(releaseExtensionAmount);
}


void startExtensionTimer(void *parameter) { // parameter required for FreeRTOS task
  vTaskDelay(5000 / portTICK_PERIOD_MS); // 5000ms / period in ms
  panelServo.write(panelExtensionAmount);
  Serial.printf("Bytes free in extension timer function: ", uxTaskGetStackHighWaterMark(NULL));
}