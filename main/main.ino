// Written by Jonas Nichols for Helios: Team 1 Cansat 2 Month Project 2026
// NOTE: Cannot be uploaded while OpenLog is connected
// TODO:
//      find stack usage of async function with printf included inside and adjust the allocated bytes
//
// EXTRAS???
//      add lebron sunshine
//      alert function
//

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <String.h>
#include <utility/imumaths.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


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
const int panelServoPin = 24;
const int panelAInputPin = 36;
const int panelBInputPin = 39; 


// servo information
Servo releaseServo;
const releaseExtensionAmount = -55;
bool released = 0;
Servo panelServo;
const panelExtensionAmount = -90;
bool panelExtended = 0;

int teamId = 1;
float altitude = 0;
int velocity;
int packetCount = 0;
int temperature;
float initalPressure;
float batteryVoltage;
float panelAVolt;
float panelBVolt;
int morseUnit = 500;    // in ms
byte mechState = 0x00;

float gyroX;
float gyroY;
float gyroZ;

float accelerationX;
float accelerationY;
float accelerationZ;

long lastGPSsample;
long latitude;
long longitude;
int satsUsed;


// ESP-NOW
// teamId, timeElapsed, packetCount, stage, mechState, altitude, temperature, batteryVoltage, latitude, longitude, satsUsed,gyrox,y,z,accelerationX,accelerationY,accelerationZ,panelVolt1,panelVolt2,,extraData
String data;
String command;
esp_now_peer_info_t groundInfo;
// Ground MAC
uint8_t MAC[] = {
  0x##, 0x##, 0x##, 0x##, 0x##, 0x##
};



void sampleSensors();
void saveTransmitData();
void startExtensionTimer(void *parameter);
void panelExtend();
void release();

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);           // TODO: when command is received, appropiate action is taken; async trigger function; ESPNOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void alert(int num = 99);
void beep(int time);

int xTomV(int bits);

String stage = 'launchPad';


int setup() {

  xTaskCreatePinnedToCore(
    startExtensionTimer,    // function
    "startExtensionTimer",  // name
    2500,                   // Stack size
    NULL,                   // parameters
    1,                       // task priority
    1,                   // Task handle !!!may be necessasry
    1                       // Core to use
  )

  // Set up ESP-NOW / Wifi
  Serial.begin(115200);
  // start Wifi
  WiFi.mode(WIFI_STA);  
  while(!WiFi.STA.started()){ delay(100); }
  // init ESP NOW
  if (esp_now_init() != ESP_OK) { alert(); }
  
  esp_now_register_send_cb(OnDataSent);

  // register and add ground
  memcpy(peerInfo.peer_addr, MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { alert(); }

  // start Serial2 for the OpenLog
  Serial2.begin(115200);

  Wire.begin();

  // Set up bmp Sensor (Barometer and temperature)
  if (!bmpSensor.begin_I2C()) { alert(); }

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
  if (!bnoSensor.begin()) { alert(); }

  // Start GPS
  if (!GNSS.begin()) { alert(); }

  GNSS.setI2COutput(COM_TYPE_UBX); // Sets output to UBX only instead of the standard NMEA
  GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  // Start buzzer
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);

  // set up servos
  releaseServo.attach(releasePin);
  panelServo.attach(panelServoPin);

  // set up panel voltage readers
  analogSetAttenuation(ADC_0db);

}


int loop() {
  stage = 'launchPad';

  while (stage == 'launchPad') {

    sampleSensors();
    
    saveTransmitData();

    if (altitude >= 10) {
      stage = 'ascent';
    }

  }

  while (stage == 'ascent') {

    sampleSensors();
    
    saveTransmitData();

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
    
    saveTransmitData();

    // when stops falling
    if (altitude <= 10 || velocity < 1.8) {

      digitalWrite(buzzer, HIGH);
      stage = 'landed';

    }
  }

  while (stage == 'landed') {

    // receive commands
    sampleSensors();
    
    saveTransmitData();

  }
}


void sampleSensors() {

  // Sample solar cell voltages
  panelAVolt = xTomV(analogRead(panelAInputPin));
  panelBVolt = xTomV(analogRead(panelBInputPin));

  // Sample pressure and temperature sensors
  bmpSensor.performReading();
  temperature = bmpSensor.temperature;
  altitude = bmpSensor.readAltitude(initalPressure);


  // Sample rotation and acceleration sensors
  sensors_event_t gyroData, accelerometerData;
  bnoSensor.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bnoSensor.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  gyroX = gyroData->gyro.x;
  gyroY = gyroData->gyro.y;
  gyroZ = gyroData->gyro.z;

  accelerationX = accelerometerData->acceleration.x;
  accelerationY = accelerometerData->acceleration.y;
  accelerationZ = accelerometerData->acceleration.z;

  // Only sample GPS again if it has been 1 second
  if (millis() - lastGPSsample > 1000) {

    // sample GPS
    longitude = GNSS.getLongitude();
    latitude = GNSS.getLatitude();
    satsUsed = GNSS.getSIV();
    
    lastGPSsample = millis();
  
  }

}


void release() {
  if (released) { return; }

  releaseServo.write(releaseExtensionAmount);
}


void panelExtend() {
  if (panelExtended) { return; }

  panelServo.write(panelExtensionAmount);
  mechState = 0x11;
}


void startExtensionTimer(void *parameter) { // parameter required for FreeRTOS task

  vTaskDelay(5000 / portTICK_PERIOD_MS); // 5000ms / period in ms

  panelExtend()
  Serial.printf("Bytes free in extension timer function: ", uxTaskGetStackHighWaterMark(NULL));
}


void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {
  command = String((char*)incomingData);

  switch (command[0]) {
    case "r":
      release();
      break;
    case "e":
      panelExtend();
      break;
    case "a":
      alert();
      break;
  }
}


void saveTransmitData() {
  data = 1 + ',' + 
    millis() + ',' + 
    packetCount + ',' + 
    stage + ',' + 
    mechState + ',' + 
    altitude + ',' + 
    temperature + ',' + 
    batteryVoltage + ',' + 
    latitude + ',' + 
    longitude + ',' + 
    satsUsed + ',' + 
    gyroX + ',' +
    gyroY + ',' +
    gyroZ + ',' +
    accelerationX + ',' + 
    accelerationY + ',' + 
    accelerationZ + ',' + 
    panelAVolt + ',' + 
    panelBVolt;

  Serial2.println(data);
  
  if (esp_now_send(MAC, (uint8_t *) &data, sizeof(data)) == ESP_OK) {
    // if sent correctly
    packetCount++;
  }
  
  
  
  // teamId, timeElapsed, packetCount, stage, mechState, altitude, temperature, batteryVoltage,
  // latitude, longitude, satsUsed,gyrox,y,z,accelerationX,accelerationY,accelerationZ,panelVolt1,panelVolt2,,extraData
}

// plays an error code based on input
void alert(int num) {
  if (num == 99) { // buzzer on
    digitalWrite(buzzer, HIGH);
  }
  else if (num > 5) {

    for (int i = 0; i < 5; i++) {
      
      if (num > 0) {
        beep(morseUnit);
        num--;
      }
      else {
        beep(morseUnit*3);
      }
      delay(morseUnit);
      
    }
  }
  else {
    for (int i = 0; i < 5; i++) {
      
      if (num > 0) {
        beep(morseUnit*3);
        num--;
      }
      else {
        beep(morseUnit);
      }
      delay(morseUnit);
      
    }
  }
}

// beeps for length
void beep(int length) {
  digitalWrite(buzzer, HIGH);
  delay(length);
  digitalWrite(buzzer, LOW);
}

// converts the analog input to mV
int xTomV(int bits) {
  return (((bits / 4095) * 850) + 100);
}