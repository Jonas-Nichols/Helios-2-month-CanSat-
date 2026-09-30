#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <String.h>


// Other pins
const int operationLight = 18;
const int buzzer = 19;
const int releasePin = 9;
const int panelServoPin = 10;
const int panelAInputPin = 33;
const int panelBInputPin = 26;

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

String stage = "launchPad";

void I2CSetup() {
    Wire.begin();
};

#include "Adafruit_BMP3XX.h"
// Barometer and Temperature sensor
Adafruit_BMP3XX bmpSensor; // I2C var for Barometer
void BMPUse();
void BMPSetup() {
    // Set up bmp Sensor (Barometer and temperature)
  if (!bmpSensor.begin_I2C()) { alert(3); }
  // Set up oversampling and filter initialization
  bmpSensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmpSensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmpSensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmpSensor.setOutputDataRate(BMP3_ODR_50_HZ);
  BMPUse();
  BMPUse();
  initalPressure = (bmpSensor.pressure / 100);
};
void BMPUse() {
// Sample pressure and temperature sensors
  bmpSensor.performReading();
  temperature = bmpSensor.temperature;
  altitude = bmpSensor.readAltitude(initalPressure);
};

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// Accelometer and Rotation sensor
Adafruit_BNO055 bnoSensor = Adafruit_BNO055(55, 0x28, &Wire);
void BNOSetup() {
    // Set up bno Sensor (orintation and accelometer)
  if (!bnoSensor.begin()) { alert(4); }
};
void BNOUse() {
  // Sample rotation and acceleration sensors
  sensors_event_t gyroData, accelerometerData;
  bnoSensor.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bnoSensor.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  gyroX = gyroData.gyro.x;
  gyroY = gyroData.gyro.y;
  gyroZ = gyroData.gyro.z;

  accelerationX = accelerometerData.acceleration.x;
  accelerationY = accelerometerData.acceleration.y;
  accelerationZ = accelerometerData.acceleration.z;
};

#include <SparkFun_u-blox_GNSS_v3.h>
// GPS
SFE_UBLOX_GNSS GNSS;
void GNSSSetup() {
    // Start GPS
  if (!GNSS.begin()) { alert(5); }
  GNSS.setI2COutput(COM_TYPE_UBX); // Sets output to UBX only instead of the standard NMEA
  GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
};
void GNSSUse() {
  // Only sample GPS again if it has been 1 second
  if (millis() - lastGPSsample > 1000) {

    // sample GPS
    longitude = GNSS.getLongitude();
    latitude = GNSS.getLatitude();
    satsUsed = GNSS.getSIV();
    
    lastGPSsample = millis();
  
  }
};

#include <ESP32Servo.h>
// servo information
Servo releaseServo;
const int releaseExtensionAmount = 55;		// [-90,90]
bool released = 0;
Servo panelServo;
const int panelExtensionAmount = 90;			// [-90,90]
bool panelExtended = 0;
float PEtimeStarted = 0;
void ServoSetup() {
      // set up servos
  releaseServo.attach(releasePin);
  panelServo.attach(panelServoPin);

	releaseServo.write(90);
	panelServo.write(90);
};
void ServoUse() {
    releaseServo.write(90-releaseExtensionAmount);

    pa nelServo.write(90-panelExtensionAmount);
  mechState = 0x11;
  panelExtended = true;
};

void panelVSetup() {
    // set up panel voltage readers
    analogSetAttenuation(ADC_0db);
}
void panelVUse() {
    // Sample solar cell voltages
    panelAVolt = xTomV(analogRead(panelAInputPin));
    panelBVolt = xTomV(analogRead(panelBInputPin));
}

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
// ESP-NOW
// teamId, timeElapsed, packetCount, stage, mechState, altitude, temperature, batteryVoltage, latitude, longitude, satsUsed,gyrox,y,z,accelerationX,accelerationY,accelerationZ,panelVolt1,panelVolt2,,extraData
String data;
String command;
esp_now_peer_info_t groundInfo;
// Ground MAC
uint8_t MAC[] = {
  // 0x##, 0x##, 0x##, 0x##, 0x##, 0x##
};
void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
);
void ESPNOWSetup() {
  // Set up ESP-NOW / Wifi
  Serial.begin(115200);
  // start Wifi
  WiFi.mode(WIFI_STA);  
  while(!WiFi.STA.started()){ delay(100); }
  // init ESP NOW
  if (esp_now_init() != ESP_OK) { alert(1); }
  
  esp_now_register_send_cb(esp_now_send_cb_t(onDataSent));

  // register and add ground
  memcpy(groundInfo.peer_addr, MAC, 6);
  groundInfo.channel = 0;
  groundInfo.encrypt = false;
  if (esp_now_add_peer(&groundInfo) != ESP_OK) { alert(2); }
  // register callback function
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
};
void ESPNOWUse() {
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
if (esp_now_send(MAC, (uint8_t *) &data, sizeof(data)) == ESP_OK) {
    // if sent correctly
    packetCount++;
  }
};
void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {
  command = String((char*)incomingData);

  switch (command[0]) {
    case 'r':
      release();
      break;
    case 'e':
      panelExtend();
      break;
    case 'a':
      alert();
      break;
  }
}

void buzzerUse() {
    // Start buzzer
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);
};

void openLogSetup() {
    // start Serial2 for the OpenLog
    Serial2.begin(115200);
};
void openLogUse() {
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
};