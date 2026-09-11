#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <String.h>
#include <utility/imumaths.h>
#include <Adafruit_BNO055.h>


// Barometer and Temperature sensor
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
Adafruit_BMP3XX bmpSensor; // I2C var for Barometer

// Accelometer and Rotation sensor
Adafruit_BNO055 bnoSensor = Adafruit_BNO055(55, 0x28, &Wire)


// Sensor pins
const int SCL = 5;
const int SDA = 4;

// Other pins
const int operationLight = 27;

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

void sampleSensors();       // add gps; add acceleometer
void saveData();            // TODO: add serial sd card connection
void transmitTelemetry();   // TODO: add ESPNOW protocol things

void release();             // TODO: Probably connect a servo
void startExtensionTimer(); // TODO: figure out async timer stuff

void onReceive();           // TODO: when command is received, appropiate action is taken; async trigger function; ESPNOW


String stage = 'launchPad';


int setup() {
  // TODO: create receiver function


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
    bnoSensor.begin()


}


int loop() {
    stage = 'launchPad';

    while (stage =='launchPad')
    {

        sampleSensors();
        saveData();
        transmitTelemetry();

        if (altitude >= 10) {
            stage = 'ascent';
        }

    }

    while (stage == 'ascent')
    {

        sampleSensors();
        saveData();
        transmitTelemetry();

        if (altitude >= 530)
        {
            stage = 'apogee';
        }
    }

    while (stage == 'apogee')
    {
        release();
        startExtensionTimer();

        stage = 'decent';
    }

    while (stage == 'decent')
    {
        // receive commands
        sampleSensors();
        saveData();
        transmitTelemetry();

        if (altitude <= 10 || velocity < 1.8)
        {
            stage = 'ascent';
        }
    }

    while (stage == 'landed')
    {
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

    orientationX = orientationData->orientation.x
    orientationY = orientationData->orientation.y
    orientationZ = orientationData->orientation.z

    accelerationX = accelerometerData->acceleration.x
    accelerationY = accelerometerData->acceleration.y
    accelerationZ = accelerometerData->acceleration.z



    // TODO: other sensors, GPS; 

}