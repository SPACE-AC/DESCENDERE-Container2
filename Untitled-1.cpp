#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Seeed_BME280.h>
#include <Servo.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>

// CONFIG - START

#define LED1_PIN 19
#define LED2_PIN 6
#define LED3_PIN 9

#define voldivpin 18
#define R1_OHM 2000.0F
#define R2_OHM 1250.0F

#define xbee Serial2

// CONFIG - END

TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(55, bno);
Adafruit_BME280 bme;

float gyro_r = 0;
float gyro_p = 0;
float gyro_y = 0;
float accel_r = 0;
float accel_p = 0;
float accel_y = 0;
float mag_r = 0;
float mag_p = 0;
float mag_y = 0;
float temp = 0;
float altitude = 0;
float voltage = 0;

#define SEALEVELPRESSURE_HPA 1013.25

void beep(int count) {
    for (int i = 0; i < count; i++) {
        digitalWrite(LED1_PIN, HIGH);
        delay(100);
        digitalWrite(LED1_PIN, LOW);
        delay(100);
    }
}

void setup() {
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    pinmode(voldivpin, INPUT);
    if (bme.begin(0x76))
        Serial.println("✔ SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        beep(1);
        delay(500);
    }
    if (bno.begin())
        Serial.println("✔ SUCCEED: PCB BNO055");
    else {
        Serial.println("[FAILED] Unable to set up PCB BNO055!");
        beep(2);
        delay(500);
    }
    bno.setExtCrystalUse(true);

    if (SD.begin(10))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        beep(5);
    }
}

void getBmeData() {
    temp = bme.readTemperature();
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void getBNOData() {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyro_r = gyro.x();
    gyro_p = gyro.y();
    gyro_y = gyro.z();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    accel_r = accel.x();
    accel_p = accel.y();
    accel_y = accel.z();
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    mag_r = mag.x();
    mag_p = mag.y();
    mag_y = mag.z();
}

unsigned long lastMillis = 0;
void loop() {
    getBMEData();
    getBNOData();
    if (xbee.available()) {
        Serial.print(xbee.read());
    }
    if (millis() - lastMillis > 1000) {
        Serial.print(altitude + ',' +
                     temp + ',' + String(voltage) + ',' + gyro_r + ',' + gyro_p + ',' + gyro_y +
                     ',' + accel_r + ',' + accel_p + ',' + accel_y + ',' + mag_r + ',' +
                     mag_p + ',' + mag_y);
    }
}