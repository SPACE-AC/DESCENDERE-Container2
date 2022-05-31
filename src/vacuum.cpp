#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Seeed_BME280.h>
#include <Servo.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// CONFIG - START

#define LED1_PIN 0
#define LED2_PIN 1
#define BUZZER_PIN 3
#define CAMERA_PIN 6

#define SERVO_PARA_PIN 4
#define SERVO_BREAK_PIN 5

#define VOLTAGE_PIN 21
#define R1_OHM 3000.0F
#define R2_OHM 1740.0F

#define xbeeGS Serial3
#define xbeeTP Serial4

#define TEAM_ID 1022

// CONFIG - END

#define gpsSerial Serial2

TinyGPSPlus gps;
BME280 bme;
Servo servoParachute;
Servo servoBreak;
time_t RTCTime;

bool shouldTransmit = true;
char FileC[100];
float groundAlt;
float apogee = INT_MIN;
float simPressure;

class PacketConstructor {
    String stateStr[6] = {"PRELAUNCH", "LAUNCH", "APOGEE", "PARADEPLOY", "TPDEPLOY", "LAND"};

    inline String getStateString() const {
        return stateStr[state];
    }

   public:
    unsigned long packetCount;
    char time[9] = "xx:xx:xx";
    bool isSimulation = false;
    bool payloadReleased = false;
    float altitude;
    float temp;
    float voltage;

    char gpsTime[9] = "xx:xx:xx";
    float gpsLat;
    float gpsLng;
    float gpsAlt;
    float gpsSat;

    unsigned short state;
    String lastCmd = "N/A";

    String combine() {
        return String(TEAM_ID) + "," + time + "," + packetCount + ",C," + (isSimulation ? 'S' : 'F') + "," + (payloadReleased ? 'R' : 'N') + "," + String(altitude, 2) + "," + String(temp, 2) + "," + String(voltage) + "," + String(gpsTime) + "," + String(gpsLat, 6) + "," + String(gpsLng, 6) + "," + String(gpsAlt) + "," + String(gpsSat) + "," + getStateString() + "," + lastCmd + "\r";
    }
}(packet);

class BrakeSystem {
   private:
    uint32_t startAt = -1;
    float degree = 180;

    double mapf(double x, double in_min, double in_max, double out_min, double out_max) const {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

   public:
    void start() {
        startAt = millis();
    }
    void halt() {
        startAt = -1;
    }
    void forceBreak() { degree = 180; }
    void forceRelease() { degree = 0; }
    void run() {
        if (startAt == -1) return;
        float t = (millis() - startAt) / 1000.0;
        if (t <= 11.6) {
            float t_loop = (int(t * 100) % 145) / 100.0;
            if (t_loop <= 0.1)
                degree = mapf(t_loop, 0, 0.1, 180, 0);
            else if (t_loop <= 0.65)
                degree = 0;
            else if (t_loop <= 0.95)
                degree = mapf(t_loop, 0.65, 0.95, 0, 180);
            else if (t_loop <= 1.45)
                degree = 180;
        } else if (t >= 12.6) {
            degree = 0;
            startAt = -1;
        }
        servoBreak.write(degree);
    }
}(brakeSystem);
class CommandHandler {
};

void beep(const unsigned int& count, const unsigned int& delay_ms = 100) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(BUZZER_PIN, LOW);
        delay(delay_ms);
    }
}

time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void toggleCamera() {
    digitalWrite(CAMERA_PIN, LOW);
    delay(550);
    digitalWrite(CAMERA_PIN, HIGH);
}

void setParachute(bool open) {
    packet.payloadReleased = open;
    servoParachute.write(open ? 15 : 105);
}

void getGPSData() {
    packet.gpsLat = gps.location.lat();
    packet.gpsLng = gps.location.lng();
    sprintf(packet.gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    packet.gpsAlt = gps.altitude.meters();
    packet.gpsSat = gps.satellites.value();
}

void getBMEData() {
    packet.temp = bme.getTemperature();
    packet.altitude = bme.calcAltitude(bme.getPressure()) - groundAlt;
    // if (packet.altitude < -500 || packet.altitude > 800) return;
    if (packet.altitude >= apogee) {
        apogee = packet.altitude;
    }
}

void recovery() {
    getBMEData();
    groundAlt = packet.altitude;
    beep(3);
}

void setup() {
    Serial.begin(9600);
    Serial.println("Initiating...");
    gpsSerial.begin(9600);
    xbeeGS.begin(115200);
    xbeeTP.begin(115200);

    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(CAMERA_PIN, OUTPUT);
    servoParachute.attach(SERVO_PARA_PIN);
    servoBreak.attach(SERVO_BREAK_PIN);
    pinMode(VOLTAGE_PIN, INPUT);

    digitalWrite(CAMERA_PIN, HIGH);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);

    setSyncProvider(getTeensy3Time);

    // Initiate I2C devices
    if (bme.init())
        Serial.println("✔ SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        // beep(1);
        delay(500);
    }
    if (SD.begin(10))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        // beep(5);
    }

    setParachute(false);

    delay(1000);
    recovery();
}

void stateLogic() {
    getBMEData();
    switch (packet.state) {
        // PRELAUNCH
        case 0:
            // Entry of LAUNCH state
            if (packet.altitude > 10) packet.state = 1;
            break;

        // LAUNCH
        case 1:
            // Entry of APOGEE state
            if (apogee - packet.altitude >= 10 && packet.altitude >= 670) packet.state = 2;
            break;

        // APOGEE
        case 2:
            // Entry of PARADEPLOY state
            if (packet.altitude <= 410) {
                packet.state = 3;
                setParachute(true);
            }
            break;

        // PARADEPLOY
        case 3:
            // Entry of TPDEPLOY state
            if (packet.altitude <= 310) {
                packet.state = 4;
                packet.payloadReleased = true;
                brakeSystem.start();
            }
            break;

        // TPDEPLOY
        case 4:
            // Entry of LAND state
            if (packet.altitude <= 5) {
                packet.state = 5;
                shouldTransmit = false;
                for (int i = 0; i < 5; i++) {
                    xbeeTP.print("OFF");
                    delay(100);
                }
            }
            break;

        // LAND
        case 5:
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
            delay(500);
            break;
    }
}

unsigned long lastTransmit = 0;
unsigned long lastStateLogic = 0;
void loop() {
    brakeSystem.run();
    if (millis() - lastStateLogic > 500) {
        lastStateLogic = millis();
        stateLogic();
    }
    if (shouldTransmit && millis() - lastTransmit > 1000) {
        lastTransmit = millis();
        Serial.print(packet.combine());
        xbeeGS.print(packet.combine());
        packet.packetCount++;
    }
    if (xbeeGS.available()) {
        String cmd = xbeeGS.readStringUntil('\r');
        if (cmd == "CMD,1022,CX,ON") {
            beep(1);
            packet.state = 0;
            shouldTransmit = true;
            getBMEData();
            xbeeGS.println(packet.altitude);
            groundAlt = bme.calcAltitude(bme.getPressure());
            xbeeGS.println(groundAlt);
        }
    }
}
