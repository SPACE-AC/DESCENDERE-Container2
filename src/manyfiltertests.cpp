#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <InternalTemperature.h>
#include <SD.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// *** CONFIG - START ***

#define LED1_PIN 0
#define LED2_PIN 1
#define BUZZER_PIN 3
#define CAMERA_PIN 6

#define SERVO_PARA_PIN 4
#define SERVO_BREAK_PIN 5

#define VOLTAGE_PIN 21
#define R1_OHM 2000.0F
#define R2_OHM 1250.0F

#define xbeeGS Serial3
#define xbeeTP Serial4

#define TEAM_ID 1022

// #define USE_360_FOR_PARA
// #define DEBUG

// *** CONFIG - END ***

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(String("[DEBUG] ") + x)
#define DEBUG_PRINTLN(x) Serial.println(String("[DEBUG] ") + x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define SEALEVELPRESSURE_HPA 1013.25
#define gpsSerial Serial2

#define pkgAddr 0
#define stateAddr 10
#define modeAddr 20
#define groundAltAddr 30
#define shouldTransmitAddr 40
#define shouldPollPayloadAddr 50
#define pressureOffsetAddr 60
#define tempOffsetAddr 70

TinyGPSPlus gps;
Adafruit_BME280 bme;
Servo servoParachute;
Servo servoBreak;
SimpleKalmanFilter altitudeFilter(1, 1, 0.01);
SimpleKalmanFilter altitudeTuned(1, 1, 0.08);
SimpleKalmanFilter witnessFilter(1, 1, 0.01);
SimpleKalmanFilter witnessFilterFaster(1, 1, 0.05);

int preHeight[] = {-0.06, -0.06, -0.06, 0.8, 0.8, 0.14, 0.14, 0.14, 0.06, 0.06, 0.25, 0.25, 0.25, 0.12, 0.12, 0.88, 0.88, 0.88, 0.16, 0.16, 0, 0, 0.36, 0.36, 0.36, 0.07, 0.07, -0.16, -0.16, -0.16, 0.03, 0.03, 0.02, 0.02, 0.02, 1.33, 1.33, -0.02, -0.02, -0.02, -0.17, -0.17, 0.05, 0.05, 0.01, 0.01, 0.01, 0.91, 0.91, 0.16, 0.16, 0.16, 0, 0, 0.27, 0.27, 0.02, 0.02, 0.02, 0.91, 0.91, -0.04, -0.04, -0.04, -0.02, -0.02, 0.39, 0.39, 0.39, -0.37, -0.37, 0.02, 0.02, 1.7, 1.7, 1.7, 5.27, 5.27, 12.92, 12.92, 12.92, 19.42, 19.42, 29.38, 29.38, 29.38, 36.53, 36.53, 39.02, 39.02, 39.02, 46.53, 46.53, 50.25, 50.25, 55.55, 55.55, 55.55, 60.05, 60.05, 65.19, 65.19, 65.19, 71.77, 71.77, 76.05, 76.05, 76.05, 80.85, 80.85, 85.18, 85.18, 85.18, 90.42, 90.42, 94.37, 94.37, 98.04, 98.04, 98.04, 101.99, 101.99, 105.92, 105.92, 105.92, 109.15, 109.15, 112.72, 112.72, 112.72, 116.85, 116.85, 119.99, 119.99, 125.36, 125.36, 125.36, 127.22, 127.22, 130.74, 130.74, 130.74, 134.94, 134.94, 138.49, 138.49, 138.49, 143.38, 143.38, 144.14, 144.14, 146.79, 146.79, 146.79, 150.77, 150.77, 153.22, 153.22, 153.22, 155.85, 155.85, 158.56, 158.56, 158.56, 160.91, 160.91, 163.75, 163.75, 163.75, 165.69, 165.69, 167.97, 167.97, 170.05, 170.05, 170.05, 172.02, 172.02, 175.89, 175.89, 175.89, 176.2, 176.2, 177.89, 177.89, 177.89, 179.86, 179.86, 181.21, 181.21, 184.08, 184.08, 184.08, 184.61, 184.61, 185.93, 185.93, 185.93, 187.31, 187.31, 188.64, 188.64, 188.64, 189.36, 189.36, 190.68, 190.68, 191.59, 191.59, 191.59, 193.35, 193.35, 193.75, 193.75, 193.75, 194.24, 194.24, 195.32, 195.32, 195.32, 195.65, 195.65, 197.26, 197.26, 196.46, 196.46, 196.46, 196.42, 196.42, 197.07, 197.07, 197.07, 197.05, 197.05, 198.27, 198.27, 198.27, 197.26, 197.26, 197.48, 197.48, 197.48, 197.17, 197.17, 196.93, 196.93, 197.86, 197.86, 197.86, 196.52, 196.52, 195.79, 195.79, 195.79, 428.17, 428.17, 193.46, 193.46, 193.46, 194.6, 194.6, 194.74, 194.74, 194.74, 194.64, 194.64, 194.42, 194.42, 193.59, 193.25, 193.25, 191.45, 191.45, 190.47, 190.47, 190.47, 184.64, 183.55, 183.55, 254.27, 253.76, 253.76, 209.01, 209.01, 189.21, 189.21, 189.21, 176.72, 176.72, 178.18, 178.18, 178.18, 177.37, 177.37, 173.31, 173.31, 173.31, 166.26, 166.26, 167.32, 167.32, 167.32, 165.01, 165.01, 162.26, 162.26, 158.18, 158.18, 157.47, 157.47, 157.47, 156.74, 156.74, 156.36, 156.36, 156.36, 155.41, 155.41, 154.28, 154.28, 154.28, 153.08, 153.08, 150.88, 150.88, 150.88, 149.18, 149.18, 148.94, 148.94, 148.94, 144.74, 144.74, 141.69, 141.69, 141.69, 141.72, 141.72, 140.73, 140.73, 138.88, 138.88, 138.88, 136.49, 136.49, 137.09, 123.06, 123.06, 122.89, 122.89, 121.29, 121.29, 121.29, 119.17, 119.17, 117.58, 117.58, 117.58, 116.17, 116.17, 114.95, 114.95, 113.21, 113.21, 113.21, 112.11, 112.11, 111.25, 111.25, 111.25, 109.28, 109.28, 108.21, 108.21, 108.21, 106.53, 106.53, 104.58, 104.58, 103.51, 103.51, 103.51, 100.97, 100.97, 99.79, 99.79, 99.79, 99.15, 99.15, 97.41, 97.41, 97.41, 95.8, 95.8, 95.79, 95.79, 93.65, 93.65, 93.65, 93.1, 93.1, 90.2, 90.2, 90.2, 89.42, 89.42, 88.07, 88.07, 88.07, 86.21, 86.21, 84.81, 84.81, 84.79, 84.79, 84.79, 82.18, 82.18, 79.75, 79.75, 79.75, 78.77, 78.77, 80.22, 80.22, 80.22, 77.91, 77.91, 76.98, 76.98, 76.71, 76.71, 76.71, 74.83, 74.83, 74.8, 74.8, 74.8, 71.11, 71.11, 70.09, 53.92, 52.22, 52.22, 52.22, 50.9, 50.9, 48.71, 48.71, 48.71, 49.23, 49.23, 48.41, 48.41, 48.41, 46.11, 46.11, 46.81, 46.81, 42.5, 42.5, 42.5, 40.71, 40.71, 40.94, 40.94, 40.94, 41.8, 41.8, 40.95, 40.95, 40.95, 39.06, 39.06, 37.47, 37.47, 37.47, 35.69, 35.69, 35.44, 35.44, 35.44, 32.64, 32.64, 29.7, 29.7, 28.46, 28.46, 28.46, 27.52, 27.52, 25.4, 25.4, 25.4, 24.07, 24.07, 22.63, 22.63, 22.63, 21.01, 21.01, 20.7, 20.7, 20.7, 19.09, 19.09, 6, 6, 6.42, 6.42, 6.42, 4.76, 4.76, 3.64, 3.64, 3.64, -2.5, -3.28, -3.28, -3.28, -5.54, -2.31, -2.27, -2.27, -2.27, -1.44, -2.08, -2.26, -2.26, -1.28, -1.28, -2.39, -2.18, -2.18, -0.97, -0.97, -2.38, -2.19, -2.19, -1.34, -1.34, -2.31, -2.31, -2.31, -0.48, -0.48, -2.21, -2.21, -2.21, -0.97, -0.97, -2.19, -2.19, -2.47, -1.17, -1.17, -2.4, -2.4, -1.15, -1.15, -1.15, -2.39, -2.39, -2.42, -2.42, -2.42, -2.39, -2.39, -2.63, -2.63, -1.48, -2.32, -2.32, -2.5, -2.5, -1.77, -2.53, -2.53, -2.35, -2.35, -1.3, -2.15, -2.15, -2.15, -2.15, -1.65, -2, -2.17, -2.17, -2.17, -1.55, -1.65, -1.59, -1.59, -1.59, -1.38, -1.44, -1.35, -1.35, -1.35, -1.3, -1.33, -1.07, -1.07, -1, -1, -0.95};

bool shouldTransmit, shouldPollPayload = false, shouldSendCustom = false;
float groundAlt;
float apogee = INT_MIN;
bool isSimulation = false;
float pressureOffset, tempOffset;

unsigned long lastTransmit = 0, lastDoStateLogic = 0, lastPoll = 0, lastKradik = 0, lastSD = 0;
bool isParachuteOn = false;
bool kradikBool = false;

char cFileName[100], cDebugFileName[100], pFileName[100];
File cFile, cDebugFile, pFile;

class PacketConstructor {
   private:
    String stateStr[6] = {"PRELAUNCH", "LAUNCH", "APOGEE", "PARADEPLOY", "TPDEPLOY", "LAND"};

   public:
    unsigned long packetCount;
    char time[9] = "xx:xx:xx";
    bool payloadReleased = false;
    float altitude;
    float temp;
    float voltage;
    char gpsTime[9] = "xx:xx:xx";
    float gpsLat;
    float gpsLng;
    float gpsAlt;
    short gpsSat;
    short state;
    String lastCmd = "CXON";

    inline String getStateString() const {
        if (state < 0 || state > 5) {
            return "UNKNOWN";
        }
        return stateStr[state];
    }

    void reset() {
        packetCount = 0;
        isSimulation = false;
        payloadReleased = false;
        state = 0;
    }

    String combine() {
        return String(TEAM_ID) + "," + time + "," + packetCount + ",C," + (isSimulation ? 'S' : 'F') + "," + (payloadReleased ? 'R' : 'N') + "," + String(altitude, 2) + "," + String(temp, 2) + "," + String(voltage, 2) + "," + String(gpsTime) + "," + String(gpsLat, 6) + "," + String(gpsLng, 6) + "," + String(gpsAlt) + "," + String(gpsSat) + "," + getStateString() + "," + lastCmd + "\r";
    }

    String combineCustom() {
        return String(TEAM_ID) + "," + time + "," + packetCount + ",X," + String(InternalTemperature.readTemperatureC()) + "\r";
    }

    String getCustomHeader() const {
        return String(TEAM_ID) + "," + time + "," + packetCount + ",X,";
    }
}(packet);

class BrakeSystem {
   private:
    long startAt = -1;
    float degree = 180;
    short mode = 1;
    unsigned short count = 0;
    uint32_t fastModeDelay;

    double mapf(double x, double in_min, double in_max, double out_min, double out_max) const {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

   public:
    void start() {
        startAt = millis();
        fastModeDelay = 0;
        count = 0;
        if (mode == 2) degree = 0;
    }
    void halt() {
        startAt = -1;
    }
    void forceBreak() {
        degree = 180;
        servoBreak.write(degree);
    }
    void forceRelease() {
        degree = 0;
        servoBreak.write(degree);
    }
    void setMode(short newMode) {
        mode = newMode;
    }
    void run() {
        if (startAt != -1) {
            float t = (millis() - startAt) / 1000.0;
            if (mode == 0) {
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
            } else if (mode == 1) {
                if (count < 50) {
                    if (millis() - fastModeDelay > 500) {
                        fastModeDelay = millis();
                        degree = degree == 0 ? 180 : 0;
                        count++;
                    }
                } else if (count < 80) {
                    if (millis() - fastModeDelay > 200) {
                        fastModeDelay = millis();
                        degree = degree == 0 ? 180 : 0;
                        count++;
                    }
                } else {
                    degree = 0;
                    startAt = -1;
                }
            } else if (mode == 2) {
                if (t <= 2 * 5) {
                    float t_loop = fmod(t, 2);
                    degree = mapf(t_loop, 0, 2, 0, 180);
                } else {
                    degree = 0;
                    startAt = -1;
                }
            } else if (mode == 3) {
                if (t <= 15.2) {
                    // float t_loop = (int(t * 100) % 145) / 100.0;
                    float t_loop = fmod(t, 1.9);
                    if (t_loop <= 0.1)
                        degree = mapf(t_loop, 0, 0.1, 180, 0);
                    else if (t_loop <= 1.1)
                        degree = 0;
                    else if (t_loop <= 1.4)
                        degree = mapf(t_loop, 1.1, 1.4, 0, 180);
                    else if (t_loop <= 1.9)
                        degree = 180;
                } else if (t >= 16.2 && t <= 26.2 && millis() - fastModeDelay > 200) {
                    fastModeDelay = millis();
                    degree = degree == 0 ? 180 : 0;
                } else if (t >= 26.2) {
                    degree = 0;
                    startAt = -1;
                }
            } else if (mode == 4) {
                if (t <= 15.2) {
                    // float t_loop = (int(t * 100) % 145) / 100.0;
                    float t_loop = fmod(t, 1.9);
                    if (t_loop <= 0.1)
                        degree = mapf(t_loop, 0, 0.1, 120, 0);
                    else if (t_loop <= 1.1)
                        degree = 0;
                    else if (t_loop <= 1.4)
                        degree = mapf(t_loop, 1.1, 1.4, 0, 120);
                    else if (t_loop <= 1.9)
                        degree = 120;
                } else if (t >= 16.2 && t <= 26.2 && millis() - fastModeDelay > 200) {
                    fastModeDelay = millis();
                    degree = degree == 0 ? 180 : 0;
                } else if (t >= 26.2) {
                    degree = 0;
                    startAt = -1;
                }
            }
        }
        servoBreak.write(degree);
    }
}(brakeSystem);

class SimulationHandler {
   private:
    bool simEnabled = false, simActivated = false, firstData = false;
    int simPressure;

   public:
    void enable() {
        simEnabled = true;
        simActivated = false;
    }
    // @return Whether simulation mode was previously enabled.
    bool activate() {
        if (!simEnabled) return false;
        simActivated = true;
        isSimulation = true;
        EEPROM.update(modeAddr, isSimulation);
        firstData = true;
        return true;
    }
    void disable() {
        simEnabled = false;
        simActivated = false;
        isSimulation = false;
        EEPROM.update(modeAddr, isSimulation);
    }
    // @param pressure Pressure in Pascals
    void setPressure(int pressure) {
        simPressure = pressure;
        if (firstData) {
            groundAlt = calcAltitude();
            EEPROM.update(groundAltAddr, groundAlt);
            firstData = false;
        }
    }
    int getPressure() const {
        return simPressure;
    }
    float calcAltitude() {
        float A = simPressure / 101325;
        float B = 1 / 5.25588;
        float C = pow(A, B);
        C = 1.0 - C;
        C = C / 0.0000225577;
        return C;
    }
}(simHandler);

time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void beep(const unsigned int& count, const unsigned int& delay_ms = 100) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(BUZZER_PIN, LOW);
        delay(delay_ms);
    }
}

void toggleCamera() {
    xbeeGS.print("Toggling camera\r");
    // return;
    digitalWrite(CAMERA_PIN, LOW);
    delay(600);
    digitalWrite(CAMERA_PIN, HIGH);
    xbeeGS.print("Camera toggled\r");
}

#ifdef USE_360_FOR_PARA
void setParachute(bool open) {
    // PROCEED TO CLOSE
    if (!open && isParachuteOn) {
        servoParachute.write(100);
        delay(50);
        servoParachute.write(90);
    }
    // PROCEED TO OPEN
    if (open && !isParachuteOn) {
        servoParachute.write(80);
        delay(50);
        servoParachute.write(90);
    }
    isParachuteOn = open;
}
#else
void setParachute(bool open) {
    servoParachute.write(open ? 90 : 0);
    isParachuteOn = open;
}
#endif

void getGPSData() {
    packet.gpsLat = gps.location.lat();
    packet.gpsLng = gps.location.lng();
    sprintf(packet.gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    packet.gpsAlt = gps.altitude.meters();
    packet.gpsSat = gps.satellites.value();
}

short bcount = 0;
float lastChosen = 0, lastFiltered = 0, lastFastWitnessed = 0;
int hCount = 0;
void getBMEData() {
    packet.temp = bme.readTemperature() + tempOffset;

    float sensorAlt = bme.readAltitude(SEALEVELPRESSURE_HPA) + pressureOffset;
    // float sensorAlt;
    // if (hCount >= 607)
    //     sensorAlt = 0;
    // else
    //     sensorAlt = preHeight[hCount++] + groundAlt;

    bcount++;
    if (bcount > 100) {
        sensorAlt += 15;
        if (bcount > 105) bcount = 0;
    }

    Serial.print(sensorAlt - groundAlt);
    Serial.print(",");
    Serial.print(lastChosen);
    // if (!cDebugFile) cDebugFile = SD.open(cDebugFileName, FILE_WRITE);
    // if (cDebugFile) {
    //     cDebugFile.print(packet.time);
    //     cDebugFile.print(" ");
    //     cDebugFile.print(packet.altitude);
    // }
    // sensorAlt = altitudeFilter.updateEstimate(sensorAlt) + pressureOffset;
    // Serial.print(",");
    Serial.print(",");
    Serial.print(altitudeFilter.updateEstimate(sensorAlt) - groundAlt);
    Serial.print(",");
    Serial.print(lastFiltered);
    Serial.print(",");
    Serial.print(lastFastWitnessed);
    Serial.print(",");
    Serial.println(altitudeTuned.updateEstimate(sensorAlt) - groundAlt);

    // Serial.println(sensorAlt);
    // if (cDebugFile) {
    //     cDebugFile.print(", ");
    //     cDebugFile.println(packet.altitude);
    //     cDebugFile.flush();
    // }
    packet.altitude = (isSimulation ? simHandler.calcAltitude() : sensorAlt) - groundAlt;
    // if (packet.altitude < -500 || packet.altitude > 800) return;
    if (packet.altitude >= apogee) {
        apogee = packet.altitude;
    }
}

void getBattery() {
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    packet.voltage = apparentVoltage * ((R1_OHM + R2_OHM) / R2_OHM);
    // if (packet.voltage < 5.3 && !Serial) {
    //     beep(5, 25);
    // }
}

void recovery() {
    Serial.println("\nRecovering...");
    packet.packetCount = EEPROM.read(pkgAddr);
    packet.state = EEPROM.read(stateAddr);
    isSimulation = EEPROM.read(modeAddr);
    EEPROM.get(groundAltAddr, groundAlt);
    shouldTransmit = EEPROM.read(shouldTransmitAddr);
    shouldPollPayload = EEPROM.read(shouldPollPayloadAddr);
    EEPROM.get(pressureOffsetAddr, pressureOffset);
    EEPROM.get(tempOffsetAddr, tempOffset);
    // packet.lastCmd = EEPROM.read(lastCmdAddr); // later krub

    Serial.println("Simulation Mode? " + String(isSimulation ? "Yes" : "No"));
    Serial.println("Should Transmit? " + String(shouldTransmit ? "Yes" : "No"));
    Serial.println("Should Poll Payload? " + String(shouldPollPayload ? "Yes" : "No"));
    Serial.println("Packet Count: " + String(packet.packetCount));
    Serial.println("State: " + packet.getStateString());
    Serial.println("Ground Altitude: " + String(groundAlt));
    Serial.println("Pressure Offset: " + String(pressureOffset));
    Serial.println("Temperature Offset: " + String(tempOffset));
    // Serial.println("Last Command: " + packet.lastCmd);
    Serial.println();

    sprintf(packet.time, "%02d:%02d:%02d", hour(), minute(), second());
    xbeeGS.print(packet.getCustomHeader() + isSimulation + "," + shouldTransmit + "," + shouldPollPayload + "," + packet.packetCount + "," + packet.getStateString() + "," + groundAlt + "," + pressureOffset + "," + tempOffset + "\r");
    Serial.print(packet.getCustomHeader() + isSimulation + "," + shouldTransmit + "," + shouldPollPayload + "," + packet.packetCount + "," + packet.getStateString() + "," + groundAlt + "," + pressureOffset + "," + tempOffset + "\n");

    DEBUG_PRINTLN("declaring file index");
    int fileIndex = 0;
    do {
        DEBUG_PRINTLN("incrementing file index");
        fileIndex++;
        String("C_" + String(fileIndex) + ".csv").toCharArray(cFileName, 100);
    } while (SD.exists(cFileName));
    DEBUG_PRINT("got file index: ");
    DEBUG_PRINTLN(fileIndex);
    String("C_DEBUG_" + String(fileIndex) + ".csv").toCharArray(cDebugFileName, 100);
    String("TP_" + String(fileIndex) + ".csv").toCharArray(pFileName, 100);
    Serial.print("Selected file name: ");
    Serial.print(cFileName);
    Serial.print(" and ");
    Serial.println(pFileName);
    DEBUG_PRINTLN("Opening file");
    File file = SD.open(cDebugFileName, FILE_WRITE);
    if (file) {
        file.println("Recovery Successful!");
        file.println("Simulation Mode? " + String(isSimulation ? "Yes" : "No"));
        file.println("Should Transmit? " + String(shouldTransmit ? "Yes" : "No"));
        file.println("Packet Count: " + packet.packetCount);
        file.println("State: " + packet.getStateString());
        file.println("Ground Altitude: " + String(groundAlt));

        file.println("TEAM_ID,MISSION_TIME,PACKET_COUNT,PACKET_TYPE,MODE,TP_RELEASED,ALTITUDE,TEMP,VOLTAGE,GPS_TIME,GPS_LATITUDE,GPS_LONGITUDE,GPS_ALTITUDE,GPS_SATS,SOFTWARE_STATE,CMD_ECHO");
        file.flush();
    }
    DEBUG_PRINTLN("wrote file and setting cam");

    digitalWrite(CAMERA_PIN, HIGH);
    DEBUG_PRINTLN("beeping 3 times");

    for (int i = 0; i < 100; i++) {
        getBMEData();
        delay(5);
    }
    beep(3);
}

void setup() {
    Serial.begin(9600);
    Serial.println("Initiating...");
    gpsSerial.begin(9600);
    xbeeGS.begin(115200);
    xbeeTP.begin(115200);
    DEBUG_PRINTLN("Serials done.");

    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(CAMERA_PIN, OUTPUT);
    servoParachute.attach(SERVO_PARA_PIN);
    servoBreak.attach(SERVO_BREAK_PIN);
    DEBUG_PRINTLN("Closing parachute");
    setParachute(false);
    brakeSystem.forceBreak();
    pinMode(VOLTAGE_PIN, INPUT);
    DEBUG_PRINTLN("Pins done.");

    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);

    DEBUG_PRINTLN("Setting teensy time provider");
    setSyncProvider(getTeensy3Time);

    // Initiate I2C devices
    DEBUG_PRINTLN("Initiating I2C devices");
    if (bme.begin(0x76))
        Serial.println("✔ SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        beep(1);
        delay(500);
    }
    if (SD.begin(10))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        beep(5);
    }

    DEBUG_PRINTLN("delaying 1 sec");
    delay(1000);
    DEBUG_PRINTLN("gonna recover");
    recovery();
}

const unsigned int witnessCount = 5;

uint32_t startPayloadDeployAt;
bool isCamOff = true;
unsigned short witnessRound = 0;
float witnessValues[witnessCount];
float witnessSum = 0;
bool hasSentLanded = false;
void stateLogic() {
    getBMEData();
    getGPSData();

    if (witnessRound > witnessCount - 1) {
        float average = witnessSum / witnessCount;
        float leastDelta = INT_MAX, chosenAlt;
        for (int i = 0; i < witnessCount; i++) {
            if (abs(witnessValues[i] - average) < leastDelta) {
                leastDelta = abs(witnessValues[i] - average);
                chosenAlt = witnessValues[i];
            }
        }
        witnessRound = 0;
        witnessSum = 0;
        // Serial.print(packet.altitude);
        // Serial.print(",");
        // Serial.print(chosenAlt);
        lastChosen = chosenAlt;
        lastFiltered = witnessFilter.updateEstimate(chosenAlt);
        lastFastWitnessed = witnessFilterFaster.updateEstimate(chosenAlt);

        chosenAlt = lastFastWitnessed;
        // Serial.print(",");
        // Serial.println(chosenAlt);
        // if (!cDebugFile) cDebugFile = SD.open(cDebugFileName, FILE_WRITE);
        // if (cDebugFile) {
        //     cDebugFile.print(packet.time);
        //     cDebugFile.print(" Chosen Altitude: ");
        //     cDebugFile.println(chosenAlt);
        //     cDebugFile.flush();
        // }
        switch (packet.state) {
            // PRELAUNCH
            case 0:
                // Entry of LAUNCH state
                if (chosenAlt > 20) packet.state = 1;
                break;

            // LAUNCH
            case 1:
                // Entry of APOGEE state
                if (chosenAlt >= 670 || apogee - chosenAlt >= 20) packet.state = 2;  // testing: (apogee - chosenAlt >= 10 && chosenAlt >= 60)
                break;

            // APOGEE
            case 2:
                // Entry of PARADEPLOY state
                if (chosenAlt <= 410) {  // testing: (chosenAlt <= apogee - 15)
                    packet.state = 3;
                    setParachute(true);
                }
                break;

            // PARADEPLOY
            case 3:
                // Entry of TPDEPLOY state
                if (chosenAlt <= 310) {
                    packet.state = 4;
                    shouldPollPayload = true;
                    lastPoll = lastTransmit + 125;
                    startPayloadDeployAt = millis();
                    isCamOff = false;
                    toggleCamera();
                    brakeSystem.start();
                    packet.payloadReleased = true;
                    for (int i = 0; i < 5; i++) {
                        xbeeTP.print("ON\r\r\r");
                        delay(50);
                    }
                    EEPROM.update(shouldPollPayloadAddr, shouldPollPayload);
                }
                break;

            // TPDEPLOY
            case 4:
                if (!isCamOff && millis() - startPayloadDeployAt > 20000) {
                    toggleCamera();
                    isCamOff = true;
                }
                // Entry of LAND state
                if (chosenAlt <= 5) {
                    packet.state = 5;
                    hasSentLanded = false;
                    for (int i = 0; i < 5; i++) {
                        xbeeTP.print("OFF\r\r\r");
                        delay(50);
                    }
                }
                break;

            // LAND
            case 5:
                if (!hasSentLanded) {
                    shouldTransmit = false;
                    hasSentLanded = true;
                }
                digitalWrite(BUZZER_PIN, HIGH);
                delay(500);
                digitalWrite(BUZZER_PIN, LOW);
                delay(500);
                break;
        }
    }
    witnessValues[witnessRound++] = packet.altitude;
    witnessSum += packet.altitude;

    EEPROM.update(stateAddr, packet.state);
}

uint32_t lastCommandAt = 0;
void doCommand(String cmd) {
    packet.lastCmd = cmd.substring(0, cmd.indexOf(",")) + cmd.substring(cmd.indexOf(",") + 1);
    // EEPROM.update(lastCmdAddr, packet.lastCmd); // later krub
    if (millis() - lastCommandAt < 1000) return;
    lastCommandAt = millis();
    if (cmd == "CX,ON") {
        beep(2);
        shouldTransmit = true;
        // groundAlt = bme.readAltitude(SEALEVELPRESSURE_HPA) + pressureOffset;
        // altitudeFilter = SimpleKalmanFilter(1, 1, 0.01);
        groundAlt = lastFiltered + groundAlt;
        setParachute(false);
        packet.payloadReleased = false;
        EEPROM.update(modeAddr, isSimulation);
        Serial.println(groundAlt);
        EEPROM.put(groundAltAddr, groundAlt);
        EEPROM.update(shouldTransmitAddr, shouldTransmit);
        EEPROM.update(pkgAddr, packet.packetCount);
        EEPROM.update(stateAddr, packet.state);
        brakeSystem.halt();
        brakeSystem.forceBreak();
        packet.reset();
        isCamOff = true;
        for (int i = 0; i < 5; i++) {
            xbeeTP.print("ALT\r\r\r");
            delay(50);
        }
    } else if (cmd == "CX,OFF") {
        beep(1);
        shouldTransmit = false;
        EEPROM.update(shouldTransmitAddr, shouldTransmit);
    } else if (cmd == "SIM,ENABLE") {
        simHandler.enable();
    } else if (cmd == "SIM,ACTIVATE") {
        if (!simHandler.activate())
            Serial.println("Not enabled, ignoring command.");

    } else if (cmd == "SIM,DISABLE") {
        simHandler.disable();
    } else if (cmd.startsWith("SIMP,")) {
        simHandler.setPressure(cmd.substring(5).toInt());
    } else if (cmd.startsWith("ST,")) {
        String hr = cmd.substring(3, 5);
        String min = cmd.substring(6, 8);
        String sec = cmd.substring(9, 11);
        setTime(hr.toInt(), min.toInt(), sec.toInt(), day(), month(), year());
        xbeeTP.print("ST," + hr + "," + min + "," + sec + "\r\r\r");
    }
    // Custom commands
    else if (cmd == "FORCE,PARADEPLOY")
        setParachute(true);
    else if (cmd == "FORCE,SEQUENCE")
        brakeSystem.start();
    else if (cmd == "FORCE,HALT")
        brakeSystem.halt();
    else if (cmd == "FORCE,RELEASE")
        brakeSystem.forceRelease();
    else if (cmd == "FORCE,BREAK")
        brakeSystem.forceBreak();
    else if (cmd == "FORCE,POLL")
        xbeeTP.print("POLL\r\r\r");
    else if (cmd == "FORCE,POLLON") {
        shouldPollPayload = true;
        EEPROM.update(shouldPollPayloadAddr, shouldPollPayload);
    } else if (cmd == "FORCE,POLLOFF") {
        shouldPollPayload = false;
        EEPROM.update(shouldPollPayloadAddr, shouldPollPayload);
    } else if (cmd == "FORCE,TPON")
        for (int i = 0; i < 5; i++) {
            xbeeTP.print("ON\r\r\r");
            delay(50);
        }
    else if (cmd == "FORCE,TPOFF")
        for (int i = 0; i < 5; i++) {
            xbeeTP.print("OFF\r\r\r");
            delay(50);
        }
    else if (cmd == "FORCE,CCAM")
        toggleCamera();
    else if (cmd == "FORCE,SENDCUSTOM")
        shouldSendCustom = !shouldSendCustom;
    else if (cmd.startsWith("CALPRES,")) {
        pressureOffset = cmd.substring(8).toFloat();
        EEPROM.put(pressureOffsetAddr, pressureOffset);
        Serial.println(pressureOffset);
        float offset;
        EEPROM.get(pressureOffsetAddr, offset);
        Serial.println(offset);
    } else if (cmd.startsWith("CALTEMP,")) {
        tempOffset = cmd.substring(8).toFloat();
        EEPROM.put(tempOffsetAddr, tempOffset);
        Serial.println(tempOffset);
        float offset;
        EEPROM.get(tempOffsetAddr, offset);
        Serial.println(offset);
    } else if (cmd.startsWith("FORCE,STATE"))
        packet.state = cmd.substring(11).toInt();
    else if (cmd.startsWith("FORCE,MODE"))
        brakeSystem.setMode(cmd.substring(10).toInt());
    else {
        xbeeGS.print(packet.getCustomHeader() + "UNK" + "," + cmd + "\r");
    }

    if (!cDebugFile) cDebugFile = SD.open(cDebugFileName, FILE_WRITE);
    if (cDebugFile) {
        cDebugFile.print(packet.time);
        cDebugFile.println(" CMD: " + cmd);
        cDebugFile.flush();
    }
}

void loop() {
    // Read serial inputs
    while (Serial2.available())
        gps.encode(Serial2.read());

    if (xbeeTP.available()) {
        // DEBUG_PRINTLN("xbeeTP available");
        String in = xbeeTP.readStringUntil('$');
        xbeeGS.print(in + '\r');
        // Serial.println(in);
        // if (!pFile) pFile = SD.open(pFileName, FILE_WRITE);
        // if (pFile) {
        //     pFile.print(in + '\r');
        //     pFile.flush();
        // }
        // char c = xbeeTP.read();
        // Serial.print(c);
        // xbeeTP.print(c);
        // if (c == '$') Serial.println();
    }

    if (Serial.available()) {
        DEBUG_PRINTLN("Serial available");
        beep(1);
        while (Serial.available()) {
            const String cmd = Serial.readStringUntil('$');
            if (cmd == "\n") return;
            if (cmd.length() >= 10)
                doCommand(cmd.substring(9));
        }
    }
    if (xbeeGS.available()) {
        DEBUG_PRINTLN("xbeeGS available");
        beep(1);
        while (xbeeGS.available()) {
            const String cmd = xbeeGS.readStringUntil('\r');
            if (cmd == "\r") return;
            if (cmd.length() >= 10)
                doCommand(cmd.substring(9));
        }
    }

    // Poll payload
    if (shouldPollPayload && millis() - lastPoll >= 250) {
        DEBUG_PRINTLN("Polling payload");
        lastPoll = millis();
        xbeeTP.print("POLL\r\r\r");
    }
    brakeSystem.run();
    if (millis() - lastDoStateLogic >= 40) {
        lastDoStateLogic = millis();
        sprintf(packet.time, "%02d:%02d:%02d", hour(), minute(), second());
        getBattery();
        stateLogic();
    }
    if (shouldTransmit && millis() - lastTransmit >= 1000) {
        lastTransmit = millis();
        // Serial.println(packet.combine());
        // Serial.println(InternalTemperature.readTemperatureC());
        xbeeGS.print(packet.combine());
        packet.packetCount++;
        EEPROM.update(pkgAddr, packet.packetCount);

        if (shouldSendCustom) {
            Serial.println(packet.combineCustom());
            xbeeGS.print(packet.combineCustom());
        }
        // Serial.println(packet.combine());
        if (!cFile) cFile = SD.open(cFileName, FILE_WRITE);
        if (cFile) {
            cFile.print(packet.combine());
            cFile.flush();
        }
    }
    if (shouldTransmit && millis() - lastSD >= 100) {
        lastSD = millis();
        if (!cDebugFile) cDebugFile = SD.open(cDebugFileName, FILE_WRITE);
        if (cDebugFile) {
            cDebugFile.print(String(packet.time) + "," + String(packet.packetCount) + "," + String(packet.altitude, 2) + "," + String(bme.readAltitude(SEALEVELPRESSURE_HPA)) + "," + String(packet.temp));
            cDebugFile.flush();
        }
    }

#ifndef USE_360_FOR_PARA
    if (millis() - lastKradik > 400 && isParachuteOn) {
        lastKradik = millis();
        servoParachute.write(kradikBool ? 70 : 97);
        kradikBool = !kradikBool;
    }
#endif
}