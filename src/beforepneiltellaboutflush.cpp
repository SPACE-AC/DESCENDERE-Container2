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

#define SD_WRITE_INTERVAL_MS 1000
// #define USE_360_FOR_PARA
#define DEBUG

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
#define lastCmdAddr 60

TinyGPSPlus gps;
Adafruit_BME280 bme;
Servo servoParachute;
Servo servoBreak;
SimpleKalmanFilter altitudeFilter(1, 1, 0.01);

bool shouldTransmit, shouldPollPayload = false, shouldSendCustom = false;
float groundAlt;
float apogee = INT_MIN;
bool isSimulation = false;

unsigned long lastTransmit = 0, lastDoStateLogic = 0, lastPoll = 0, lastKradik = 0;
bool isParachuteOn = false;
bool kradikBool = false;

char cFileName[100], cDebugFileName[100], pFileName[100];

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
        return String(TEAM_ID) + "," + time + "," + packetCount + ",C," + (isSimulation ? 'S' : 'F') + "," + (payloadReleased ? 'R' : 'N') + "," + String(altitude, 2) + "," + String(temp, 2) + "," + String(voltage) + "," + String(gpsTime) + "," + String(gpsLat, 6) + "," + String(gpsLng, 6) + "," + String(gpsAlt) + "," + String(gpsSat) + "," + getStateString() + "," + lastCmd + "\r";
    }

    String combineCustom() {
        return String(TEAM_ID) + "," + time + "," + packetCount + ",X," + String(InternalTemperature.readTemperatureC()) + "\r";
    }
}(packet);

class BrakeSystem {
   private:
    long startAt = -1;
    float degree = 180;
    short mode = 0;
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
                if (millis() - fastModeDelay > 500) {
                    if (count++ < 30) {
                        fastModeDelay = millis();
                        degree = degree == 0 ? 180 : 0;
                    } else {
                        degree = 0;
                        startAt = -1;
                    }
                }
            } else if (mode == 2) {
                if (t <= 10) {
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
                } else if (t >= 16.2) {
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
                } else if (t >= 16.2) {
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

class SDWriter {
   private:
    // bool isWriting = false;
    String buffer = "";
    uint32_t lastWrite = 0;
    const char* filename;

   public:
    SDWriter() {}
    SDWriter(const char* filename) : filename(filename) {}

    void setFileName(const char* filename) {
        this->filename = filename;
    }

    void write(const String& data) {
        // if (isWriting) return;
        // isWriting = true;
        // this->data = data;
        buffer += data;
    }

    // aliased to write()
    void print(const String& data) {
        write(data);
    }

    void println(const String& data) {
        write(data + "\n");
    }

    // Call this every loop
    void update() {
        // if (!isWriting) return;
        if (buffer.length() <= 0) return;
        if (millis() - lastWrite > SD_WRITE_INTERVAL_MS) {
            lastWrite = millis();
            File file = SD.open(filename, FILE_WRITE);
            if (file) {
                file.print(buffer);
                file.close();
            }
            buffer = "";
            // isWriting = false;
        }
    }
};

SDWriter containerWriter;
SDWriter debugWriter;
SDWriter payloadWriter;

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

// short bcount = 0;
void getBMEData() {
    packet.temp = bme.readTemperature();
    packet.altitude = isSimulation ? simHandler.calcAltitude() : (bme.readAltitude(SEALEVELPRESSURE_HPA) - groundAlt);
    // bcount++;
    // if (bcount > 100) {
    //     packet.altitude += 10000;
    //     if (bcount > 101) bcount = 0;
    // }
    // File file = SD.open(cDebugFileName, FILE_WRITE);
    debugWriter.print(packet.time);
    // Serial.print(packet.time);
    debugWriter.print(" Measured Alt: ");
    // Serial.print(" Measured Alt: ");
    debugWriter.print(packet.altitude);
    // Serial.print(packet.altitude);
    packet.altitude = altitudeFilter.updateEstimate(packet.altitude);
    debugWriter.print(", Estimated Alt: ");
    // Serial.print(",");
    debugWriter.println(packet.altitude);
    // Serial.println(packet.altitude);
    // file.close();
    // if (packet.altitude < -500 || packet.altitude > 800) return;
    if (packet.altitude >= apogee) {
        apogee = packet.altitude;
    }
}

void getBattery() {
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    packet.voltage = apparentVoltage * ((R1_OHM + R2_OHM) / R2_OHM);
    if (packet.voltage < 5.3 && !Serial) {
        beep(5, 25);
    }
}

// void writeMainSD(const String& data) {
// }

// void writeDebugSD(const String& data) {
//     File file = SD.open(cDebugFileName, FILE_WRITE);
//     if (file) {
//         file.print(data);
//         file.close();
//     }
// }

// void intervalTasks() {
//     static String cbuffer = "";
//     cbuffer += data;
//     static uint32_t lastWriteMain = 0;
//     if (millis() - lastWriteMain > SD_WRITE_INTERVAL_MS) {
//         File file = SD.open(cFileName, FILE_WRITE);
//         if (file) {
//             file.print(data);
//             file.close();
//         }
//         lastWriteMain = millis();
//     }
// }

void recovery() {
    Serial.println("\nRecovering...");
    packet.packetCount = EEPROM.read(pkgAddr);
    packet.state = EEPROM.read(stateAddr);
    isSimulation = EEPROM.read(modeAddr);
    groundAlt = EEPROM.read(groundAltAddr);
    shouldTransmit = EEPROM.read(shouldTransmitAddr);
    shouldPollPayload = EEPROM.read(shouldPollPayloadAddr);
    // packet.lastCmd = EEPROM.read(lastCmdAddr); // later krub

    Serial.println("Simulation Mode? " + String(isSimulation ? "Yes" : "No"));
    Serial.println("Should Transmit? " + String(shouldTransmit ? "Yes" : "No"));
    Serial.println("Should Poll Payload? " + String(shouldPollPayload ? "Yes" : "No"));
    Serial.println("Packet Count: " + String(packet.packetCount));
    Serial.println("State: " + packet.getStateString());
    Serial.println("Ground Altitude: " + String(groundAlt));
    Serial.println("Last Command: " + packet.lastCmd);
    Serial.println();

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
        file.close();
    }
    containerWriter.setFileName(cFileName);
    debugWriter.setFileName(cDebugFileName);
    payloadWriter.setFileName(pFileName);
    DEBUG_PRINTLN("wrote file and setting cam");

    digitalWrite(CAMERA_PIN, HIGH);
    DEBUG_PRINTLN("beeping 3 times");
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
        Serial.println("??? SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        beep(1);
        delay(500);
    }
    if (SD.begin(10))
        Serial.println("??? SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        beep(5);
    }

    DEBUG_PRINTLN("delaying 1 sec");
    delay(1000);
    DEBUG_PRINTLN("gonna recover");
    recovery();
}

uint32_t startPayloadDeployAt;
bool isCamOff = true;
unsigned short witnessRound = 0;
float witnessValues[5];
float witnessSum = 0;
void stateLogic() {
    getBMEData();
    containerWriter.print(packet.combine());
    if (witnessRound > 4) {
        float average = witnessSum / 5;
        float leastDelta = INT_MAX, chosenAlt;
        for (int i = 0; i < 5; i++) {
            if (abs(witnessValues[i] - average) < leastDelta) {
                leastDelta = abs(witnessValues[i] - average);
                chosenAlt = witnessValues[i];
            }
        }
        witnessRound = 0;
        witnessSum = 0;
        debugWriter.print(packet.time);
        debugWriter.print(" Chosen Altitude: ");
        debugWriter.println(chosenAlt);
        switch (packet.state) {
            // PRELAUNCH
            case 0:
                // Entry of LAUNCH state
                if (chosenAlt > 30) packet.state = 1;
                break;

            // LAUNCH
            case 1:
                // Entry of APOGEE state
                if (chosenAlt >= 100 || apogee - chosenAlt >= 10) packet.state = 2;  // testing: (apogee - chosenAlt >= 10 && chosenAlt >= 60)
                break;

            // APOGEE
            case 2:
                // Entry of PARADEPLOY state
                if (chosenAlt <= (apogee > 100 ? 0.6 * apogee : 70)) {  // testing: (chosenAlt <= apogee - 15)
                    packet.state = 3;
                    setParachute(true);
                }
                break;

            // PARADEPLOY
            case 3:
                // Entry of TPDEPLOY state
                if (chosenAlt <= (apogee > 100 ? 0.45 * apogee : 50)) {
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
                    // shouldTransmit = false; // ! DO NOT FORGET TO UNCOMMENT BEFORE REAL MISSION
                    for (int i = 0; i < 5; i++) {
                        xbeeTP.print("OFF\r\r\r");
                        delay(50);
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
        groundAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
        packet.reset();
        setParachute(false);
        packet.payloadReleased = false;
        EEPROM.update(modeAddr, isSimulation);
        EEPROM.update(groundAltAddr, groundAlt);
        EEPROM.update(shouldTransmitAddr, shouldTransmit);
        EEPROM.update(pkgAddr, packet.packetCount);
        EEPROM.update(stateAddr, packet.state);
        brakeSystem.halt();
        brakeSystem.forceBreak();
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
    else if (cmd.startsWith("FORCE,STATE"))
        packet.state = cmd.substring(11).toInt();
    else if (cmd.startsWith("FORCE,MODE"))
        brakeSystem.setMode(cmd.substring(10).toInt());
    else {
        xbeeGS.print("Unknown command: ");
        xbeeGS.print(cmd + '\r');
    }

    debugWriter.print(packet.time);
    debugWriter.println(" CMD: " + cmd);
}

void loop() {
    // Read serial inputs
    while (Serial2.available())
        gps.encode(Serial2.read());

    if (xbeeTP.available()) {
        // DEBUG_PRINTLN("xbeeTP available");
        String in = xbeeTP.readStringUntil('$');
        xbeeGS.print(in + '\r');
        Serial.println(in);
        payloadWriter.print(in + '\r');
        // char c = xbeeTP.read();
        // Serial.print(c);
        // xbeeTP.print(c);
        // if (c == '$') Serial.println();
    }

    if (Serial.available()) {
        DEBUG_PRINTLN("Serial available");
        beep(1);
        while (Serial.available()) {
            const String cmd = Serial.readStringUntil('\n');
            if (cmd == "\n") return;
            doCommand(cmd.substring(9));
        }
    }
    if (xbeeGS.available()) {
        DEBUG_PRINTLN("xbeeGS available");
        beep(1);
        while (xbeeGS.available()) {
            const String cmd = xbeeGS.readStringUntil('\r');
            if (cmd == "\r") return;
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
        getGPSData();
        Serial.println(packet.combine());
        // Serial.println(InternalTemperature.readTemperatureC());
        xbeeGS.print(packet.combine());
        packet.packetCount++;
        EEPROM.update(pkgAddr, packet.packetCount);

        if (shouldSendCustom) {
            Serial.println(packet.combineCustom());
            xbeeGS.print(packet.combineCustom());
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