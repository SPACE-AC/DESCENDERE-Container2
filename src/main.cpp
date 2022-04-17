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

#define pkgAddr 0
#define stateAddr 10
#define modeAddr 20
#define groundAltAddr 30
#define shouldTransmitAddr 40
#define shouldPollPayloadAddr 50

TinyGPSPlus gps;
BME280 bme;
Servo servoParachute;
Servo servoBreak;
time_t RTCTime;

bool shouldTransmit, shouldPollPayload = false;
float groundAlt;
float apogee = INT_MIN;
bool isSimulation = false;

unsigned long lastTransmit = 0, lastDoStateLogic = 0, lastPoll = 0;

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
    float gpsSat;
    unsigned short state;
    String lastCmd;

    inline String getStateString() const {
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
}(packet);

class BreakSystem {
   private:
    long startAt = -1;
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
}(breakSystem);

class SimulationHandler {
   private:
    bool simEnabled = false, simActivated = false, firstData = false;
    int simPressure;

   public:
    void enable() {
        simEnabled = true;
        simActivated = false;
    }
    // @return Whether the operation is successful.
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
            groundAlt = bme.calcAltitude(simPressure);
            EEPROM.update(groundAltAddr, groundAlt);
            firstData = false;
        }
    }
    int getPressure() const {
        return simPressure;
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
    packet.altitude = bme.calcAltitude(isSimulation ? simHandler.getPressure() : bme.getPressure()) - groundAlt;
    // if (packet.altitude < -500 || packet.altitude > 800) return;
    if (packet.altitude >= apogee) {
        apogee = packet.altitude;
    }
}

void getBattery() {
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    packet.voltage = apparentVoltage * ((R1_OHM + R2_OHM) / R2_OHM);
    if (packet.voltage < 5.3) {
        beep(5, 25);
    }
}

char cFileName[100], pFileName[100];
void recovery() {
    Serial.println("Recovering...");
    EEPROM.get(pkgAddr, packet.packetCount);
    EEPROM.get(stateAddr, packet.state);
    EEPROM.get(modeAddr, isSimulation);
    EEPROM.get(groundAltAddr, groundAlt);
    EEPROM.get(shouldTransmitAddr, shouldTransmit);
    EEPROM.get(shouldPollPayloadAddr, shouldPollPayload);

    Serial.println("Simulation Mode? " + isSimulation ? "Yes" : "No");
    Serial.println("Should Transmit? " + shouldTransmit ? "Yes" : "No");
    Serial.println("Current Packet Count: " + packet.packetCount);
    Serial.println("Current State: " + packet.getStateString());
    Serial.println("Ground Altitude: " + String(groundAlt));

    int fileIndex = 0;
    do {
        fileIndex++;
        String("C_" + String(fileIndex) + ".txt").toCharArray(cFileName, 100);
    } while (SD.exists(cFileName));
    String("TP_" + String(fileIndex) + ".txt").toCharArray(pFileName, 100);
    Serial.print("Selected file name: ");
    Serial.print(cFileName);
    Serial.print(" and ");
    Serial.println(pFileName);

    File file = SD.open(cFileName, FILE_WRITE);
    if (file) {
        file.println("Recovery Successful!");
        file.println("Simulation Mode? " + isSimulation ? "Yes" : "No");
        file.println("Should Transmit? " + shouldTransmit ? "Yes" : "No");
        file.println("Current Packet Count: " + packet.packetCount);
        file.println("Current State: " + packet.getStateString());
        file.println("Ground Altitude: " + String(groundAlt));
        file.close();
    }

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
        beep(1);
        delay(500);
    }
    if (SD.begin(10))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        beep(5);
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
                shouldPollPayload = true;
                lastPoll = lastTransmit + 125;
                breakSystem.start();
                packet.payloadReleased = true;
                for (int i = 0; i < 5; i++) {
                    xbeeTP.print("ON");
                    delay(50);
                }
                EEPROM.update(shouldPollPayloadAddr, shouldPollPayload);
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
    EEPROM.update(stateAddr, packet.state);

    File file = SD.open(cFileName, FILE_WRITE);
    if (file) {
        file.println(packet.combine());
        file.close();
    }
}

void doCommand(String cmd) {
    packet.lastCmd = cmd.substring(0, cmd.indexOf(",")) + cmd.substring(cmd.indexOf(",") + 1);
    if (cmd == "CX,ON") {
        beep(2);
        shouldTransmit = true;
        groundAlt = bme.calcAltitude(bme.getPressure());
        packet.reset();
        setParachute(false);
        EEPROM.update(modeAddr, isSimulation);
        EEPROM.update(groundAltAddr, groundAlt);
        EEPROM.update(shouldTransmitAddr, shouldTransmit);
        EEPROM.update(pkgAddr, packet.packetCount);
        EEPROM.update(stateAddr, packet.state);
        breakSystem.forceBreak();
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
    }
    // Custom commands
    else if (cmd == "FORCE,PARADEPLOY")
        setParachute(true);
    else if (cmd == "FORCE,SEQUENCE")
        breakSystem.start();
    else if (cmd == "FORCE,HALT")
        breakSystem.halt();
    else if (cmd == "FORCE,RELEASE")
        breakSystem.forceRelease();
    else if (cmd == "FORCE,BREAK")
        breakSystem.forceBreak();
    else if (cmd == "FORCE,POLL")
        xbeeTP.print("POLL\r\r\r");
    else if (cmd == "FORCE,POLLON")
        shouldPollPayload = true;
    else if (cmd == "FORCE,POLLOFF")
        shouldPollPayload = false;
    else if (cmd == "FORCE,TPON")
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
    else if (cmd.startsWith("FORCE,STATE"))
        packet.state = cmd.substring(11).toInt();

    File file = SD.open(cFileName, FILE_WRITE);
    if (file) {
        file.println("CMD: " + cmd);
        file.close();
    }
}

void loop() {
    // Read serial inputs
    while (Serial2.available())
        gps.encode(Serial2.read());
    if (xbeeTP.available()) {
        String in = xbeeTP.readStringUntil('$');
        xbeeGS.print(in);
        Serial.println(in);
    }

    if (Serial.available()) {
        beep(1);
        while (Serial.available()) {
            const String cmd = Serial.readStringUntil('\n');
            if (cmd == "\r") return;
            doCommand(cmd.substring(9));
        }
    }
    if (xbeeGS.available()) {
        beep(1);
        while (xbeeGS.available()) {
            const String cmd = xbeeGS.readStringUntil('\r');
            if (cmd == "\r") return;
            doCommand(cmd.substring(9));
        }
    }

    // Poll payload
    if (shouldPollPayload && millis() - lastPoll >= 250) {
        lastPoll = millis();
        xbeeTP.print("POLL\r\r\r");
    }

    breakSystem.run();
    if (millis() - lastDoStateLogic >= 500) {
        lastDoStateLogic = millis();
        sprintf(packet.time, "%02d:%02d:%02d", hour(), minute(), second());
        getGPSData();
        getBattery();
        stateLogic();
    }
    if (shouldTransmit && millis() - lastTransmit >= 1000) {
        lastTransmit = millis();
        Serial.print(packet.combine());
        xbeeGS.print(packet.combine());
        packet.packetCount++;
        EEPROM.update(pkgAddr, packet.packetCount);
    }
}