#include <Arduino.h>
#include <SimpleKalmanFilter.h>

float value = 0.0F;
float filtered_value = 0.0F;
void setup() {
    Serial.begin(115200);
}

void loop() {
    if (Serial.available()) {
        value *= Serial.read();
    }

    Serial.print(value);
}