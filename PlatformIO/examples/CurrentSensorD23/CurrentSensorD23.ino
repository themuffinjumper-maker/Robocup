// Standalone Teensy 4.0 diagnostic for the Keyestudio ACS712-30A output.
// Wire the sensor OUT through the existing 12k/12k divider to D23/A9 and
// share ground. This sketch only samples and prints; it never drives motors.

constexpr uint8_t CURRENT_PIN = A9; // D23, ADC1 channel 14 on Teensy 4.0
constexpr float ADC_REFERENCE_V = 3.286f; // measured Teensy 3V3 rail
constexpr uint8_t SOFTWARE_SAMPLES = 16;

void setup()
{
    pinMode(CURRENT_PIN, INPUT);
    analogReadResolution(12);
    analogReadAveraging(16);
    Serial.begin(115200);
}

void loop()
{
    // Discard one conversion after any ADC mux settling, then average a short
    // burst. At a 100 ms print rate, this is still responsive to motor load.
    (void)analogRead(CURRENT_PIN);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < SOFTWARE_SAMPLES; ++i)
        sum += analogRead(CURRENT_PIN);

    const float averageCounts = static_cast<float>(sum) / SOFTWARE_SAMPLES;
    const float teensyPinVolts = averageCounts * (ADC_REFERENCE_V / 4095.0f);
    const float sensorOutputVolts = 2.0f * teensyPinVolts;

    Serial.print("D23/A9 ADC1 counts=");
    Serial.print(averageCounts, 1);
    Serial.print("/4095  pin=");
    Serial.print(teensyPinVolts, 4);
    Serial.print(" V  sensor OUT before divider=");
    Serial.print(sensorOutputVolts, 4);
    Serial.println(" V");
    delay(100);
}
