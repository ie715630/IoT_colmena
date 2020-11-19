const int SW420_GPIO = 15;

void setup() {
    Serial.begin(115200);

    pinMode(SW420_GPIO, INPUT);
}

void loop() {
    bool vibration_value = get_vibration();
    Serial.println(vibration_value);
    delay()
}

bool get_vibration() {
    return digitalRead(SW420_GPIO);
}
