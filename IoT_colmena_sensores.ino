#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BME_ADDR (0x76)
const int SW420_GPIO = 15;

Adafruit_BME280 bme280;

void setup() {
    Serial.begin(115200);
    bool i2c_init = bme280.begin(BME_ADDR);

    if (!i2c_init) {
        Serial.println("BME device not connected");
    }

    pinMode(SW420_GPIO, INPUT);
}

void loop() {
    bool vibration_value = get_vibration();
    float temperature_value = get_temperature();
    float pressure_value = get_pressure(); 
    float humidity_value = get_humidity();

    Serial.print("Vibration: ");
    Serial.print(vibration_value);
    Serial.print(" Temp: ");
    Serial.print(temperature_value);
    Serial.print(" Pressure: ");
    Serial.print(pressure_value);
    Serial.print(" Humidity: ");
    Serial.println(humidity_value);
}

bool get_vibration() {
    return digitalRead(SW420_GPIO);
}

float get_temperature() {
    return bme280.readTemperature();
}

float get_pressure() {
    return bme280.readPressure() / 100.0F;
}

float get_humidity() {
    return bme280.readHumidity();
}
