/*
   Documentation for the ThingSpeak Communication Library for Arduino is in the README.md folder where the library was installed.
   See https://www.mathworks.com/help/thingspeak/index.html for the full ThingSpeak documentation.

   For licensing information, see the accompanying license file.

   Copyright 2018, The MathWorks, Inc.
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "ThingSpeak.h"

#define WIFI_SSID "TP-Link_B184"
#define PASS_SSID ""
#define CH_ID 1241687
#define WRITE_APIKEY "PVC8Y4ZX3HURZ9YN"
#define BME_ADDR (0x76) // Device Addr
#define HTTP_SUCCESS_CODE 200
#define SAMPLING_TIME 20000
#define MAX_TEMP 50
#define MIN_TEMP -20
#define LIMIT_HUMIDITY
// #define SAMPLES_IN_1_HR 3600/(SAMPLING_TIME/1000)
# define SAMPLES_IN_1_HR 1
Adafruit_BME280 bme280;
WiFiClient  client;

char ssid[] = WIFI_SSID; // WIFI network SSID
char pass[] = PASS_SSID; // WIFI network password
unsigned long myChannelNumber = CH_ID; // Channel Number from TS
const char * myWriteAPIKey = WRITE_APIKEY; // Write API Key from TS
const int SW420_GPIO = 15;

void setup() {
    Serial.begin(115200);  //Initialize serial

    bool i2c_init = bme280.begin(BME_ADDR); // Initialize BME
    if (!i2c_init) {
        Serial.println("BME device not connected");
    }

    pinMode(SW420_GPIO, INPUT); // Initialize SW420

    WiFi.mode(WIFI_STA);
    ThingSpeak.begin(client);  // Initialize ThingSpeak
}

void loop() {
    static unsigned long prev_time = millis();
    bool wifi_success = false;
    static int sample_num = 0;
    static float temperature_mean = 0;
    static float humidity_mean = 0;
    static float pressure_mean = 0;
    static int frec_mean = 0;

    wifi_success = connect_to_wifi();

    if(!wifi_success) {
        Serial.println("WIFI connection failed");
        while (1);
    }

    float temperature_value = get_temperature();
    float humidity_value = get_humidity();
    float pressure_value = get_pressure();
    int frec_value = (int) get_vibration_frec();

    if ((millis() - prev_time) >= SAMPLING_TIME)
    {
        sample_num += 1;
        temperature_mean += temperature_value;
        humidity_mean += humidity_value;
        pressure_mean += pressure_value;
        frec_mean += frec_value;

        if (sample_num == SAMPLES_IN_1_HR) {
            sample_num = 0;
            ThingSpeak.setField(1, humidity_mean/SAMPLES_IN_1_HR);
            ThingSpeak.setField(2, temperature_mean/SAMPLES_IN_1_HR);
            ThingSpeak.setField(3, pressure_mean/SAMPLES_IN_1_HR);
            ThingSpeak.setField(4, frec_mean/SAMPLES_IN_1_HR);

            temperature_mean = 0;
            humidity_mean = 0;
            pressure_mean = 0;
            frec_mean = 0;

            //    ThingSpeak.setField(1, temperature_value);
            //    ThingSpeak.setField(2, humidity_value);
            //    ThingSpeak.setField(3, pressure_value);
            //    ThingSpeak.setField(4, frec_value);

            int ch_write = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
            if(ch_write == HTTP_SUCCESS_CODE) {
                Serial.println("Data sent correctly");
            }
            else {
                Serial.println("Problem updating channel. HTTP error code " + String(ch_write));
            }
        }

        prev_time = millis();
    } else if (millis() < prev_time) { // Overflow
        prev_time == millis();
    }
}

bool connect_to_wifi() {
    uint8_t num_trys = 10;

    bool wifi_not_connected = (WiFi.status() != WL_CONNECTED);

    if(wifi_not_connected) { // Connect to WIFI if not connected
        do {
            WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network.
            Serial.print(".");
            delay(5000);
            num_trys -= 1;
            wifi_not_connected = (WiFi.status() != WL_CONNECTED);
        } while((wifi_not_connected) && (num_trys != 0));
    }

    if (num_trys == 0) {
        return false;
    } else {
        return true;
    }
}

unsigned long get_vibration_frec() {
    const unsigned long count_millis = 500;
    static unsigned long prev_millis = millis();
    static bool prev_val = 0;
    static unsigned long sample_counter = 0;
    static unsigned long frec = 0;

    bool curr_val = get_vibration();
    if (curr_val != prev_val) {
        sample_counter++;
        prev_val = curr_val;
    }

    if (millis() < prev_millis) {
        prev_millis = millis();
        sample_counter = 0;
    }

    if ((millis() - prev_millis) >= count_millis) {
        prev_millis = millis();
        frec = sample_counter;
        sample_counter = 0;
    }

    return frec;
}

float get_temperature() {
    static float prev_temp = 0;
    float temp = bme280.readTemperature();
    if ((temp > MIN_TEMP) && (temp < MAX_TEMP)) {
        prev_temp = temp;
    }
    return prev_temp;
}

float get_pressure() {
    return bme280.readPressure() / 100.0F;
}

float get_humidity() {
    return bme280.readHumidity();
}

bool get_vibration() {
    return digitalRead(SW420_GPIO);
}
