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
#define CH_ID 1241725
#define WRITE_APIKEY "N1Y9MTMI5RESKR93"
#define BME_ADDR (0x76) // Device Addr
#define HTTP_SUCCESS_CODE 200
#define MIN_DELAY_VAL 20000

Adafruit_BME280 bme280;
WiFiClient  client;

char ssid[] = WIFI_SSID; // WIFI network SSID 
char pass[] = PASS_SSID; // WIFI network password 
unsigned long myChannelNumber = CH_ID; // Channel Number from TS 
const char * myWriteAPIKey = WRITE_APIKEY; // Write API Key from TS 
int keyIndex = 0;
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

    bool wifi_success = connect_to_wifi();

    if(!wifi_success) {
        Serial.println("WIFI connection failed");
        while (1);
    }
    
    float temperature_value = get_temperature();
    float humidity_value = get_humidity();
    float pressure_value = get_pressure(); 
    int frec_value = (int) get_vibration_frec();

    ThingSpeak.setField(1, temperature_value);
    ThingSpeak.setField(2, humidity_value);
    ThingSpeak.setField(3, pressure_value);
    ThingSpeak.setField(4, frec_value);

    ThingSpeak.setStatus(String("Testing..."));
    
    int ch_write = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(ch_write == HTTP_SUCCESS_CODE) {
        Serial.print("Data sent correctly");
    }
    else {
        Serial.println("Problem updating channel. HTTP error code " + String(ch_write));
    }

    delay(MIN_DELAY_VAL); // Wait 20 seconds to update the channel again
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
    return bme280.readTemperature();
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
