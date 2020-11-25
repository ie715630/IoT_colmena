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
#define CH_ID 1241668 
#define WRITE_APIKEY "Q61FSQB8WASPJQTA"
#define BME_ADDR (0x76) // Device Addr
#define HTTP_SUCCESS_CODE 200
#define MIN_DELAY_VAL 20000

Adafruit_BME280 bme280;
WiFiClient  client;

const int SW420_GPIO = 15;
char ssid[] = WIFI_SSID; // WIFI network SSID 
char pass[] = PASS_SSID; // WIFI network password 
unsigned long myChannelNumber = CH_ID; // Channel Number from TS 
const char * myWriteAPIKey = WRITE_APIKEY; // Write API Key from TS 
int keyIndex = 0;

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

    if (wifi_success) {
        Serial.println("Connected to WIFI");
    } else {
        Serial.println("WIFI connection failed");
        while (1);
    }

    float temperature_value = bme280.readTemperature();

    // Write to ThingSpeak. There are up to 8 fields in a channel,
    // allowing you to store up to 8 different pieces of information in a channel.
    // Here, we write to field 1.
    int ch_write = ThingSpeak.writeField(myChannelNumber, 1, temperature_value, myWriteAPIKey);
    if(ch_write == HTTP_SUCCESS_CODE) {
        Serial.print("Temperature sent: ");
        Serial.println(temperature_value);
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
