// Reference: https://randomnerdtutorials.com/esp32-ds18b20-temperature-arduino-ide/
// Reference: https://randomnerdtutorials.com/esp32-bh1750-ambient-light-sensor/

#include <WiFi.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_ADS1X15.h>

// TDS Sensor via ADS1115 (SDA = GPIO 16, SCL = GPIO 17)
Adafruit_ADS1115 ads; // menggunakan ADS1115
float tdsValue = 0;
float temperature = 25.0; // kompensasi suhu
TwoWire WireADS = TwoWire(1);  // gunakan I2C0 untuk ADS1115 (bisa diganti ke 1 jika konflik)

// XKC-Y25-V PNP Water Level Sensor
#define SENSOR1_PIN 25  // GPIO XKC-Y25-V PNP 1 on upper box
#define SENSOR2_PIN 26  // GPIO XKC-Y25-V PNP 2 on bottom box

// Relay
#define RELAY1 18  // Relay channel 1 WATER PUMP
#define RELAY2 19  // Relay channel 2 LED GROW
#define RELAY3 23  // Relay channel 3 MIST MAKER
#define RELAY4 24  // Relay channel 4

// DS18B20 Water Temperature Sensor
const int oneWireBus = 4; // Connect to GPIO4 (D4) on ESP32
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// BH1750FVI Light Sensor
BH1750 lightMeter;
/*
SCL BH1750FVI > D22 ESP32
SDA BH1750FVI > D21 ESP32
*/

// WiFi & ThingSpeak Setup
const char* ssid = "WAWAN 6969"; // WiFi 4 SSID (2,4 GHz)
const char* password = "HERMAWAN"; // WiFi Password
const char* server = "https://api.thingspeak.com/update"; // ThingSpeak Server URL
String apiKey = "90JR03UWI2W7AKL8"; // ThingSpeak API Key

// Upload Parameters
unsigned long lastUploadTime = 0;
const unsigned long uploadInterval = 15000; // 15 detik


void setup() {
  Serial.begin(115200);
  sensors.begin();
  Wire.begin(); // I2C default pins for ESP32: SDA=21, SCL=22
  WireADS.begin(16, 17);  // SDA = D16, SCL = D17 (bisa kamu ubah sesuai wiring fisik)
  lightMeter.begin();
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);

  // Relay must be on the OFF state while first start
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);
  digitalWrite(RELAY4, LOW);

  // ADS115 TDS Initiation
  if (!ads.begin(0x48, &WireADS)) {
    Serial.println("Gagal mendeteksi ADS1115!");
    while (1);
  }
  Serial.println("ADS1115 terdeteksi!");
  ads.setGain(GAIN_ONE);  // 1x gain: ±4.096V (1 bit = 125uV)

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan ke WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
}

void uploadThingSpeak(float suhu, float lux, float tds) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String url = server;
    url += "?api_key=" + apiKey;
    url += "&field1=" + String(suhu, 2);
    url += "&field2=" + String(lux, 2);
    url += "&field3=" + String(tds, 0);

    http.begin(url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      Serial.print("Data terkirim! Response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Gagal kirim data! Error code: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi tidak terhubung!");
  }
}

void loop_XKC_Y25_V_PNP(){
  int sensor1State = digitalRead(SENSOR1_PIN);
  int sensor2State = digitalRead(SENSOR2_PIN);

  Serial.print("Upper Sensor: ");
  if (sensor1State == HIGH) {
    Serial.print("Volume Air Cukup");
    digitalWrite(RELAY1, LOW); // Water Pump OFF
  }

  Serial.print(" | Bottom Sensor: ");
  if (sensor2State == LOW) {
    Serial.println("Volume Air Kurang");
    digitalWrite(RELAY1, HIGH); // Water Pump ON
  }
  delay(500);
}

void loop_MIST_MAKER(){
  int bacasensor1 = digitalRead(SENSOR1_PIN);

  if (bacasensor1 == HIGH) {
    digitalWrite(RELAY3, HIGH); // Mist Maker ON
      delay(15000);
    digitalWrite(RELAY3, LOW); // Mist Maker OFF
  }
}

void loop() {
  if (millis() - lastUploadTime >= uploadInterval) {
    lastUploadTime = millis();

    sensors.requestTemperatures();
    float suhu = sensors.getTempCByIndex(0);
    float lux = lightMeter.readLightLevel();

    int16_t adc0 = ads.readADC_SingleEnded(0);
    float voltage = adc0 * 0.1875 / 1000.0;
    float compensation = 1.0 + 0.02 * (temperature - 25.0);
    float compensatedVoltage = voltage / compensation;
    float tds = (133.42 * pow(compensatedVoltage, 3)
               - 255.86 * pow(compensatedVoltage, 2)
               + 857.39 * compensatedVoltage) * 0.5;

    // Cetak ke serial
    Serial.println("=== DATA TERBACA ===");
    Serial.print("Suhu: "); Serial.print(suhu); Serial.println(" °C");
    Serial.print("Lux: "); Serial.print(lux); Serial.println(" lx");
    Serial.print("TDS: "); Serial.print(tds); Serial.println(" ppm");

    uploadThingSpeak(suhu, lux, tds);

    // jalankan relay/otomatisasi sensor lain
    loop_XKC_Y25_V_PNP();
    loop_MIST_MAKER();
  }
}