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
TwoWire WireADS = TwoWire(0);  // gunakan I2C0 untuk ADS1115 (bisa diganti ke 1 jika konflik)

// XKC-Y25-V PNP Water Level Sensor
#define SENSOR1_PIN 25  // GPIO XKC-Y25-V PNP 1 on upper box
#define SENSOR2_PIN 26  // GPIO XKC-Y25-V PNP 2 on bottom box

// Relay
#define RELAY1 19  // Relay channel 1 WATER PUMP
#define RELAY2 20  // Relay channel 2 LED GROW
#define RELAY3 23  // Relay channel 3 
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
const char* ssid = "Griya Laksita Putra 1"; // WiFi 4 SSID (2,4 GHz)
const char* password = "laksita@123"; // WiFi Password
const char* server = "https://api.thingspeak.com/update"; // ThingSpeak Server URL
String apiKey = "90JR03UWI2W7AKL8"; // ThingSpeak API Key

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

void loop_DS18B20_ts() {
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  //float temperatureF = sensors.getTempFByIndex(0);
  
  Serial.print("Suhu: ");
  Serial.print(temperatureC);
  Serial.println(" ºC");
  
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
  
    String url = server;
    url += "?api_key=" + apiKey + "&field1=" + String(temperatureC, 2);
  
    http.begin(url);
    int httpResponseCode = http.GET();
  
    if (httpResponseCode > 0) {
      Serial.print("Data Suhu terkirim! HTTP response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Gagal kirim data Suhu. Error: ");
      Serial.println(httpResponseCode);
    }
      http.end();
  }
  else {
      Serial.println("WiFi tidak terhubung!");
    }
  delay(15000); // Tunggu 15 detik (batas minimum ThingSpeak)
}

void loop_BH1750FVI_ts() {
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");

  if (lux <= 200) {
    digitalWrite(RELAY2, HIGH); // LED Grow ON when going to night
  }
  else {
    digitalWrite(RELAY2, LOW); // LED Grow OFF when on day
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = server;
    url += "?api_key=" + apiKey + "&field2=" + String(lux, 2);
      
    http.begin(url);
    int httpResponseCode = http.GET();
  
    if (httpResponseCode > 0) {
      Serial.print("Data Lux terkirim! Kode respons: ");
      Serial.println(httpResponseCode);
    }
    else {
      Serial.print("Gagal kirim data Lux. Error: ");
      Serial.println(httpResponseCode);
    }
      http.end();
  }
  else {
    Serial.println("WiFi tidak terhubung!");
  }
  delay(15000); // Batas minimum kirim ThingSpeak = 15 detik
}

void loop_TDSMeter_ts(){
  int16_t adc0 = ads.readADC_SingleEnded(0); // channel A0
  float voltage = adc0 * 0.1875 / 1000.0; // default gain = ±6.144V, tiap bit = 0.1875mV

  // kompensasi suhu
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensatedVoltage = voltage / compensationCoefficient;

  // konversi ke ppm
  float tdsValue = (133.42 * compensatedVoltage * compensatedVoltage * compensatedVoltage
                    - 255.86 * compensatedVoltage * compensatedVoltage
                    + 857.39 * compensatedVoltage) * 0.5;

  Serial.print("ADS TDS Value: ");
  Serial.print(tdsValue, 0);
  Serial.println(" ppm");

  // Kirim ke ThingSpeak
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = server;
    url += "?api_key=" + apiKey + "&field4=" + String(tdsValue, 0); // Ganti ke field yang belum dipakai

    http.begin(url);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      Serial.print("Data TDS via ADS terkirim! Response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Gagal kirim TDS via ADS: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi tidak terhubung saat kirim TDS via ADS");
  }

  delay(15000); // sesuai interval ThingSpeak
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

void loop() {
  loop_DS18B20_ts();
  loop_BH1750FVI_ts();
  loop_TDSMeter_ts();
  loop_XKC_Y25_V_PNP();
}