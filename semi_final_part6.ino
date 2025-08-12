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
#define RELAY1 19  // Relay channel 1 WATER PUMP
#define RELAY2 18  // Relay channel 2 MIST MAKER
// #define RELAY3 23  // Relay channel 3 OFF

//Buzzer
#define BUZZ 27  // Buzzer pin

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
String apiKey = "90JR03UWI2W7AKL8"; // ThingSpeak API Key

// Sensor parameters
float suhu = 0;
float lux = 0;
float tds = 0;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 2000; // Read sensor every 2s

// Mist Maker Parameters
unsigned long mistInterval = 10000; // Mist Maker ON every 15min
unsigned long mistDuration = 5000;  // Running 15s while mist maker ON
unsigned long lastMistTime = 0;
bool mistRunning = false;
unsigned long mistStartTime = 0;

// Relay & Water Level Status Parameter
bool statusRelay1 = 0;
bool statusRelay2 = 0;
bool pumpLocked = false; // true = pompa tidak boleh nyala walaupun upperSensor LOW
// bool statusRelay3 = 0;
int statusAir = 0;

// Buzzer Parameters
static unsigned long lastBuzzToggle = 0;
static bool buzzState = false;

// Upload Parameters
unsigned long lastUploadTime = 0;
const unsigned long uploadInterval = 60000; // Upload every 60s

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
  // pinMode(RELAY3, OUTPUT);

  pinMode(BUZZ, OUTPUT);

  // Relay must be on the OFF state while first start
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  // digitalWrite(RELAY3, LOW);

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

void uploadThingSpeak(float suhu, float lux, float tds, bool relay1, bool relay2, int statusAir) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String url = "https://api.thingspeak.com/update?api_key=" + apiKey + "&field1=" + String(suhu, 2) + "&field2=" + String(lux, 2) + "&field3=" + String(tds, 2) + "&field4=" + relay1 + "&field5=" + relay2 + "&field6=" + statusAir;
    http.begin(url);
    Serial.println(url);

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

// String urlEncode(const char *msg) {
//   const char *hex = "0123456789ABCDEF";
//   String encodedMsg = "";

//   while (*msg != '\0') {
//     if (('a' <= *msg && *msg <= 'z') ||
//         ('A' <= *msg && *msg <= 'Z') ||
//         ('0' <= *msg && *msg <= '9')) {
//       encodedMsg += *msg;
//     } else if (*msg == ' ') {
//       encodedMsg += "%20";
//     } else {
//       encodedMsg += '%';
//       encodedMsg += hex[*msg >> 4];
//       encodedMsg += hex[*msg & 15];
//     }
//     msg++;
//   }
//   return encodedMsg;
// }

void loop() {
  unsigned long currentTime = millis();

  // Sensor Reading Parameters
  if (currentTime - lastReadTime >= readInterval) {
    lastReadTime = currentTime;

    sensors.requestTemperatures();
    suhu = sensors.getTempCByIndex(0);
    lux = lightMeter.readLightLevel();

    int16_t adc0 = ads.readADC_SingleEnded(0);
    float voltage = adc0 * 0.1875 / 1000.0;
    float compensation = 1.0 + 0.02 * (temperature - 25.0);
    float compensatedVoltage = voltage / compensation;
    tds = (133.42 * pow(compensatedVoltage, 3)
         - 255.86 * pow(compensatedVoltage, 2)
         + 857.39 * compensatedVoltage) * 0.5;

    Serial.println("=== SENSOR UPDATE ===");
    Serial.print("Suhu: "); Serial.print(suhu); Serial.println(" °C");
    Serial.print("Lux: "); Serial.print(lux); Serial.println(" lx");
    Serial.print("TDS: "); Serial.print(tds); Serial.println(" ppm");

    /*if (lux <= 200) { // Default value = 200
      digitalWrite(RELAY2, HIGH);
      Serial.println("LED Grow: ON");
      statusRelay2 = 1;
    } else {
      digitalWrite(RELAY2, LOW);
      Serial.println("LED Grow: OFF");
      statusRelay2 = 0;
    }*/

    if (tds <= 100) {
      if (millis() - lastBuzzToggle >= 500) { // interval 500ms
          lastBuzzToggle = millis();
          buzzState = !buzzState; // toggle ON/OFF
          digitalWrite(BUZZ, buzzState ? HIGH : LOW);
      }
    } else {
    digitalWrite(BUZZ, LOW); // matikan buzzer kalau TDS normal
    buzzState = false;       // reset state
    }

    int upperSensor = digitalRead(SENSOR1_PIN);   // HIGH = Water Enough
    int bottomSensor = digitalRead(SENSOR2_PIN);  // HIGH = Water Available

    if (bottomSensor == LOW) {
        // Air sedikit → pompa nyala & unlock
        pumpLocked = false;
        digitalWrite(RELAY1, HIGH);  // ON
        digitalWrite(RELAY2, LOW);   // Mist OFF
        mistRunning = false;
        Serial.println("Air SEDIKIT! Pompa ON, Mist OFF.");
        statusAir = 0;
        statusRelay1 = 1;

    } else {
        if (upperSensor == HIGH) {
            // Air penuh → pompa mati & lock
            pumpLocked = true;
            digitalWrite(RELAY1, LOW);
            Serial.println("Air Penuh, Pompa OFF.");
            statusAir = 2;
            statusRelay1 = 0;

        } else {
            // Air mulai turun tapi masih cukup
            if (!pumpLocked) {
                digitalWrite(RELAY1, HIGH);  // ON
                Serial.println("Air Belum Penuh, melanjutkan pengisian air.");
                statusRelay1 = 1;
                statusAir = 1;
            } else {
                digitalWrite(RELAY1, LOW);   // Tetap OFF karena locked
                Serial.println("Air cukup tapi pompa tetap mati (locked).");
                statusRelay1 = 0;
                statusAir = 1;
            }
        }

        // Mist maker hanya jalan kalau air cukup atau penuh
        if (!mistRunning && currentTime - lastMistTime >= mistInterval) {
            mistRunning = true;
            mistStartTime = currentTime;
            digitalWrite(RELAY2, HIGH);
            Serial.println("Mist Maker ON (start)");
            statusRelay2 = 1;
        }
        if (mistRunning && currentTime - mistStartTime >= mistDuration) {
            mistRunning = false;
            lastMistTime = currentTime;
            digitalWrite(RELAY2, LOW);
            Serial.println("Mist Maker OFF (done)");
            statusRelay2 = 0;
        }
    }
    Serial.print("Relay 1= "); Serial.println(statusRelay1);
    Serial.print("Relay 2= "); Serial.println(statusRelay2);
    Serial.print("Status Air= "); Serial.println(statusAir);
  }

  // Upload Parameters
  if (currentTime - lastUploadTime >= uploadInterval) {
    lastUploadTime = currentTime;
    uploadThingSpeak(
    suhu,
    lux,
    tds,
    statusRelay1,
    statusRelay2,
    statusAir
    //urlEncode(statusAir.c_str())
    );
  }
}