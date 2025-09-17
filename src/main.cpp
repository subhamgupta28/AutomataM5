#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "Automata.h"
#include "ArduinoJson.h"
#include "MAX30105.h"

#include "heartRate.h"
#include "spo2_algorithm.h"
#define BUFFER_SIZE 100 // 4s of data at 25Hz
// ------------------- Sensor Setup -------------------
MAX30105 particleSensor;

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;
int sampleCount = 0; // current number of samples

// ------------------- Pins & Hardware ----------------
#define I2C_SDA_PIN 7
#define I2C_SCL_PIN 9
#define BUZZER 12 // adjust to your hardware

Adafruit_NeoPixel pixel(1, PIN_LED, NEO_GRB + NEO_KHZ800);

// ------------------- Automata Setup -----------------
const char *HOST = "raspberry.local";
int PORT = 8010;
Automata automata("Capsule", HOST, PORT);

// ------------------- Misc ---------------------------
RTC_DATA_ATTR int bootCount = 1;
unsigned long start = 0;
StaticJsonDocument<256> doc; // safe JSON buffer

// ------------------- Helper Functions ----------------
bool isFingerPresent(uint32_t irValue)
{
  // Adjust based on your hardware / environment
  return (irValue > 150000 && irValue < 500000); // 235671
}

void setLEDStatus(uint8_t r, uint8_t g, uint8_t b)
{
  pixel.setPixelColor(0, r, g, b);
  pixel.show();
}

void action(const Action action)
{
  int buzTone = action.data["buzzer"];
  tone(BUZZER, buzTone, 200);
  delay(10);
  noTone(BUZZER);
}

void sendData()
{
  automata.sendData(doc);
}

void initMAX30105()
{
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30105 not found. Check wiring!");
    while (1)
      ;
  }

  Serial.println("Place finger on sensor.");

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 25; // 25 Hz
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

// ------------------- Non-Blocking Sensor Loop ----------------
void updateMAX30105()
{
  if (particleSensor.available())
  {
    uint32_t red = particleSensor.getRed();
    uint32_t ir = particleSensor.getIR();
    particleSensor.nextSample();

    // Debug: always print raw values
    Serial.print("RED=");
    Serial.print(red);
    Serial.print(", IR=");
    Serial.println(ir);

    // Quick finger detection (no wait for full buffer)
    if (!isFingerPresent(ir))
    {
      setLEDStatus(0, 0, 0); //  no finger
      sampleCount = 0;         // reset
      return;
    }
    setLEDStatus(0, 0, 200); // blue = finger
    // Store samples
    redBuffer[sampleCount] = red;
    irBuffer[sampleCount] = ir;
    sampleCount++;

    // Once enough samples collected, calculate
    if (sampleCount >= BUFFER_SIZE)
    {
      maxim_heart_rate_and_oxygen_saturation(
          irBuffer, BUFFER_SIZE, redBuffer,
          &spo2, &validSPO2, &heartRate, &validHeartRate);

      if (validSPO2 && validHeartRate)
      {
        Serial.print("HR=");
        Serial.print(heartRate);
        Serial.print(", SPO2=");
        Serial.println(spo2);

        doc["heartRate"] = heartRate;
        doc["spo2"] = spo2;

        if (millis() - start > 500)
        {
          automata.sendLive(doc);
          start = millis();
        }

        setLEDStatus(0, 200, 0); // green
      }
      else
      {
        Serial.println("Invalid calculation");
        setLEDStatus(200, 0, 0); // red
      }

      sampleCount = 0; // reset
    }
  }
  else
  {
    particleSensor.check();
  }
}

// ------------------- Setup & Loop ----------------
void setup()
{
  delay(3000);
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);

  pixel.begin();
  pixel.setBrightness(100);
  setLEDStatus(255, 0, 0); // red at boot

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initMAX30105();

  automata.begin();
  automata.addAttribute("heartRate", "Heart Rate", "", "DATA|MAIN");
  automata.addAttribute("spo2", "SPO2", "", "DATA|MAIN");
  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);

  setLEDStatus(0, 0, 0);
  start = millis();
}

void loop()
{
  updateMAX30105(); // non-blocking sensor read
}
