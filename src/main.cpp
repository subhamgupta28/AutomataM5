#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "Automata.h"
#include "ArduinoJson.h"
#include "MAX30105.h"

#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data

int32_t bufferLength;  // data length
int32_t spo2;          // SPO2 value
int8_t validSPO2;      // indicator to show if the SPO2 calculation is valid
int32_t heartRate;     // heart rate value
int8_t validHeartRate; // indicator to show if the heart rate calculation is valid

#define I2C_SDA_PIN 7 // A pin of the rotary encoder
#define I2C_SCL_PIN 9 // B pin of the rotary encoder

// const char *HOST = "192.168.1.50";
// int PORT = 8010;

const char *HOST = "raspberry.local";
int PORT = 8010;

uint8_t i2cAddress = 0x69;

Automata automata("Capsule", HOST, PORT);

RTC_DATA_ATTR int bootCount = 1;

unsigned long previousMillis = 0;

long start = millis();
bool buz = false;
long buzTime = 100;
int buzt = millis();
int buzTone = 10;
String dateTime = "";
bool isRecord = false;
String downUrl = "";
int ld = 0;

Adafruit_NeoPixel pixel(1, PIN_LED, NEO_GRB + NEO_KHZ800);

JsonDocument doc;

void action(const Action action)
{
  int buzTone = action.data["buzzer"];
  tone(BUZZER, buzTone, 500);
  delay(100);
  analogWrite(BUZZER, LOW);
  String jsonString;
  serializeJson(action.data, jsonString);
  Serial.println(jsonString);
}

void sendData()
{
  pixel.setPixelColor(0, 250, 0, 0);
  pixel.show();
  automata.sendData(doc);
}

void initMAX30105()
{
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  byte ledBrightness = 60; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;    // Options: 69, 118, 215, 411
  int adcRange = 4096;     // Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void setup()
{
  delay(3000);
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT);
  pixel.begin();
  pixel.setBrightness(100);
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  initMAX30105();
  automata.begin();
  automata.addAttribute("heartRate", "Heart Rate", "deg", "DATA|MAIN");
  automata.addAttribute("spo2", "SPO2", "deg", "DATA|MAIN");

  automata.registerDevice();
  pixel.setPixelColor(0, 0, 0, 0);
  pixel.show();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);
}

void readMAX30105()
{
  bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps

  // read the first 100 samples, and determine the signal range
  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false) // do we have new data?
      particleSensor.check();                   // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) // do we have new data?
        particleSensor.check();                   // Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); // We're finished with this sample so move to next sample

      // send samples and calculation result to terminal program through UART
      if (validSPO2 && validHeartRate)
      {
        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.print(irBuffer[i], DEC);

        Serial.print(F(", HR="));
        Serial.print(heartRate, DEC);

        Serial.print(F(", HRvalid="));
        Serial.print(validHeartRate, DEC);

        Serial.print(F(", SPO2="));
        Serial.print(spo2, DEC);

        Serial.print(F(", SPO2Valid="));
        Serial.println(validSPO2, DEC);
        pixel.setPixelColor(0, 200, 200, 0);
        pixel.show();
      }
      else
      {
        pixel.setPixelColor(0, 255, 0, 0);
        pixel.show();
      }
    }

    // After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

void loop()
{
  readMAX30105();
  automata.loop();
  doc["heartRate"] = heartRate;
  doc["spo2"] = spo2;
  if ((millis() - start) > 1000)
  {
    pixel.setPixelColor(0, 200, 200, 0);
    pixel.show();
    automata.sendLive(doc);
    start = millis();
  }

  pixel.setPixelColor(0, 0, 0, 0);
  pixel.show();
  // put your main code here, to run repeatedly:
  // delay(100);
}
