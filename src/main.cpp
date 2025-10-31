#include "config.h"
#include "Automata.h"
#include "ArduinoJson.h"
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiUdp.h>
#define I2C_SDA_PIN 7
#define I2C_SCL_PIN 9
// const char* HOST = "192.168.29.67";
// int PORT = 8080;

const char *HOST = "raspberry.local";
int PORT = 8010;

Preferences preferences;
Automata automata("Battery 178WH", HOST, PORT);
JsonDocument doc;
Adafruit_INA219 ina219_a(0x40);
Adafruit_NeoPixel led(1, 21, NEO_RGB + NEO_KHZ800);

long start = millis();
float c1_shunt = 0;

float c1_volt = 0;

float c1_curr = 0;

float c1_pow = 0;

float targetCapacity = 10.6;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
float totalEnergy = 0;
float percent = 0;
float capacity_mAh = 0;
float dischargingTimeHours = 0;
int nextPowerReadTime = 0;
bool onOff = true;

// Reference voltage for ESP32 ADC (default is 3.3V)
const float referenceVoltage = 3.3;
unsigned long lastUpdateTime = 0;

String isDischarge = "DISCHARGE";

void action(const Action action)
{

  if (action.data.containsKey("reset"))
  {
    // reset = !reset;
    percent = 100.0;
    preferences.putFloat("percent", percent);

    preferences.putFloat("totalEnergy", 0);
    preferences.putFloat("capacity_mAh", 0);
    // startTime = now;
    // startTimeStr = getTimeStr();
    totalEnergy = 0;
    capacity_mAh = 0;
  }

  String jsonString;
  serializeJson(action.data, jsonString);
  Serial.println(jsonString);
}
void getData()
{
  percent = preferences.getFloat("percent", 100.0); // default to full charge

  totalEnergy = preferences.getFloat("totalEnergy", 0);
  capacity_mAh = preferences.getFloat("capacity_mAh", 0);
}
void saveData()
{
  preferences.putFloat("percent", percent);

  preferences.putFloat("totalEnergy", totalEnergy);
  preferences.putFloat("capacity_mAh", capacity_mAh);
}
void sendData()
{
  saveData();
  automata.sendData(doc);
}

void setup()
{
  Serial.begin(115200);
  // pinMode(LED_BUILTIN, OUTPUT);
  led.begin();
  led.setBrightness(5);
  led.setPixelColor(0, 180, 250, 50);
  led.show();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("waiting");
  delay(1000);
  if (!ina219_a.begin())
  {
    Serial.println("Failed to find INA219_B chip");
  }
  preferences.begin("dummy", false);

  automata.begin();
  getData();

  // automata.addAttribute("C1", "V1", "V", "DATA|AUX");
  // automata.addAttribute("C2", "V2", "V", "DATA|AUX");
  // automata.addAttribute("C3", "V3", "V", "DATA|AUX");
  // automata.addAttribute("C4", "V4", "V", "DATA|AUX");

  automata.addAttribute("C1_CURR", "Current", "A", "DATA|AUX");
  automata.addAttribute("C1_POWER", "Power", "W", "DATA|MAIN");

  automata.addAttribute("busVoltage", "Voltage", "V", "DATA|MAIN");
  automata.addAttribute("current", "Current", "A", "DATA|MAIN");
  automata.addAttribute("power", "Power", "W", "DATA|AUX");
  automata.addAttribute("totalEnergy", "Energy", "Wh", "DATA|MAIN");

  automata.addAttribute("percent", "Percent", "%", "DATA|MAIN");
  automata.addAttribute("capacity", "Capacity", "Ah", "DATA|MAIN");
  automata.addAttribute("reset", "Reset", "", "ACTION|MENU|BTN");
  automata.addAttribute("dischargingTime", "Time Left", "Hr", "DATA|MAIN");
    automata.addAttribute("status", "Status", "", "DATA|MAIN");
  // automata.addAttribute("onOff", "OnOff", "", "ACTION|SWITCH");

  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);

  // brightness = preferences.getInt("bright", 2);
  // presets = preferences.getInt("presets", 1);
  // onOff = preferences.getBool("onOff", true);
  led.setPixelColor(0, 250, 250, 250);
  led.show();
}
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readPow()
{
  // Read INA219 data
  c1_shunt = ina219_a.getShuntVoltage_mV();
  c1_volt = ina219_a.getBusVoltage_V();
  c1_curr = ina219_a.getCurrent_mA() * 10.0*-1;
  c1_curr = c1_curr / 1000; // Scaled and converted to A
  c1_pow = c1_volt * c1_curr;

  // Assign readings to global variables
  busvoltage = c1_volt;
  shuntvoltage = c1_shunt;
  current_mA = c1_curr;
  power_mW = busvoltage * current_mA;
  loadvoltage = busvoltage + (shuntvoltage / 1000.0);

  // Time tracking
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  float timeInterval = (currentMillis - previousMillis) / 1000.0;

  totalEnergy += power_mW * (timeInterval / (60 * 60));
  capacity_mAh += current_mA * (timeInterval / (60 * 60));
  isDischarge = current_mA < 0 ? "DISCHARGING" : "CHARGING";
  percent = mapf(busvoltage, 12.8, 16.8, 0.0, 100.0);

  if (isDischarge == "DISCHARGING")
  {
    dischargingTimeHours = (targetCapacity - abs(capacity_mAh)) / abs(current_mA);
  }
  if (percent > 100)
  {
    percent = 100;
  }
  if (percent < 0)
    percent = 0;
  previousMillis = currentMillis;
}

float readVoltage(int pin, float ratio)
{
  int rawADC = analogRead(pin);
  float voltage = (rawADC / 4095.0) * referenceVoltage;
  return voltage * ratio;
}

void loop()
{

  readPow();

  doc["C1_CURR"] = String(c1_curr, 2);
  doc["C1_POWER"] = String(c1_pow, 2);
  // doc["shuntVoltage"] = String(shuntvoltage, 3);
  doc["busVoltage"] = String(busvoltage, 2);
  doc["current"] = String(current_mA, 2);
  doc["power"] = String(power_mW, 2);
  doc["totalEnergy"] = String(totalEnergy, 2);
  // doc["loadVoltage"] = String(loadvoltage, 3);
  doc["percent"] = String(percent, 2);
  doc["capacity"] = String(capacity_mAh, 2);
  doc["dischargingTime"] = String(dischargingTimeHours, 2);
  doc["status"] = isDischarge;

  if ((millis() - start) > 800)
  {

    // digitalWrite(LED_BUILTIN, LOW);
    led.setPixelColor(0, 0, 0, 250);
    led.show();
    automata.sendLive(doc);
    delay(50);
    start = millis();
  }

  led.setPixelColor(0, 0, 0, 0);
  led.show();
  delay(100);
}
