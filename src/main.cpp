// #include "SparkFun_BMI270_Arduino_Library.h" // imu
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
// #include "I2C_BM8563.h"
#include "config.h"
#include <SD.h>
#include <SPI.h>
#include "Automata.h"
#include "ArduinoJson.h"
#include <SoftwareSerial.h>


#define pinA 7 // A pin of the rotary encoder
#define pinB 9 // B pin of the rotary encoder

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

int minVal = 265;
int maxVal = 402;
float ax;
float ay;
float az;

float gx;
float gy;
float gz;

float x;
float y;
float z;

volatile int encoderPos = 0; // Store encoder position
int lastEncoded = 0;


Adafruit_NeoPixel pixel(1, PIN_LED, NEO_GRB + NEO_KHZ800);
// I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS);
// BMI270 imu;
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

void encoderData(){
  int	currentStateCLK = digitalRead(pinA);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (currentStateCLK != lastEncoded  && currentStateCLK == 1){

		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		if (digitalRead(pinB) != currentStateCLK) {
			encoderPos --;
      pixel.setPixelColor(0, 0, 250, 0);
      pixel.show();
		} else {
			// Encoder is rotating CW so increment
			encoderPos ++;
      pixel.setPixelColor(0, 0, 0, 250);
      pixel.show();
		}
    delay(100);
    JsonDocument doc;
    doc["encoderPos"] = encoderPos;
    doc["key"] = "encoderPos";
	}

	// Remember last CLK state
	lastEncoded = currentStateCLK;
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(POWER_HOLD_PIN, OUTPUT);
  digitalWrite(POWER_HOLD_PIN, HIGH);
  Serial.begin(115200);

  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(E_LED, OUTPUT);
  pinMode(BAT, INPUT);
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(pinA), encoder, RISING);
  pixel.begin();
  pixel.setBrightness(100);
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();
  tone(BUZZER, 4000, 500);
  delay(100);
  analogWrite(BUZZER, LOW);

  // Wire.begin(BM8563_I2C_SDA, BM8563_I2C_SCL);

  // while (imu.beginI2C(i2cAddress))
  // {
  //   Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
  //   delay(1000);
  // }

  // rtc.begin();
  // rtc.clearIRQ();

  delay(200);

  automata.begin();
  // automata.addAttribute("x", "Pos X", "deg", "DATA|MAIN");
  // automata.addAttribute("y", "Pos Y", "deg", "DATA|MAIN");
  // automata.addAttribute("x", "Pos Z", "deg", "DATA|MAIN");
  // automata.addAttribute("encoderPos", "Encoder", "deg", "DATA|MAIN");

  // automata.addAttribute("ax", "Accel X", "g's","DATA|MAIN");
  // automata.addAttribute("ay", "Accel Y", "g's","DATA|MAIN");
  // automata.addAttribute("az", "Accel Z", "g's","DATA|MAIN");

  automata.addAttribute("button", "Button", "On/Off", "ACTION|MENU|BTN");
  automata.addAttribute("buzzer", "Buzzer", "tone", "ACTION|IN");
  // automata.addAttribute("rtcTime", "RTC Time", "");

  automata.registerDevice();
  pixel.setPixelColor(0, 0, 0, 0);
  pixel.show();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);
}
// void updateTime()
// {
//   struct tm timeInfo;
//   if (getLocalTime(&timeInfo))
//   {
//     // Set RTC time
//     I2C_BM8563_TimeTypeDef timeStruct;
//     timeStruct.hours = timeInfo.tm_hour;
//     timeStruct.minutes = timeInfo.tm_min;
//     timeStruct.seconds = timeInfo.tm_sec;
//     rtc.setTime(&timeStruct);

//     // Set RTC Date
//     I2C_BM8563_DateTypeDef dateStruct;
//     dateStruct.weekDay = timeInfo.tm_wday;
//     dateStruct.month = timeInfo.tm_mon + 1;
//     dateStruct.date = timeInfo.tm_mday;
//     dateStruct.year = timeInfo.tm_year + 1900;
//     rtc.setDate(&dateStruct);
//   }
// }
float toDegrees(float radians)
{
  return radians * 180.0 / PI;
}
// void readIMU()
// {
//   imu.getSensorData();

//   // Print acceleration data
//   for (int i = 0; i < 8; i++)
//   {
//     Serial.print(imu.data.auxData[i]);
//     Serial.print(" ");
//   }

//   ax = imu.data.accelX;
//   ay = imu.data.accelY;
//   az = imu.data.accelZ;
//   gx = imu.data.gyroX * 0.001;
//   gy = imu.data.gyroY * 0.001;
//   gz = imu.data.gyroZ * 0.001;

//   float roll = atan2(ay, az);
//   float pitch = atan2(-ax, sqrt(ay * ay + az * az));
//   float yaw = atan2(gy, gx); // Yaw can be more complex; this is a basic calculation

//   // Convert radians to degrees
//   x = toDegrees(roll);
//   y = toDegrees(pitch);
//   z = toDegrees(yaw);

//   Serial.print("Acceleration in g's");
//   Serial.print("\t");
//   Serial.print("X: ");
//   Serial.print(ax, 3);
//   Serial.print("\t");
//   Serial.print("Y: ");
//   Serial.print(ay, 3);
//   Serial.print("\t");
//   Serial.print("Z: ");
//   Serial.print(az, 3);
//   Serial.print("\t");
//   Serial.print("Rotation in deg/sec");
//   Serial.print("\t");
//   Serial.print("X: ");
//   Serial.print(gx, 3);
//   Serial.print("\t");
//   Serial.print("Y: ");
//   Serial.print(gy, 3);
//   Serial.print("\t");
//   Serial.print("Z: ");
//   Serial.println(gz, 3);
// }
void powerOff(int tim)
{

  // rtc.SetAlarmIRQ(tim);
  delay(200);
  pixel.setPixelColor(0, 0, 0, 0);
  pixel.show();
  // esp_deep_sleep_enable_timer_wakeup(1000000*tim);
  esp_deep_sleep_start();
}
// void readRTC()
// {
//   I2C_BM8563_DateTypeDef dateStruct;
//   I2C_BM8563_TimeTypeDef timeStruct;
//   rtc.getDate(&dateStruct);
//   rtc.getTime(&timeStruct);

//   dateTime = String(dateStruct.year) + "/" + String(dateStruct.month) + "/" + String(dateStruct.date) + " " + String(timeStruct.hours) + ":" + String(timeStruct.minutes) + ":" + String(timeStruct.seconds);
// }




void loop()
{

  // readRTC();
  // readIMU();
  // encoderData();
  float batteryVoltage = ((analogRead(BAT) * 2 * 3.3 * 1000) / 4096) / 1000;

  automata.loop();

  float bt = ((analogRead(4) * 2 * 3.3 * 1000) / 4096) / 1000;
  // doc["x"] = String(x, 2);
  // doc["y"] = String(y, 2);
  // doc["z"] = String(z, 2);
  // doc["ax"] = String(ax, 2);
  // doc["ay"] = String(ay, 2);
  // doc["az"] = String(az, 2);
  // doc["encoderPos"] = encoderPos;
  // doc["rtcTime"] = dateTime;
  doc["button"] = digitalRead(BUTTON_PIN);

  if ((millis() - start) > 1000)
  {
    automata.sendLive(doc);
    start = millis();
  }

  if (buz && (millis() - buzt) > buzTime)
  {
    tone(BUZZER, buzTone, 50);
    delay(100);
    // analogWrite(BUZZER, LOW);
    buzt = millis();
  }

  if (digitalRead(BUTTON_PIN) == LOW)
  {
    tone(BUZZER, 4000, 50);
    JsonDocument doc;
    doc["button"] = digitalRead(BUTTON_PIN);
    doc["key"] = "button";
    automata.sendAction(doc);
    pixel.setPixelColor(0, 250, 250, 255);
    pixel.show();
    delay(300);
  }
  else
  {
    pixel.setPixelColor(0, 20, 20, 0);
    pixel.show();
  }

  pixel.setPixelColor(0, 0, 0, 0);
  pixel.show();
  // put your main code here, to run repeatedly:
  delay(100);
}
