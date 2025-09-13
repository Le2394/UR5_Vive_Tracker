#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "math.h"
const char* ssid = "Tenda_CB9AA0";
const char* password = "phamducan"; 

const char* host = "192.168.0.156";    
const uint16_t port = 1111;            

const char* loadcellHost = "10.0.0.36";
const uint16_t loadcellPort = 3333;  

WiFiClient client;
WiFiClient loadcellClient;

#define BUTTON_POS 18
#define BUTTON_ROT 19
#define AS5600_ADDR  0x36  
#define CONF_H       0x07  
#define CONF_L       0x08 
#define TCAADDR 0x70
#define PI 3.14159265358979323846
const float amplitude = 2047.0;
const float offset = 2047.0;
const float frequency = 0.5;
float t = 0.0;
Adafruit_MCP4725 dac;

TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t dacTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;

bool lastPosButtonState = HIGH;
bool lastRotButtonState = HIGH;

int magnetStatus = 0;

float angleOffset = 0.0;

void tcaselect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void checkMagnetPresence() {
  tcaselect(0);
  while ((magnetStatus & 32) != 32) {
    magnetStatus = 0;
    Wire.beginTransmission(0x36);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    while (Wire.available() == 0);
    magnetStatus = Wire.read();
  }
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void configureAS5600() {
  tcaselect(0);
  uint16_t conf = (0b10 << 8) | (0b100 << 2);
  uint8_t confHigh = (conf >> 8) & 0xFF;
  uint8_t confLow  = conf & 0xFF;

  writeRegister(CONF_H, confHigh); 
  delay(10);
  writeRegister(CONF_L, confLow);  
  delay(10);
}

uint16_t readAngle() {
  tcaselect(0);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);

  uint16_t angle = 0;
  if (Wire.available() >= 2) {
    angle = Wire.read() << 8; 
    angle |= Wire.read();
  }
  return angle;
}

float getAngleDegrees() {
  return (readAngle() * 360.0) / 4096.0;
}

void setZeroAngle() {
  angleOffset = getAngleDegrees();
}

float getZeroedAngle() {
  float rawAngle = getAngleDegrees();
  float adjusted = rawAngle - angleOffset;

  if (adjusted < 0) adjusted = 0.0;
  if (adjusted >= 360.0) adjusted -= 360.0;

  return adjusted;
}

void test() {
  static float phase = 0.0;

  for (int i = 0; i <= 3400; i+=100)
  {
    float virtualInput = 2048;
    uint16_t dacValue = virtualInput;
  
  dac.setVoltage(dacValue, false);

  Serial.printf("Input: %.2f → DAC: %d\n", virtualInput, dacValue);

  delay(200);
  }
  for (int i = 3400; i >= 0; i-=50)
  {
    float virtualInput = 2048;
    uint16_t dacValue = virtualInput;
  
  dac.setVoltage(dacValue, false);

  Serial.printf("Input: %.2f → DAC: %d\n", virtualInput, dacValue);

  delay(200);
  }
}

void sensorTask(void* parameter) {
  for (;;) {
    if (!client.connected()) {
      Serial.println("Disconnected. Reconnecting...");
      if (client.connect(host, port)) {
        Serial.println("Reconnected to server.");
      } else {
        Serial.println("Reconnect failed.");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
    }
    tcaselect(0);
    float angle = getZeroedAngle();
    Serial.print("Angle:");
    Serial.println(getZeroedAngle(), 2); 
    String msg = "Angle:" + String(angle) + "\n";
    client.write(msg.c_str()); 
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void dacTask(void* parameter) {
  for (;;) {
    // if (!loadcellClient.connected()) {
    //   if (loadcellClient.connect(loadcellHost, loadcellPort)) {
    //     Serial.println("[DAC] Reconnected to Load Cell server.");
    //   } else {
    //     Serial.println("[DAC] Reconnect failed.");
    //     vTaskDelay(pdMS_TO_TICKS(500));
    //     continue;
    //   }
    // }
    // tcaselect(1);
    // if (loadcellClient.available() >= sizeof(float)) {
    //   float rawForce = 0.0;
    //   loadcellClient.read((uint8_t*)&rawForce, sizeof(float));
    //   rawForce = constrain(rawForce, 0.0, 1.25);
    //   float normalizedForce = rawForce / 1.25;
    //   uint16_t dacValue = (uint16_t)(rawForce * 4095/1.25);
    //   dac.setVoltage(dacValue, false);
    //   Serial.printf("[DAC] Force: %.2f → DAC: %d\n", rawForce, dacValue);
    // }
    
    // for (int i = 0; i <= 3400; i+=100)
    // {
      // tcaselect(1);
      // float virtualInput = 3225;
      // uint16_t dacValue = virtualInput;
    
      // dac.setVoltage(dacValue, false);

      // Serial.printf("Input: %.2f → DAC: %d\n", virtualInput, dacValue);

      // vTaskDelay(pdMS_TO_TICKS(200));
    // }

    // for (int i = 3400; i >= 0; i-=100)
    // {
    //   float virtualInput = i;
    //   uint16_t dacValue = virtualInput;
    
    //   dac.setVoltage(dacValue, false);

    //   Serial.printf("Input: %.2f → DAC: %d\n", virtualInput, dacValue);

    //   vTaskDelay(pdMS_TO_TICKS(200));
    // }

  //   tcaselect(1);
  // float sineValue = sin(2 * PI * frequency * t);
  // float virtualInput = amplitude * sineValue + offset;
  // uint16_t dacValue = (uint16_t)virtualInput;

  // dac.setVoltage(dacValue, false);

  // Serial.println(dacValue);

  // t += 0.01;
  // if (t >= 1.0 / frequency) t = 0; 
  // vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void buttonTask(void* parameter) {
  bool lastPosButtonState = HIGH;
  bool lastRotButtonState = HIGH;

  for (;;) {
    bool currentPosButtonState = digitalRead(BUTTON_POS);
    bool currentRotButtonState = digitalRead(BUTTON_ROT);

    if (lastPosButtonState == HIGH && currentPosButtonState == LOW) {
      client.println("Pos");
      Serial.println("[Button] Sent: Pos");
    } else if (lastPosButtonState == LOW && currentPosButtonState == HIGH) {
      client.println("stopPos");
      Serial.println("[Button] Sent: stopPos");
    }

    if (lastRotButtonState == HIGH && currentRotButtonState == LOW) {
      client.println("Rot");
      Serial.println("[Button] Sent: Rot");
    } else if (lastRotButtonState == LOW && currentRotButtonState == HIGH) {
      client.println("stopRot");
      Serial.println("[Button] Sent: stopRot");
    }

    lastPosButtonState = currentPosButtonState;
    lastRotButtonState = currentRotButtonState;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(800000L);
  pinMode(BUTTON_POS, INPUT_PULLUP);
  pinMode(BUTTON_ROT, INPUT_PULLUP);
  for (uint8_t i = 0; i < 2; i++) {
    tcaselect(i);
  }
  Serial.println("Scanning I2C devices...");
  for (uint8_t i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }
  dac.begin(0x60);
  delay(100);
  checkMagnetPresence();
  configureAS5600();
  delay(50);
  setZeroAngle();
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  if (!client.connect(host, port)) {
    Serial.println("Connection to server failed!");
  } else {
    Serial.println("Connected to server!");
  }

  if (!loadcellClient.connect(loadcellHost, loadcellPort)) {
    Serial.println("Connection to loadcellServer failed!");
  } else {
    Serial.println("Connected to loadcellServer!");
  }

  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 4096, NULL, 1, &sensorTaskHandle, 1);
  xTaskCreatePinnedToCore(dacTask,    "DAC Task",    4096, NULL, 1, &dacTaskHandle,    1);
  xTaskCreatePinnedToCore(buttonTask, "Button Task", 2048, NULL, 1, &buttonTaskHandle, 0);
}

void loop() {
}