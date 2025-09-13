#include <WiFi.h>
#include <Wire.h>

const char* ssid = "Codientu";
const char* password = "khongbiet"; 

const char* host = "10.0.0.36";    
const uint16_t port = 1111;            

WiFiClient client;

#define BUTTON_POS 12
#define BUTTON_ROT 13

bool lastPosButtonState = HIGH;
bool lastRotButtonState = HIGH;

int magnetStatus = 0;
int lowbyte;
word highbyte;
int rawAngle;
float degAngle;

int quadrantNumber, previousquadrantNumber;
float numberofTurns = 0;
float correctedAngle = 0;
float startAngle = 0;
float totalAngle = 0;
float previoustotalAngle = -1;
float recentTotalAngle = 0;
float rpmValue = 0;
float rpmInterval = 200;
float rpmTimer = 0;
float timerdiff = 0;
float filteredDegAngle = 0;
const float alpha = 0.5;

void ReadRawAngle() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0D);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (Wire.available() == 0);
  lowbyte = Wire.read();

  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (Wire.available() == 0);
  highbyte = Wire.read();
  highbyte = highbyte << 8;
  rawAngle = highbyte | lowbyte;
  degAngle = rawAngle * 0.087890625;
  filteredDegAngle = alpha * degAngle + (1 - alpha) * filteredDegAngle;
  //Serial.println(degAngle);
  //client.println(degAngle);
}

void correctAngle() {
  correctedAngle = filteredDegAngle - startAngle;
  if (correctedAngle < 0) {
    correctedAngle += 360;
  }
  //Serial.println(correctedAngle);
  //client.println(correctedAngle);
}

void checkQuadrant() {
  if (correctedAngle >= 0 && correctedAngle <= 90) quadrantNumber = 1;
  if (correctedAngle > 90 && correctedAngle <= 180) quadrantNumber = 2;
  if (correctedAngle > 180 && correctedAngle <= 270) quadrantNumber = 3;
  if (correctedAngle > 270 && correctedAngle < 360) quadrantNumber = 4;

  if (quadrantNumber != previousquadrantNumber) {
    if (quadrantNumber == 1 && previousquadrantNumber == 4) numberofTurns++;
    if (quadrantNumber == 4 && previousquadrantNumber == 1) numberofTurns--;
    previousquadrantNumber = quadrantNumber;
  }

  totalAngle = (numberofTurns * 360) + correctedAngle;
  Serial.print("Angle:");
  Serial.println(totalAngle);
  client.print("Angle:");
  client.println(totalAngle);
}

void checkMagnetPresence() {
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

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(800000L);
  
  pinMode(BUTTON_POS, INPUT_PULLUP);
  pinMode(BUTTON_ROT, INPUT_PULLUP);

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
  
  checkMagnetPresence();
  ReadRawAngle();
  startAngle = degAngle;
  filteredDegAngle = degAngle;
}

void loop() {

  if (!client.connected()) {
    Serial.println("Disconnected. Reconnecting...");
    if (client.connect(host, port)) {
      Serial.println("Reconnected to server.");
    } else {
      delay(1000);
      return;
    }
  }
  
  ReadRawAngle();
  correctAngle();
  checkQuadrant();

  bool currentPosButtonState = digitalRead(BUTTON_POS);
  bool currentRotButtonState = digitalRead(BUTTON_ROT);

  if (lastPosButtonState == HIGH && currentPosButtonState == LOW) {
    client.println("Pos");
    Serial.println("Sent: Pos");
  }
  else if (lastPosButtonState == LOW && currentPosButtonState == HIGH) {
    client.println("stopPos");
    Serial.println("Sent: stopPos");
  }
  if (lastRotButtonState == HIGH && currentRotButtonState == LOW) {
    client.println("Rot");
    Serial.println("Sent: Rot");
  }
  else if (lastRotButtonState == LOW && currentRotButtonState == HIGH) {
    client.println("stopRot");
    Serial.println("Sent: stopRot");
  }
  
  lastPosButtonState = currentPosButtonState;
  lastRotButtonState = currentRotButtonState;
  delay(50);
}

