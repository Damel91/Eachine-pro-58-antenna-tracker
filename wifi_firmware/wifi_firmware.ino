#include <ArduinoJson.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <BMx280I2C.h>
#include <ESP8266mDNS.h>

#define I2C_ADDRESS 0x76
#define I2C_SLAVE 8 //Arduino nano register
#define SDA_PIN D4
#define SCL_PIN D3

String message, txMessage;
char a[15];

//create a BMx280I2C object using the I2C interface with I2C Address 0x76
BMx280I2C bmx280(I2C_ADDRESS);
double batteryValue = 0;
double cellVoltage = 0;
int batteryCellCount = 0;
int batteryStatus = 0;
int val = 0;
int RSSIA = 0;
int RSSIB = 0;
double pressure = 0.00;
double temperature = 0.00;
double altitude = 0.00;
bool whoGoes = false;

int trackerMode = 0, pitchPos = 0, yawPos = 0, kP = 0, kI = 0, kD = 0, isVerticalTracking = 0, deadBand = 0;

unsigned long batteryTimer = 0, trackerTimer = 0, barometerTimer = 0;
unsigned long timeToWait = 500;
 
// Set AP credentials
#define AP_SSID "TrackerGroundStation"
#define AP_PASS "pass_word"

const char* HOSTNAME = "antenna_tracker";

AsyncWebServer server(80); //Server on port 80

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000L);
  wifiConfig();
  baroConfig();
  serverResource();
}

void loop() {
  checkBattery();
  checkBarometer();
  checkTracker();
}

void checkBattery(){
  unsigned long now = millis();
  if(now - batteryTimer > timeToWait){
    batteryTimer = now;
    int val = analogRead(A0);
    batteryValue = val/40.5952380952;
    batteryCellCount = abs(batteryValue / 3.501);
    if(batteryCellCount > 6)
      batteryCellCount--;
      
    cellVoltage = batteryValue / batteryCellCount;
    double minVoltage = batteryCellCount * 3.5;
    double maxVoltage = batteryCellCount * 4.2;
    batteryStatus = (batteryValue - minVoltage) / (maxVoltage - minVoltage) * 100;
  }
}

void checkBarometer(){
  unsigned long now = millis();
  //start a measurement
  if (!bmx280.measure() && (now - barometerTimer <= timeToWait))
    return;

  barometerTimer = now;
    
  if(bmx280.hasValue()){
    pressure = bmx280.getPressure64();
    temperature = bmx280.getTemperature();
    altitude = 44330 * (1 - pow((bmx280.getPressure()/101325.00),(1/5.255)));
  }
}

void checkTracker(){
  unsigned long now = millis();
  if(now - trackerTimer > 150){
    trackerTimer = now;
    if(whoGoes){
      sendMessage();
      whoGoes = false;
    }
    else
      Receive();
  }
}

void Receive() {
  Wire.requestFrom(I2C_SLAVE, 21);
  
  char c;
  message = "";
  while (Wire.available())   // slave may send less than requested
  {
    c = Wire.read();    // receive a byte as character
    message = message + String(c);
  }

  Serial.println(message);

  int indexN = message.indexOf('N');
  int indexA = message.indexOf('A');
  int indexB = message.indexOf('B');
  int indexY = message.indexOf('Y');
  int indexP = message.indexOf('P');
  int indexK = message.indexOf('K');
  int indexI = message.indexOf('I');
  int indexD = message.indexOf('D');
  int indexM = message.indexOf('M');
  int indexV = message.indexOf('V');
  int indexC = message.indexOf('C');

  if(indexN<0){
    if(indexI<0){
      RSSIA = message.substring(0, indexA).toInt();
      RSSIB = message.substring(indexA+1, indexB).toInt();
      yawPos = message.substring(indexB+1, indexY).toInt();
      pitchPos = message.substring(indexY+1, indexP).toInt();
      kP = message.substring(indexP+1, indexK).toInt(); 
    } else {
      kI = message.substring(0, indexI).toInt();
      kD = message.substring(indexI+1, indexD).toInt();
      trackerMode = message.substring(indexD+1, indexM).toInt();
      isVerticalTracking = message.substring(indexM+1, indexV).toInt();
      deadBand = message.substring(indexV+1, indexC).toInt();
    }   
  }
}

void sendMessage() {
  Wire.beginTransmission(I2C_SLAVE); // transmit to device #8
  Wire.write(a);        
  Wire.endTransmission();    // stop transmitting
}


void wifiConfig(){
  // Begin Access Point
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setHostname(HOSTNAME);

}

void baroConfig(){
  //begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
  //and reads compensation parameters.
  if (!bmx280.begin())
  {
    //Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
    while (1);
  }

  //reset sensor to default parameters.
  bmx280.resetToDefaults();

  //by default sensing is disabled and must be enabled by setting a non-zero
  //oversampling setting.
  //set an oversampling setting for pressure and temperature measurements. 
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
}


void serverResource(){
  if (!MDNS.begin(HOSTNAME)){
    return;
  }
  MDNS.addService("http", "tcp", 80);
        
  server.on("/getLoad", HTTP_GET, [](AsyncWebServerRequest * request) {
    DynamicJsonDocument doc(1024);
    doc["batteryInfo"][0] = batteryValue;
    doc["batteryInfo"][1] = cellVoltage;
    doc["batteryInfo"][2] = batteryCellCount;
    doc["batteryInfo"][3] = batteryStatus;
    doc["barometerInfo"][0] = pressure;
    doc["barometerInfo"][1] = altitude;
    doc["barometerInfo"][2] = temperature;
    doc["trackerInfo"][0] = trackerMode;
    doc["trackerInfo"][1] = pitchPos;
    doc["trackerInfo"][2] = yawPos;
    doc["trackerInfo"][3] = RSSIA;
    doc["trackerInfo"][4] = RSSIB;
    doc["trackerInfo"][5] = kP;
    doc["trackerInfo"][6] = kI;
    doc["trackerInfo"][7] = kD;
    doc["trackerInfo"][8] = isVerticalTracking;
    doc["trackerInfo"][9] = deadBand;
    
    String jsonData;
    serializeJson(doc, jsonData);
    request->send(200, "text/json", jsonData);
  });

  server.on("/sendToTracker", HTTP_GET, [](AsyncWebServerRequest * request) {
    txMessage = request->getParam("testo")->value();
    txMessage.toCharArray(a, 15);
    whoGoes = true;
    request->send(200, "text/plain", txMessage);
  });

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.onNotFound(notFound);
  server.begin();
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}
