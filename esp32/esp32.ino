#include <Arduino.h>
#include <Wire.h>  
#include <WiFi.h>
#include <Adafruit_GFX.h>
// install lib berikut di platformio (buka terminal dari menu platformio, jalankan pio lib -g install https://github.com .....)
#include <Adafruit_SH1106.h> //https://github.com/nhatuan84/esp32-sh1106-oled

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "SPIFFS.h"

#define DTR PI/180.0
#define RTD 180.0/PI
#define freq 50
#define res 16

#define control_freq freq
#define control_T 1.0 // detik

#define L0 0.1
#define L1 0.02
#define L2 0.13
#define L3 0.02
#define L4 0.15
#define L5 0.0
#define L6 0.125
#define L7 0

#define J1 5
#define J2 18
#define J3 19
#define J4 33
#define J5 32
#define J6 2

#define CHN_J1 1
#define CHN_J2 2
#define CHN_J3 3
#define CHN_J4 4
#define CHN_J5 5
#define CHN_J6 6

Adafruit_SH1106 display(SDA, SCL);
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

String password="12345678";
IPAddress IP_ap = {192, 168, 4, 1};
IPAddress gateway_ap = {192, 168, 4, 1};
IPAddress NMask_ap = {255, 255, 255, 0};

int delayControl = 1000/control_freq;

bool running = false;
bool sendwsdata = false;

float joint1, joint1_awal, joint1_cmd;
float joint2, joint2_awal, joint2_cmd;
float joint3, joint3_awal, joint3_cmd;
float joint4, joint4_awal, joint4_cmd;
float joint5, joint5_awal, joint5_cmd;
float joint6, joint6_awal, joint6_cmd;

float x;
float y;
float z;

int Ksisa = 0;

// put function declarations here:
int myFunction(int, int);

int ServoPos(float pos, float sdtmin, float sdtmax, int pulsamin, int pulsamax) {
  return (int)((((pos - sdtmin) / (sdtmax - sdtmin)) * (float)(pulsamax - pulsamin)) + pulsamin) / 20000.0 * 65535.0;
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

void listSPIFFS() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  
  Serial.println("Start reading SPIFFS");

  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.name());
    file = root.openNextFile();
  }
  Serial.println("End reading SPIFFS");
}

String getOutputStates() {
  StaticJsonDocument<2000> myArray;
  myArray["t"] = millis();
  myArray["ksisa"] = Ksisa;
  myArray["joint"]["1"] = String(joint1, 2);
  myArray["joint"]["2"] = String(joint2, 2);
  myArray["joint"]["3"] = String(joint3, 2);
  myArray["joint"]["4"] = String(joint4, 2);
  myArray["joint"]["5"] = String(joint5, 2);
  myArray["joint"]["6"] = String(joint6, 2);
  myArray["jointcmd"]["1"] = String(joint1_cmd, 2);
  myArray["jointcmd"]["2"] = String(joint2_cmd, 2);
  myArray["jointcmd"]["3"] = String(joint3_cmd, 2);
  myArray["jointcmd"]["4"] = String(joint4_cmd, 2);
  myArray["jointcmd"]["5"] = String(joint5_cmd, 2);
  myArray["jointcmd"]["6"] = String(joint6_cmd, 2);
  myArray["end"]["x"] = String(x, 3);
  myArray["end"]["y"] = String(y, 3);
  myArray["end"]["z"] = String(z, 3);
  
  String jsonString;
  serializeJson(myArray, jsonString);
  return jsonString;
}

void notifyClients(String state) {
  ws.textAll(state);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    if (strcmp((char*)data, "states") == 0) {
      notifyClients(getOutputStates());
    } else if (strcmp((char*)data, "Run") == 0) {
      if (!running) running = true; else running = false;
    } else if (strcmp((char*)data, "Reset") == 0) {
      ESP.restart();
    } else if (strcmp((char*)data, "feedme") == 0) {
      sendwsdata = true;
    } else {
      Serial.printf((char*)data);
      DynamicJsonDocument docJson(2048);
      DeserializationError deserror = deserializeJson(docJson, (char*)data);
      if (deserror) Serial.println(String(F("deserializeJson() failed: ")) + String(deserror.f_str()));
      String cmd = docJson["cmd"].as<String>();   
      Serial.println("\ndata cmd Masuk " + cmd);
      if (cmd.equals("joint")) {
        String joint = docJson["joint"].as<String>(); 
        float val = docJson["value"].as<float>(); 
        if (joint.equals("dq1")) {
          joint1_awal = joint1; joint1_cmd = joint1 + val * DTR; Ksisa = control_freq * control_T; 
        } else if (joint.equals("dq2")) {
          joint2_awal = joint2; joint2_cmd = joint2 + val * DTR; Ksisa = control_freq * control_T; 
        } else if (joint.equals("dq3")) {
          joint3_awal = joint3; joint3_cmd = joint3 + val * DTR; Ksisa = control_freq * control_T; 
        } else if (joint.equals("dq4")) {
          joint4_awal = joint4; joint4_cmd = joint4 + val * DTR; Ksisa = control_freq * control_T; 
        } else if (joint.equals("dq5")) {
          joint5_awal = joint5; joint5_cmd = joint5 + val * DTR; Ksisa = control_freq * control_T; 
        } else if (joint.equals("dq6")) {
          joint6_awal = joint6; joint6_cmd = joint6 + val * DTR; Ksisa = control_freq * control_T; 
        }
        Serial.println("joint " + joint + ":" + String(val));
      } else if (cmd.equals("task")) {
        String axis = docJson["axis"].as<String>(); 
        float val = docJson["value"].as<float>(); 
        Serial.println("joint " + axis + ":" + String(val));
      } 
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void handleWeb(void *pvParameters) {
  printf("handleWeb running on core %d\n", xPortGetCoreID()); // on Serial

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("request web /");
    request->send(SPIFFS, "/index.html", "text/html", false);
  });
  server.on("/css/gabung.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse* response = request->beginResponse(SPIFFS, "/css/gabung.css.gz", "text/css");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });
  server.on("/js/gabung.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse* response = request->beginResponse(SPIFFS, "/js/gabung.js.gz", "text/javascript");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });
  server.on("/lastdata", HTTP_GET, [](AsyncWebServerRequest *request) {
    String jsonString = getOutputStates();
    request->send(200, "application/json", jsonString);
  });
  server.on("/update", HTTP_PUT, [](AsyncWebServerRequest *request) {
    float temp;
    if (request->hasParam("j1")) {
      joint1 = request->getParam("j1")->value().toFloat();
      temp = ServoPos(joint1, -45.0, 45.0, 1200, 2200);
      ledcWrite(CHN_J1, temp);
      request->send(200, "text/plain", "Berhasil update j1 " + String(joint1));
    }
    if (request->hasParam("j2")) {
      joint2 = request->getParam("j2")->value().toFloat();
      if (joint2 > 135.0 || joint2 < 45.0) {
        request->send(400, "text/plain", "Gagal ");
        return;
      }
      temp = ServoPos(joint2, 45.0, 135.0, 900, 2100);
      ledcWrite(CHN_J2, temp);
      request->send(200, "text/plain", "Berhasil update j2 " + String(joint2));
    }
    if (request->hasParam("j3")) {
      joint3 = request->getParam("j3")->value().toFloat();
      if (joint3 > 135.0 || joint3 < 45.0) {
        request->send(400, "text/plain", "Gagal ");
        return;
      }
      temp = ServoPos(joint3, 45.0, 135.0, 900, 2100);
      ledcWrite(CHN_J3, temp);
      request->send(200, "text/plain", "Berhasil update j3 " + String(joint3));
    }
    if (request->hasParam("j4")) {
      joint4 = request->getParam("j4")->value().toFloat();
      if (joint4 > 135.0 || joint4 < 45.0) {
        request->send(400, "text/plain", "Gagal ");
        return;
      }
      temp = ServoPos(joint4, 45.0, 135.0, 900, 2100);
      ledcWrite(CHN_J4, temp);
      request->send(200, "text/plain", "Berhasil update j4 " + String(joint4));
    }
    if (request->hasParam("j5")) {
      joint5 = request->getParam("j5")->value().toFloat();
      if (joint5 > 135.0 || joint5 < 45.0) {
        request->send(400, "text/plain", "Gagal ");
        return;
      }
      temp = ServoPos(joint5, 45.0, 135.0, 900, 2100);
      ledcWrite(CHN_J5, temp);
      request->send(200, "text/plain", "Berhasil update j5 " + String(joint5));
    }
    if (request->hasParam("j6")) {
      joint6 = request->getParam("j6")->value().toFloat();
      if (joint6 > 135.0 || joint6 < 45.0) {
        request->send(400, "text/plain", "Gagal ");
        return;
      }
      temp = ServoPos(joint6, 45.0, 135.0, 900, 2100);
      ledcWrite(CHN_J6, temp);
      request->send(200, "text/plain", "Berhasil update j6 " + String(joint6));
    }
    request->send(200, "text/plain", "Tidak ada yang di update");
  });

  server.serveStatic("/", SPIFFS, "/");
  server.onNotFound([](AsyncWebServerRequest *request) { request->send(404, "text/plain", "Not found"); });

  // Start ElegantOTA
  AsyncElegantOTA.begin(&server);
  initWebSocket();
  server.begin();
  for (;;) {
    delay(1000);
  }
}

void controlLoop(void *pvParameters) {
  printf("controlLoop running on core %d\n", xPortGetCoreID()); // on Serial
  // setup PWM motor
  ledcSetup(CHN_J1, freq, res);
  ledcSetup(CHN_J2, freq, res);
  ledcSetup(CHN_J3, freq, res);
  ledcSetup(CHN_J4, freq, res);
  ledcSetup(CHN_J5, freq, res);
  ledcSetup(CHN_J6, freq, res);
  ledcAttachPin(J1, CHN_J1);
  ledcAttachPin(J2, CHN_J2);
  ledcAttachPin(J3, CHN_J3);
  ledcAttachPin(J4, CHN_J4);
  ledcAttachPin(J5, CHN_J5);
  ledcAttachPin(J6, CHN_J6);

  float maxK = control_freq * control_T;

  for (;;) {
    if (Ksisa > 0) {
      joint1 = joint1_awal + (joint1_cmd - joint1_awal) * (maxK - (float)Ksisa) / maxK;
      joint2 = joint2_awal + (joint2_cmd - joint2_awal) * (maxK - (float)Ksisa) / maxK;
      joint3 = joint3_awal + (joint3_cmd - joint3_awal) * (maxK - (float)Ksisa) / maxK;
      joint4 = joint4_awal + (joint4_cmd - joint4_awal) * (maxK - (float)Ksisa) / maxK;
      joint5 = joint5_awal + (joint5_cmd - joint5_awal) * (maxK - (float)Ksisa) / maxK;
      joint6 = joint6_awal + (joint6_cmd - joint6_awal) * (maxK - (float)Ksisa) / maxK;
      Ksisa--;
    } else {
      joint1_awal = joint1;
      joint2_awal = joint2;
      joint3_awal = joint3;
      joint4_awal = joint4;
      joint5_awal = joint5;
      joint6_awal = joint6;
    }

    // Forward kinematics for a 6-DOF robot would be updated here accordingly
    // Here is a placeholder calculation for x, y, z
    x = L1*cos(joint1) + L5*(cos(joint5)*(sin(joint1)*sin(joint4) + cos(joint4)*(cos(joint1)*cos(joint2)*cos(joint3) - cos(joint1)*sin(joint2)*sin(joint3))) - sin(joint5)*(cos(joint1)*cos(joint2)*sin(joint3) + cos(joint1)*cos(joint3)*sin(joint2))) + L6*(sin(joint5)*(sin(joint1)*sin(joint4) + cos(joint4)*(cos(joint1)*cos(joint2)*cos(joint3) - cos(joint1)*sin(joint2)*sin(joint3))) + cos(joint5)*(cos(joint1)*cos(joint2)*sin(joint3) + cos(joint1)*cos(joint3)*sin(joint2))) + L7*(sin(joint5)*(sin(joint1)*sin(joint4) + cos(joint4)*(cos(joint1)*cos(joint2)*cos(joint3) - cos(joint1)*sin(joint2)*sin(joint3))) + cos(joint5)*(cos(joint1)*cos(joint2)*sin(joint3) + cos(joint1)*cos(joint3)*sin(joint2))) + L3*(cos(joint1)*cos(joint2)*cos(joint3) - cos(joint1)*sin(joint2)*sin(joint3)) + L4*(cos(joint1)*cos(joint2)*sin(joint3) + cos(joint1)*cos(joint3)*sin(joint2)) + L2*cos(joint1)*cos(joint2);
    y = L4*(cos(joint2)*sin(joint1)*sin(joint3) + cos(joint3)*sin(joint1)*sin(joint2)) - L3*(sin(joint1)*sin(joint2)*sin(joint3) - cos(joint2)*cos(joint3)*sin(joint1)) + L1*sin(joint1) - L5*(cos(joint5)*(cos(joint1)*sin(joint4) + cos(joint4)*(sin(joint1)*sin(joint2)*sin(joint3) - cos(joint2)*cos(joint3)*sin(joint1))) + sin(joint5)*(cos(joint2)*sin(joint1)*sin(joint3) + cos(joint3)*sin(joint1)*sin(joint2))) - L6*(sin(joint5)*(cos(joint1)*sin(joint4) + cos(joint4)*(sin(joint1)*sin(joint2)*sin(joint3) - cos(joint2)*cos(joint3)*sin(joint1))) - cos(joint5)*(cos(joint2)*sin(joint1)*sin(joint3) + cos(joint3)*sin(joint1)*sin(joint2))) - L7*(sin(joint5)*(cos(joint1)*sin(joint4) + cos(joint4)*(sin(joint1)*sin(joint2)*sin(joint3) - cos(joint2)*cos(joint3)*sin(joint1))) - cos(joint5)*(cos(joint2)*sin(joint1)*sin(joint3) + cos(joint3)*sin(joint1)*sin(joint2))) + L2*cos(joint2)*sin(joint1);
    z = L0 + L3*(cos(joint2)*sin(joint3) + cos(joint3)*sin(joint2)) - L4*(cos(joint2)*cos(joint3) - sin(joint2)*sin(joint3)) + L5*(sin(joint5)*(cos(joint2)*cos(joint3) - sin(joint2)*sin(joint3)) + cos(joint4)*cos(joint5)*(cos(joint2)*sin(joint3) + cos(joint3)*sin(joint2))) - L6*(cos(joint5)*(cos(joint2)*cos(joint3) - sin(joint2)*sin(joint3)) - cos(joint4)*sin(joint5)*(cos(joint2)*sin(joint3) + cos(joint3)*sin(joint2))) - L7*(cos(joint5)*(cos(joint2)*cos(joint3) - sin(joint2)*sin(joint3)) - cos(joint4)*sin(joint5)*(cos(joint2)*sin(joint3) + cos(joint3)*sin(joint2))) + L2*sin(joint2);
    if (sendwsdata) {
      notifyClients(getOutputStates());
    }
    vTaskDelay(delayControl / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200); 
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

  initSPIFFS();
  listSPIFFS();
  WiFi.mode(WIFI_MODE_AP);
  uint64_t chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
  uint16_t chip = (uint16_t)(chipid >> 32);
  String ssid = "YASKAWA-" + String(chip, HEX);
  WiFi.softAP(ssid.c_str(), password.c_str(), 3, 0, 11); // ssid, passwd, ch, hidden, maxconn
  WiFi.softAPConfig(IP_ap, IP_ap, NMask_ap);
  // put your setup code here, to run once:
  pinMode(J1, OUTPUT);
  pinMode(J2, OUTPUT);
  pinMode(J3, OUTPUT);
  pinMode(J4, OUTPUT);
  pinMode(J5, OUTPUT);
  pinMode(J6, OUTPUT);

  xTaskCreatePinnedToCore(
    handleWeb, /* Function to implement the task */
    "handleWeb", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    NULL,  /* Task handle. */
    1); /* Core where the task should run */
  delay(1000);  // send Torque Reference to CANbus
  xTaskCreatePinnedToCore(
    controlLoop, /* Function to implement the task */
    "controlLoop", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    NULL,  /* Task handle. */
    0); /* Core where the task should run */
  delay(1000);
  Serial.println("Setup selesai");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}