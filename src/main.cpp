#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <ESP32Encoder.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <VNC.h>
#include <SPI.h>
#include <VNC_ILI9341.h>
// EEPROM Settings
#define EEPROM_SIZE 64
#define EEPROM_SN_ADDR 0
#define EEPROM_FW_ADDR 32
#include "image.h"
#define SCREEN_WIDTH 400
#define SCREEN_HEIGHT 400
// Pin Definitions
#define TFT_CS 5    // Default VSPI CS
#define TFT_DC 32   // Data/Command pin
#define TFT_MOSI 23 // Default VSPI MOSI
#define TFT_SCLK 18 // Default VSPI SCLK
#define TFT_LED 17  // Backlight control
#define encClk 4
#define encDT 33
#define encSW 34

// WiFi Settings
const char *ssid = "XXXXXX";
const char *password = "XXXXXX";
const char *vnc_ip = "192.168.1.87";
const uint16_t vnc_port = 5900;
const char *vnc_pass = "XXXXXX";

// TFT Display Instance
ILI9341VNC tft = ILI9341VNC(TFT_CS, TFT_DC, -1);
arduinoVNC vnc = arduinoVNC(&tft);
// Encoder Instance
ESP32Encoder encoder;
bool linkConnection = false;
unsigned long lastButtonPress = 0;
volatile int32_t counter = 0;

// Serial Number and Firmware Version
String sn;
String firmwareVer;

String getVNCAddr()
{
  return String(vnc_ip) + String(":") + String(vnc_port);
}
void TFTnoWifi(void)
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, ((tft.getHeight() / 2) - (5 * 8)));
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(5);
  tft.println("NO WIFI!");
  tft.setTextSize(2);
  tft.println();
}

void TFTnoVNC(void)
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, ((tft.getHeight() / 2) - (4 * 8)));
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(4);
  tft.println("connect VNC");
  tft.setTextSize(2);
  tft.println();
  tft.print(vnc_ip);
  tft.print(":");
  tft.println(vnc_port);
}
void vnc_task(void *pvParameters)
{
  while (1)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      tft.println("WiFi Disconnected!Check your WiFi ");
      vnc.reconnect();
      vTaskDelay(100);
    }
    else
    {
      vnc.loop();
      if (!vnc.connected())
      {
        tft.println("Connecting VNC ");

        vTaskDelay(5000);
      }
    }
    vTaskDelay(1);
  }
}
void customInit()
{
  tft.begin();
  tft.setRotation(0);        // Adjust rotation as needed (0-3)
  tft.setSPISpeed(20000000); // Set SPI speed to 20 MHz

  // Custom command sequence to ensure the correct screen size is set
  tft.writeCommand(ILI9341_MADCTL);
  tft.writeCommand(0x48); // Set the correct memory access control

  // Set the display area
  tft.setAddrWindow(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH); // Turn on the backlight
  Serial.print("Connecting to ");
  Serial.println(ssid);

  tft.println("Connecting to " + String(ssid));

  WiFi.begin(ssid, password);
  TFTnoWifi();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  TFTnoVNC();
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println(F("[SETUP] VNC..."));

  vnc.begin(vnc_ip, vnc_port);
  vnc.setPassword(vnc_pass); // check for vnc server settings

  xTaskCreatePinnedToCore(vnc_task,
                          "vnc_task",
                          10000,
                          NULL,
                          1,
                          NULL,
                          0);
}
void updateCounter()
{
  // tft.fillRect(10, 40, 100, 30, ILI9341_BLACK); // Clear the area where the counter is displayed
  // tft.setCursor(10, 40);
  // tft.print(counter);
}

void readEEPROMSettings()
{
  sn = EEPROM.readString(EEPROM_SN_ADDR);
  firmwareVer = EEPROM.readString(EEPROM_FW_ADDR);

  if (sn.length() == 0)
  {
    sn = String(random(100000, 999999));
    EEPROM.writeString(EEPROM_SN_ADDR, sn);
    EEPROM.commit();
  }

  if (firmwareVer.length() == 0)
  {
    firmwareVer = "toBeWritten";
    EEPROM.writeString(EEPROM_FW_ADDR, firmwareVer);
    EEPROM.commit();
  }
}

void sendJsonResponse(const String &status, const String &message, const String &dataType)
{
  StaticJsonDocument<200> doc;
  doc["status"] = status;
  doc["message"] = message;
  doc["dataType"] = dataType;
  String output;
  serializeJson(doc, output);
  Serial.println(output);
}

void saveSettings()
{
  EEPROM.writeString(EEPROM_SN_ADDR, sn);
  EEPROM.writeString(EEPROM_FW_ADDR, firmwareVer);
  EEPROM.commit();
  sendJsonResponse("success", "Settings saved", "status");
}

void restoreSettings()
{
  sn = EEPROM.readString(EEPROM_SN_ADDR);
  firmwareVer = EEPROM.readString(EEPROM_FW_ADDR);
  sendJsonResponse("success", "Settings restored", "status");
}

void sendStatus()
{
  StaticJsonDocument<200> doc;
  doc["status"] = "status";
  doc["message"] = "Device status";
  doc["data"]["serialNumber"] = sn;
  doc["data"]["firmwareVersion"] = firmwareVer;
  doc["data"]["encoderPosition"] = encoder.getCount();
  String output;
  serializeJson(doc, output);
  Serial.println(output);
}

void handleSerialCommand(const String &command)
{
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, command);

  if (error)
  {
    sendJsonResponse("error", "Invalid JSON", "info");
    return;
  }

  String cmd = doc["cmd"];
  if (cmd == "initialize")
  {
    linkConnection = true;
    sendJsonResponse("connected", "Successfully connected", "status");
  }
  else if (cmd == "disconnect")
  {
    linkConnection = false;
  }
  else if (cmd == "setSN")
  {
    String newSN = doc["args"][0];
    sn = newSN;
    EEPROM.writeString(EEPROM_SN_ADDR, sn);
    EEPROM.commit();
    sendJsonResponse("success", "Serial number updated", "status");
  }
  else if (cmd == "setFW")
  {
    String newFW = doc["args"][0];
    firmwareVer = newFW;
    EEPROM.writeString(EEPROM_FW_ADDR, firmwareVer);
    EEPROM.commit();
    sendJsonResponse("success", "Firmware version updated", "status");
  }
  else if (cmd == "saveSettings")
  {
    saveSettings();
  }
  else if (cmd == "getStatus")
  {
    sendStatus();
  }
  else if (cmd == "restoreSettings")
  {
    restoreSettings();
  }
  else if (cmd == "calibrate")
  {
    sendJsonResponse("success", "Calibration complete", "status");
  }
  else if (cmd == "diagnostic")
  {
    sendJsonResponse("success", "Diagnostic complete", "status");
  }
  else
  {
    sendJsonResponse("error", "Unknown command", "info");
  }
}
void startCountdown()
{
  // buttonclick
}
void handleRotary()
{
  int32_t newCounter = encoder.getCount();
  if (newCounter != counter)
  {
    counter = newCounter;
    updateCounter();
  }

  if (digitalRead(encSW) == LOW)
  {
    counter = 0;
    encoder.clearCount();
    startCountdown();
    // updateCounter();
  }
}

void sendHeartbeat()
{
  sendJsonResponse("heartbeat", "alive", "status");
}

void setup()
{
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  readEEPROMSettings();

  // Initialize Serial Communication
  Serial.begin(115200);

  // Initialize TFT Display
  customInit();

  // Initialize Encoder
  encoder.attachHalfQuad(encDT, encClk);
  encoder.clearCount();

  // Initialize Encoder Switch Pin
  pinMode(encSW, INPUT_PULLUP);

  // Initial Display Setup
  updateCounter();
}

void loop()
{
  static unsigned long lastHeartbeatTime = 0;
  static unsigned long lastStatusTime = 0;

  if (Serial.available() > 0)
  {
    String incomingCommand = Serial.readStringUntil('\n');
    handleSerialCommand(incomingCommand);
  }

  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= 5000 && linkConnection)
  {
    sendHeartbeat();
    lastHeartbeatTime = currentTime;
  }

  if (currentTime - lastStatusTime >= 10000 && linkConnection)
  {
    sendStatus();
    lastStatusTime = currentTime;
  }

  handleRotary();
}
