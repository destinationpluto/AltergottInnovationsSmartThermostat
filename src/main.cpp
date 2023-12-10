// ************************************************
// ********************coz.i***********************
// *****ALTERGOTT*INNOVATIONS*SMART*THERMOSTAT*****
// ************************************************
// ****************MASTER***BRANCH*****************
#include <FS.h>
#include <SPI.h>
#include <Arduino.h>     //this needs to be first, or it all crashes and burns...
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <Wire.h>        // This library is already built in to the Arduino IDE
#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
// #include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <DHT.h>
#include <Ticker.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp8266.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <StackArray.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RunningMedian.h>
#include <time.h>
#include "settings.h"
#include <JsonListener.h>
// #include <OpenWeatherMapCurrent.h>
// #include <OpenWeatherMapForecast.h>
// #include <Astronomy.h>
#include <MiniGrafx.h>
#include <Carousel.h>
#include <ILI9341_SPI.h>
#include "moonphases.h"
#include "weathericons.h"
#include "Symbols.h"
#include "fonts.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <PlutoStorage.h>

/*******DECLARING FUNCTIONS*******/
void initSpiffs();
void initHardware();
void initSoftware();
void drawTime();
void ReadButtons();
void ConnectFlag();
void OTAFlag();
void ReadDHTFlag();
void DisplayFlag();
void DriveOutputFlag();
void ChangeHeatIconFlag();
void initializeDisplay();
void OutputFunction();
void SetPointFunction(float increaseby, float setvalue);
void DriveOutput();
void EspStandby();
void Standby();
void handleBlynkAndUpdate();
void ConnectFunction();
void TFTFunction();
void drawThermostat();
void drawFire();
void drawWiFiQuality();
void ChangeDisplayFlag();
void DisplayOffFunction();
void DisplayOnFunction();
void ReadDHTFunction();
void SetupFunction();
void EspWebUpdate();
void RestartFunction();
void EspSyncBlynk();
void EspSyncPower();
void EspSyncReadings();
void EspSyncSetpoint();
void EspOTAFunction();
void updateNTP();
void UpdateDataFlag();
void printTime(time_t offset);
void CheckDisplayState();
void SmartWiFiSetupFunction();
void ListenTCP();
void CheckTcpData(String data);
void SaveServerToken(String serverToken);
void CheckSmartWifiState();
void DisplaySetupMode();
int8_t getWifiQuality();

ADC_MODE(ADC_VCC);

PlutoStorage storage;
RunningMedian TemperatureSamples = RunningMedian(10);
RunningMedian HumiditySamples = RunningMedian(10);

// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 600000);
// WiFiManager SmartWifi;

#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
simpleDSTadjust dstAdjusted(StartRule, EndRule);
time_t dstOffset = 0;

// defines the colors usable in the paletted 16 color frame buffer
uint16_t palette[] = {ILI9341_BLACK, // 0
                      ILI9341_WHITE, // 1
                      ILI9341_BLUE,  // 2
                      ILI9341_RED};  // 3

int SCREEN_WIDTH = 240;
int SCREEN_HEIGHT = 320;
// Limited to 4 colors due to memory constraints
int BITS_PER_PIXEL = 2; // 2^2 =  4 colors

ILI9341_SPI tft = ILI9341_SPI(TFT_CS, TFT_DC);
// Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
MiniGrafx gfx = MiniGrafx(&tft, BITS_PER_PIXEL, palette);
bool fireIconLarge = false;

// ************************************************
// ************OPERATING*STATE*VARIABLES***********
// ************************************************
enum operatingStateESP
{
  ESP_STANDBY = 0,
  ESP_CONNECT,
  ESP_DISPLAY,
  ESP_CHANGE_DISPLAY,
  ESP_LCDOFF,
  ESP_LCDON,
  ESP_READDHT,
  ESP_SETUP,
  ESP_WEBUPDATE,
  ESP_RESTART,
  ESP_SYNC_BLYNK,
  ESP_SYNC_POWER,
  ESP_SYNC_SETPOINT,
  ESP_SYNC_READINGS,
  ESP_OTA,
  ESP_UPDATEDATA
};

enum operatingStateThermostat
{
  STANDBY = 0,
  DRIVE_OUTPUT,
  INCREASE_SETPOINT,
  DECREASE_SETPOINT
};
enum OperatingDisplayState
{
  DISPLAY_ON = 0,
  DISPLAY_OFF
};
StackArray<operatingStateESP> opStateEspStack;
StackArray<operatingStateThermostat> opStateThermoStack;
operatingStateESP opStateEsp = ESP_STANDBY;
operatingStateThermostat opStateThermostat = STANDBY;
OperatingDisplayState DisplayState = DISPLAY_ON;

const unsigned long EspStateMachineInterval = 1;
const unsigned long ThermostatStateMachineInterval = 20;
unsigned long previousMillisThermostat = 0;
unsigned long previousMillisStateEsp = 0;
unsigned int showPage = 0;
// ************************************************
// Blynk Variables
// ************************************************
bool isWifiConnected = false;
bool isBlynkConnected = false;
char blynk_token[40] = "YourBlynkToken";

// char ipaddress[40] = "Blynk";

const unsigned long BlynkDebounce = 100;
const unsigned long BlynkMaxPushInterval = 100; // only allow pushes every 200 millisecond(not more!)
unsigned long lastInteraction;
const unsigned long turnDisplayOffIntervall = 1000 * 30 * 1; // turn display off after 5min
unsigned long LastBlynkSyncTime = 0;
unsigned long EspWebUpdateMillis = 0;
unsigned long BlynkLastTime = 0;
const long intervalForSetup = 1000 * 4;                  // 60 seconds for Setup
const unsigned long timePeriodWebUpdate = 5 * 60 * 1000; // 5mins
bool AppConnected = false;
bool debug = false;

// flag for saving data
bool shouldSaveConfig = false;
bool shouldDoWebUpdate = false;
// callback notifying us of the need to save config
void saveConfigCallback()
{
  shouldSaveConfig = true;
}

DHT dht(DHTPIN, DHTTYPE);

WiFiServer wifiServer(80);

Ticker TimerOutput;
Ticker TimerConnect;
// Ticker TimerBlynk;
Ticker TimerUpdateData;
Ticker TimerOTA;
Ticker TimerReadDHT;
Ticker TimerDisplay;
Ticker TimerChangeHeatIcon;
// Ticker TimerChangeDisplay;

BLYNK_CONNECTED()
{
  Blynk.syncAll();
}
BLYNK_APP_CONNECTED()
{
  // lastInteraction = millis();
  AppConnected = true;
  opStateEspStack.push(ESP_SYNC_BLYNK);
}

// This is called when Smartphone App is closed
BLYNK_APP_DISCONNECTED()
{
  AppConnected = false;
}
BLYNK_WRITE(BLYNKUPDATE) // Button Widget is writing to pin V1
{
  BlynkLastTime = millis();
  int pinData = param.asInt();
  if (pinData == 1)
  {
    opStateEspStack.push(ESP_WEBUPDATE);
  }
}
BLYNK_WRITE(BLYNKSETINPUT) // Button Widget is writing to pin V1
{
  lastInteraction = millis();
  double pinTemp = param.asDouble();
  if (pinTemp <= MaxSetTemp && pinTemp >= MinSetTemp)
  {
    BlynkLastTime = millis();
    SetPointFunction(0, pinTemp);
  }
}

void setup()
{
  initHardware();
  delay(500);
  dht.begin();
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  if (IS_SERIAL)
  {
    Serial.begin(115200);
  }

  delay(10);
  initSoftware();
  initSpiffs();
  initializeDisplay();
  opStateEsp = ESP_STANDBY;
  opStateThermostat = STANDBY;
  windowStartTime = millis();
  // initialize PID Setpoint
  SetPointFunction(0, SetTemp);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetGapResetITerm(GapResetITerm);
  // tell the PID to range between 0 and the full window size
  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  TimerConnect.attach(timeConnectInterval, ConnectFlag);
  TimerOTA.attach(timeOTA, OTAFlag);
  TimerReadDHT.attach(timeReadDHT, ReadDHTFlag);
  // TimerDisplay.attach_ms(timeDisplayIntervalMs, DisplayFlag);
  // TimerChangeDisplay.attach(timeChangeDisplayInterval, ChangeDisplayFlag);
  TimerChangeHeatIcon.attach(timeChangeHeatIcon, ChangeHeatIconFlag);
  TimerOutput.attach(timeDriveOutput, DriveOutputFlag);
  TimerUpdateData.attach(timeUpdateData, UpdateDataFlag);
  opStateEspStack.push(ESP_CONNECT);
  opStateEspStack.push(ESP_READDHT);
  SetPointFunction(0, storage.getTempSetpoint());
}

void EspStandby()
{

  if (opStateEspStack.count() > 1000)
  {
    ESP.restart();
  }

  if (!opStateEspStack.isEmpty())
  {
    opStateEsp = opStateEspStack.pop();
  }
}

void loop()
{

  handleBlynkAndUpdate();
  if (!SetupModeStarted)
  {
    TFTFunction();
    ReadButtons();

    unsigned long currentMillisStateMachine = millis();
    if (currentMillisStateMachine - previousMillisStateEsp >= EspStateMachineInterval)
    {
      previousMillisStateEsp = currentMillisStateMachine;
      switch (opStateEsp)
      {
      case ESP_STANDBY:
        EspStandby();
        break;
      case ESP_READDHT:
        ReadDHTFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_CONNECT:
        ConnectFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_DISPLAY:
        // TFTFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_CHANGE_DISPLAY:
        // ChangeDisplayFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_LCDOFF:
        DisplayOffFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_LCDON:
        DisplayOnFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_SETUP:
        // SetupFunction();
        SmartWiFiSetupFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_WEBUPDATE:
        EspWebUpdate();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_RESTART:
        RestartFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_SYNC_BLYNK:
        EspSyncBlynk();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_SYNC_POWER:
        EspSyncPower();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_SYNC_READINGS:
        EspSyncReadings();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_SYNC_SETPOINT:
        EspSyncSetpoint();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_OTA:
        EspOTAFunction();
        opStateEsp = ESP_STANDBY;
        break;
      case ESP_UPDATEDATA:
        updateNTP();
        opStateEsp = ESP_STANDBY;
        break;
      }
    }

    if (currentMillisStateMachine - previousMillisThermostat >= ThermostatStateMachineInterval)
    {
      previousMillisThermostat = currentMillisStateMachine;
      switch (opStateThermostat)
      {
      case STANDBY:
        Standby();
        break;
      case DRIVE_OUTPUT:
        DriveOutput();
        opStateThermostat = STANDBY;
        break;
      case INCREASE_SETPOINT:
        SetPointFunction(0.5, 0);
        opStateThermostat = STANDBY;
        break;
      case DECREASE_SETPOINT:
        SetPointFunction(-0.5, 0);
        opStateThermostat = STANDBY;
        break;
      }
    }
  }
  else if (SmartWifiSetupStarted)
  {
    delay(500);
  }
  yield();
}

void ReadButtons()
{
  unsigned long currentMillis = millis();
  int readingUp = digitalRead(ButtonUp);
  int readingDown = digitalRead(ButtonDown);

  if (readingUp == HIGH && lastButtonStateUp == LOW && (currentMillis - debounceTimeUp > debounceInterval))
  {
    lastInteraction = currentMillis;
    timePressedUp = millis();
    debounceTimeUp = currentMillis;
    // opStateThermoStack.push(INCREASE_SETPOINT);
    if (DisplayState == DISPLAY_ON)
    {
      SetPointFunction(0.5f, 0);
      opStateEspStack.push(ESP_SYNC_SETPOINT);
    }
  }
  if (readingDown == HIGH && lastButtonStateDown == LOW && (currentMillis - debounceTimeDown > debounceInterval))
  {
    lastInteraction = currentMillis;
    timePressedDown = millis();
    debounceTimeDown = currentMillis;
    if (DisplayState == DISPLAY_ON)
    {
      SetPointFunction(-0.5f, 0);
      opStateEspStack.push(ESP_SYNC_SETPOINT);
    }
  }
  if (readingDown == HIGH && lastButtonStateDown == HIGH && (currentMillis - debounceTimeDown > debounceInterval) && (currentMillis - timePressedDown > intervalForSetup))
  {
    lastInteraction = currentMillis;
    debounceTimeDown = currentMillis;
    opStateEspStack.push(ESP_SETUP);
  }
  if (readingUp == HIGH && lastButtonStateUp == HIGH && (currentMillis - debounceTimeUp > debounceInterval) && (currentMillis - timePressedUp > intervalForSetup))
  {
    lastInteraction = currentMillis;
    debounceTimeUp = currentMillis;
    opStateEspStack.push(ESP_SETUP);
  }

  lastButtonStateUp = readingUp;
  lastButtonStateDown = readingDown;
}

void Standby()
{
  // read Buttons
  if (!opStateThermoStack.isEmpty())
  {
    opStateThermostat = opStateThermoStack.pop();
  }
}
void DriveOutput()
{
  SetpointPID = Setpoint;
  InputPID = TemperatureSamples.getMedian();
  myPID.Compute();
  onTime = OutputPID;
  OutputPercent = (OutputPID / WindowSize) * 100;

  unsigned long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if (now - windowStartTime > WindowSize)
  { // time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if ((onTime >= minimumOnTime) && (onTime >= (now - windowStartTime)))
  {
    Output = 255;
    digitalWrite(RELAY_PIN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
    Output = 0;
  }
  opStateEspStack.push(ESP_SYNC_POWER);
}
void TFTFunction()
{
  CheckDisplayState();
  if (DisplayState == DISPLAY_ON)
  {
    drawThermostat();
    drawFire();
    drawTime();
    drawWiFiQuality();
    gfx.commit();
  }
}

void DisplayOffFunction()
{
  DisplayState = DISPLAY_OFF;
  analogWrite(TFT_LED, LOWDISPBRIGHTNESS);
}

void DisplayOnFunction()
{
  DisplayState = DISPLAY_ON;
  analogWrite(TFT_LED, HIGHDISPBRIGHTNESS); // HIGH to Turn on;
}

void initSpiffs()
{
  if (SPIFFS.begin())
  {
    if (SPIFFS.exists("/config.json"))
    {
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        size_t size = configFile.size();

        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success())
        {

          delay(20);
          if (json.containsKey("blynk_token"))
          {
            strcpy(blynk_token, json["blynk_token"]);
          }
          Serial.print("Bynk token");
          Serial.print(blynk_token);
          Serial.print("SSID");
          Serial.print(WiFi.SSID());
          Serial.print("PASS");
          Serial.print(WiFi.psk());
        }
        else
        {
          delay(20);
          Serial.print("failed load json");
          delay(20);
        }
      }
    }
  }
  else
  {
    delay(20);
    Serial.print("failed mount FS");
    delay(20);
  }
  delay(10);
}

void ConnectFunction()
{
  // Serial.println(WiFi.status());
  if (WiFi.status() == WL_CONNECTED)
  {
    isWifiConnected = true;
    // timeClient.update();
    if (!Blynk.connected())
    {
      isBlynkConnected = false;
      Blynk.config(blynk_token, PLUTOSERVERURL, PLUTOPORT);
      Blynk.connect(PLUTOTIMEOUT);
      Serial.print("Blynk not connected,");
      Serial.println(blynk_token);
    }
    else
    {
      Serial.print("Blynk Connected,");
      Serial.println(blynk_token);
      isBlynkConnected = true;
    }
  }
  else
  {
    isWifiConnected = false;
    isBlynkConnected = false;
    WiFi.begin();
  }

  opStateEsp = ESP_STANDBY;
  long val = millis() * 0.001;
  int days = elapsedDays(val);
  if (days >= 7)
  {
    opStateEspStack.push(ESP_RESTART);
  }
}

void DriveOutputFlag()
{
  opStateThermoStack.push(DRIVE_OUTPUT);
}
void DisplayFlag()
{
}

void ChangeDisplayFlag()
{
  opStateEspStack.push(ESP_CHANGE_DISPLAY);
}

void ConnectFlag()
{

  opStateEspStack.push(ESP_CONNECT);
}

void OTAFlag()
{
  opStateEspStack.push(ESP_OTA);
}

void UpdateDataFlag()
{
  opStateEspStack.push(ESP_UPDATEDATA);
}

void ReadDHTFlag()
{
  opStateEspStack.push(ESP_READDHT);
}

void BlynkFlag()
{

  opStateEspStack.push(ESP_SYNC_BLYNK);
}

void EspWebUpdate()
{
  EspWebUpdateMillis = millis();
  const char *host = "esp8266-webupdate";
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin();

  MDNS.begin(host);

  httpUpdater.setup(&httpServer);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  shouldDoWebUpdate = true;
  // opStateEsp = ESP_STANDBY;
}

void handleBlynkAndUpdate()
{
  CheckSmartWifiState();
  ListenTCP();
  if (Blynk.connected() && !SetupModeStarted)
  {
    Blynk.run();
  }
}

void SetupFunction()
{
  /*
  opStateEsp = ESP_STANDBY;
  // TimerBlynk.detach();
  // TimerChangeDisplay.detach();
  TimerConnect.detach();
  TimerOTA.detach();
  TimerOutput.detach();
  TimerReadDHT.detach();
  TimerUpdateData.detach();

  //TimerDisplay.detach();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_blynk_token("blynk", "Blynk Token", blynk_token, 40);

  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_blynk_token);

  wifiManager.setTimeout(600); //600 seconds timeout

  //Timeout
  if (!wifiManager.startConfigPortal("CoziThermostat", "12345678"))
  {
    delay(20);
    return;
  }
  //if you get here you have connected to the WiFi
  delay(10);
  //read updated parameters
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      delay(20);
    }
    json.printTo(configFile);
    configFile.close(); //save successful
    delay(100);
    ESP.restart();
  }
  */
}

void SaveServerToken(String serverToken)
{
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  json["blynk_token"] = serverToken;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    delay(20);
  }
  json.printTo(configFile);
  configFile.close(); // save successful
  delay(100);
  ESP.reset();
}

void CheckSmartWifiState()
{
  if (SmartWifiSetupStarted)
  {
    if (WiFi.smartConfigDone())
    {
      SmartWifiSetupStarted = false;
      SmartWifiConfigDone = true;
      Serial.println("SmartConfig Success");
      Serial.println(WiFi.SSID());
      Serial.println(WiFi.psk());
      WiFi.begin();
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(100);
        Serial.println("Connecting..");
      }

      Serial.print("Connected to WiFi. IP:");
      Serial.println(WiFi.localIP());
      wifiServer.begin();
    }
  }
  if (SetupModeStarted == true && (millis() - timeEnteredSetupMode) >= SmartWifiTimeout)
  {
    ESP.reset();
  }
}

void SmartWiFiSetupFunction()
{
  opStateEsp = ESP_STANDBY;
  // WiFi.disconnect();
  // SPIFFS.format();
  SmartWifiSetupStarted = true;
  SetupModeStarted = true;
  timeEnteredSetupMode = millis();
  TimerDisplay.detach();
  TimerConnect.detach();
  TimerOTA.detach();
  TimerOutput.detach();
  TimerReadDHT.detach();
  TimerUpdateData.detach();
  DisplaySetupMode();
  WiFi.beginSmartConfig();
}

void ListenTCP()
{
  if (SmartWifiConfigDone)
  {
    WiFiClient client = wifiServer.available();

    if (client)
    {

      while (client.connected())
      {

        if (client.available() > 0)
        {

          receivedChar = client.read();
          Serial.println(receivedChar);
          if (receivedChar == startMarker)
          {
            Serial.print("Data in progress");
            DataInProgress = true;
          }
          else
          {
            if (DataInProgress)
            {
              if (receivedChar != endMarker)
              {
                if (indexTCP <= MaxSerialIndex)
                {
                  ReceivedData[indexTCP] = receivedChar;
                  Serial.print(indexTCP);
                }
                indexTCP++;
              }
              else
              {
                Serial.print("Data ended");
                Serial.print("indexTCP");
                if (indexTCP == MaxSerialIndex + 1)
                {
                  Serial.println("Data Ready");
                  ReceivedData[32] = '\0';
                  Serial.println(ReceivedData);
                  DataCorrupted = false;
                  DataReady = true;
                  String str(ReceivedData);
                  // String data = ReceivedData;
                  SmartWifiConfigDone = false;
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  client.println("OK");
                  client.flush();
                  CheckTcpData(str);
                  return;
                }
                else
                {
                  Serial.print("Data Corrupted");
                  DataCorrupted = true;
                  DataReady = false;
                }
                DataInProgress = false;
                indexTCP = 0;
              }
            }
          }
        }
      }
    }
  }
}

void sendOK()
{
  WiFiClient client = wifiServer.available();

  if (client)
  {

    if (client.connected())
    {
      client.print("OK");
      client.flush();
      client.print("OK");
      client.flush();
      client.print("OK");
      client.flush();
    }
  }
}

void CheckTcpData(String data)
{

  Serial.println("datalength");
  Serial.println(data.length());
  Serial.println(data);
  if (data.length() == DataLength)
  {
    Serial.print("check tcp ok");
    sendOK();
    sendOK();
    sendOK();
    sendOK();
    SaveServerToken(data);

    // Token is OK --> save
    // sendOKTicker
  }
}

void RestartFunction()
{
  // opStateEsp = ESP_STANDBY;
  ESP.restart();
}

void EspSyncBlynk()
{
  opStateEsp = ESP_STANDBY;
  if (Blynk.connected())
  {
    unsigned long currentMillis = millis();

    if ((currentMillis - BlynkLastTime >= BlynkDebounce) && (currentMillis - LastBlynkSyncTime >= BlynkMaxPushInterval))
    {
      LastBlynkSyncTime = millis();
      Blynk.virtualWrite(BLYNKTEMPERATURE, TemperatureSamples.getMedian());
      Blynk.virtualWrite(BLYNKHUMIDITY, HumiditySamples.getMedian());
      Blynk.virtualWrite(BLYNKLED, Output);
      Blynk.virtualWrite(BLYNKSETINPUT, Setpoint);
    }
  }
}

void ReadDHTFunction()
{
  HumidityReading = dht.readHumidity();
  // Read temperature as Celsius (the default)
  TemperatureReading = dht.readTemperature();
  if (isnan(HumidityReading) || isnan(TemperatureReading))
  {
    FailedReadings++;
    Serial.println("Failed to read from DHT sensor!");
    if (FailedReadings >= 20)
    {
      TemperatureSamples.add(99.9f);
      HumiditySamples.add(99.9f);
    }
  }
  else
  {
    FailedReadings = 0;
    TemperatureSamples.add(TemperatureReading + TempAdjustment);
    HumiditySamples.add(HumidityReading);
    opStateEspStack.push(ESP_SYNC_READINGS);
  }
}

void EspOTAFunction()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    // Serial.println("OTA check");
    // t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateWebAdress + MODEL + UNIT_NUMBER + SOFTWARE_VERSION + ".bin");
    /*
        switch (ret)
        {
        case HTTP_UPDATE_FAILED:
          //USE_SERIAL.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          //USE_SERIAL.println("HTTP_UPDATE_NO_UPDATES");
          break;

        case HTTP_UPDATE_OK:
          //USE_SERIAL.println("HTTP_UPDATE_OK");
          break;
        }*/
  }
}

void initializeDisplay()
{
  // Initialize the driver only once
  gfx.init();
  // fill the buffer with black
  gfx.fillBuffer(MINI_BLACK);
  // write the buffer to the display
  gfx.commit();
  gfx.setRotation(3);
}

void SetPointFunction(float increaseby, float setvalue)
{
  if (setvalue == 0)
  {
    if (Setpoint < MaxSetTemp && increaseby > 0)
    {
      Setpoint += increaseby;
    }
    if (Setpoint > MinSetTemp && increaseby < 0)
    {
      Setpoint += increaseby;
    }
    storage.saveTempSetpoint(&Setpoint);
  }
  if (setvalue != 0)
  {
    if (setvalue >= MinSetTemp && setvalue <= MaxSetTemp)
    {
      Setpoint = setvalue;
    }
  }
}
void initHardware()
{
  pinMode(DHTPIN, INPUT);     // Initialize the Relay pin as an output
  pinMode(RELAY_PIN, OUTPUT); // Initialize the Relay pin as an output
  digitalWrite(RELAY_PIN, LOW);
  pinMode(TFT_LED, OUTPUT);
  pinMode(D3, OUTPUT);
  analogWriteRange(100);
  digitalWrite(TFT_LED, LOW); // HIGH to Turn on;
  digitalWrite(D3, LOW);      // HIGH to Turn on;
  pinMode(ButtonDown, INPUT);
  pinMode(ButtonUp, INPUT);
}
void initSoftware()
{
  TemperatureSamples.add(99.9f);
  HumiditySamples.add(99.9f);
}

void updateNTP()
{
  if (WiFi.isConnected())
  {
    configTime(UTC_OFFSET * 3600, 0, NTP_SERVERS);
    if (time(nullptr))
    {
      dstOffset = UTC_OFFSET * 3600 + dstAdjusted.time(nullptr) - time(nullptr);
    }
    // calculate for time calculation how much the dst class adds.

    //  Serial.printf("Time difference for DST: %d", dstOffset);
  }
}

void printTime(time_t offset)
{
  char buf[30];
  char *dstAbbrev;
  time_t t = dstAdjusted.time(&dstAbbrev) + offset;
  struct tm *timeinfo = localtime(&t);

  int hour = (timeinfo->tm_hour + 11) % 12 + 1; // take care of noon and midnight
  sprintf(buf, "%02d/%02d/%04d %02d:%02d:%02d%s %s\n", timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_year + 1900, hour, timeinfo->tm_min, timeinfo->tm_sec, timeinfo->tm_hour >= 12 ? "pm" : "am", dstAbbrev);
  // Serial.print(buf);
}
int8_t getWifiQuality()
{
  int32_t dbm = WiFi.RSSI();
  if (dbm <= -100)
  {
    return 0;
  }
  else if (dbm >= -50)
  {
    return 100;
  }
  else
  {
    return 2 * (dbm + 100);
  }
}
void drawWiFiQuality()
{
  int8_t quality = getWifiQuality();
  gfx.setColor(MINI_WHITE);
  gfx.setFont(Commodore_64_Angled_30);
  gfx.setTextAlignment(TEXT_ALIGN_RIGHT);
  if (WiFi.isConnected())
  {
    if (quality > 0 && quality < 25)
    {
      gfx.drawPalettedBitmapFromPgm(280, 10, WiFiVeryWeak32);
    }
    if (quality >= 25 && quality < 50)
    {
      gfx.drawPalettedBitmapFromPgm(280, 10, WiFiWeak32);
    }
    if (quality >= 50 && quality < 75)
    {
      gfx.drawPalettedBitmapFromPgm(280, 10, wifiMedium32);
    }
    if (quality >= 75 && quality <= 100)
    {
      gfx.drawPalettedBitmapFromPgm(280, 10, WiFiStrong32);
    }
  }
}
void drawTime()
{

  char time_str[11];
  char *dstAbbrev;
  time_t now = dstAdjusted.time(&dstAbbrev);
  struct tm *timeinfo = localtime(&now);

  gfx.setTextAlignment(TEXT_ALIGN_CENTER);
  gfx.setFont(FFF_Forward_30);
  gfx.setColor(MINI_WHITE);
  String date = ctime(&now);
  date = date.substring(0, 11) + String(1900 + timeinfo->tm_year);
  // gfx.drawString(120, 6, date);

  gfx.setFont(Commodore_64_Angled_30);

  if (IS_STYLE_12HR)
  {
    int hour = (timeinfo->tm_hour + 11) % 12 + 1; // take care of noon and midnight
    // sprintf(time_str, "%2d:%02d:%02d\n", hour, timeinfo->tm_min, timeinfo->tm_sec);
    sprintf(time_str, "%2d:%02d\n", hour, timeinfo->tm_min);
    gfx.drawString(160, 10, time_str);
  }
  else
  {
    // sprintf(time_str, "%02d:%02d:%02d\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    sprintf(time_str, "%02d:%02d\n", timeinfo->tm_hour, timeinfo->tm_min);
    gfx.drawString(160, 10, time_str);
  }

  // gfx.setTextAlignment(TEXT_ALIGN_LEFT);
  // gfx.setFont(ArialMT_Plain_10);
  // gfx.setColor(MINI_BLUE);
  if (IS_STYLE_12HR)
  {
    sprintf(time_str, "%s\n%s", dstAbbrev, timeinfo->tm_hour >= 12 ? "PM" : "AM");
    // gfx.drawString(195, 27, time_str);
  }
  else
  {
    sprintf(time_str, "%s", dstAbbrev);
    // gfx.drawString(195, 27, time_str); // Known bug: Cuts off 4th character of timezone abbreviation
  }
}

void ChangeHeatIconFlag()
{
  if (fireIconLarge)
  {
    fireIconLarge = false;
  }
  else
  {
    fireIconLarge = true;
  }
}
void drawFire()
{
  if (Output != 0)
  {
    gfx.setFont(ArialRoundedMTBold_14);
    gfx.setTextAlignment(TEXT_ALIGN_CENTER);
    gfx.setColor(MINI_WHITE);
    gfx.drawString(280, 140, String(OutputPercent, 1) + "%");
    if (fireIconLarge)
    {
      gfx.drawPalettedBitmapFromPgm(260, 170, fire52);
    }
    else
    {
      gfx.drawPalettedBitmapFromPgm(273, 183, fire26);
    }
  }
}

void drawThermostat()
{
  gfx.fillBuffer(MINI_BLACK);
  gfx.setFont(Commodore_64_Angled_48);
  // gfx.setFont(forward);
  gfx.setTextAlignment(TEXT_ALIGN_CENTER);
  gfx.setColor(MINI_WHITE);
  gfx.drawString(160, 80, String(Setpoint, 1) + (IS_METRIC ? "°C" : "°F"));

  // gfx.drawString(160, 100, "20.2");
  // gfx.drawx
  gfx.drawPalettedBitmapFromPgm(6, 170, thermometerIcon64);
  gfx.setFont(Commodore_64_Angled_30);
  // gfx.setTextAlignment(TEXT_ALIGN_LEFT);

  gfx.setTextAlignment(TEXT_ALIGN_CENTER);
  gfx.setColor(MINI_WHITE);
  gfx.drawString(160, 160, String(TemperatureSamples.getMedian(), 1) + (IS_METRIC ? "°C" : "°F"));

  // gfx.drawPalettedBitmapFromPgm(250, 170, humidityIcon64);
  gfx.setFont(Commodore_64_Angled_30);
  // gfx.setTextAlignment(TEXT_ALIGN_RIGHT);
  gfx.setTextAlignment(TEXT_ALIGN_CENTER);
  gfx.setColor(MINI_WHITE);
  gfx.drawString(160, 200, String(HumiditySamples.getMedian(), 1) + "%");

  if (FailedReadings > 0)
  {
    gfx.setFont(Commodore_64_Angled_30);
    // gfx.setTextAlignment(TEXT_ALIGN_RIGHT);
    gfx.setTextAlignment(TEXT_ALIGN_LEFT);
    gfx.setColor(MINI_RED);
    // gfx.drawString(5, 5, String(FailedReadings)+"!");
  }
}
void EspSyncPower()
{
  if (Blynk.connected())
  {
    unsigned long currentMillis = millis();

    if ((currentMillis - BlynkLastTime >= BlynkDebounce) && (currentMillis - LastBlynkSyncTime >= BlynkMaxPushInterval))
    {
      LastBlynkSyncTime = millis();

      Blynk.virtualWrite(BLYNKLED, Output);
    }
  }
}
void EspSyncReadings()
{
  if (Blynk.connected())
  {
    unsigned long currentMillis = millis();

    if ((currentMillis - BlynkLastTime >= BlynkDebounce) && (currentMillis - LastBlynkSyncTime >= BlynkMaxPushInterval))
    {
      LastBlynkSyncTime = millis();
      Blynk.virtualWrite(BLYNKTEMPERATURE, TemperatureSamples.getMedian());
      Blynk.virtualWrite(BLYNKHUMIDITY, HumiditySamples.getMedian());
    }
  }
}
void EspSyncSetpoint()
{
  if (Blynk.connected())
  {
    unsigned long currentMillis = millis();

    if ((currentMillis - BlynkLastTime >= BlynkDebounce) && (currentMillis - LastBlynkSyncTime >= BlynkMaxPushInterval))
    {
      LastBlynkSyncTime = millis();
      Blynk.virtualWrite(BLYNKSETINPUT, Setpoint);
    }
  }
}
void CheckDisplayState()
{
  if (millis() - lastInteraction > turnDisplayOffIntervall)
  {
    analogWrite(TFT_LED, LOWDISPBRIGHTNESS);
    DisplayState = DISPLAY_OFF;
  }
  else if (millis() - lastInteraction <= turnDisplayOffIntervall)
  {
    analogWrite(TFT_LED, HIGHDISPBRIGHTNESS);
    DisplayState = DISPLAY_ON;
  }
}

void DisplaySetupMode()
{
  gfx.fillBuffer(MINI_BLACK);
  gfx.setFont(Commodore_64_Angled_30);
  // gfx.setFont(forward);
  gfx.setTextAlignment(TEXT_ALIGN_CENTER);
  gfx.setColor(MINI_WHITE);
  gfx.drawString(160, 40, "SETUP MODE");
  gfx.setTextAlignment(TEXT_ALIGN_CENTER);
  gfx.setColor(MINI_WHITE);
  gfx.drawString(160, 120, "ENTER APP");
  gfx.commit();
}