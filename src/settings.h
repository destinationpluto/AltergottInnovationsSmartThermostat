/**The MIT License (MIT)
Copyright (c) 2015 by Daniel Eichhorn
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
See more at http://blog.squix.ch
*/

#include <simpleDSTadjust.h>

const String UpdateWebAdress = "http://altergottinnovations.000webhostapp.com/update/";
const char MODEL = 'B';
const String UNIT_NUMBER = "0001";
const String SOFTWARE_VERSION = "V2005";//"V3000";

//const IPAddress PLUTOIP(3,122,33,94);
#define PLUTOSERVERURL "plutosmarthome.com"
#define PLUTOPORT 8080
#define PLUTOTIMEOUT 1000

const int LOWDISPBRIGHTNESS = 0;
const int HIGHDISPBRIGHTNESS = 100;


// ************************************************
// ************PID********************************
// ************************************************
const float TempAdjustment = -2.4;
const double MinSetTemp = 5.0;
const double MaxSetTemp = 35.0;
const double GapResetITerm = 3;
double OutputPercent = 0;
//Specify the links and initial tuning parameters
double Kp = 200000, Ki = 50, Kd = 0;
//const double TempThresholdAggrON = 3; // Setpoint - TemperatureMedian > TempThresholdAggrON  -> HEAT ON
//const double TempThresholdAggrOFF = 0.5;    // TemperatureMedian - Setpoint > TempThresholdAggrON  -> HEAT OFF
const unsigned long minimumOnTime = 40000;


const unsigned long WindowSize = 1000 * 60 * 10;
/* Feather Huzzah + 2.4" TFT wing */
// Pins for the ILI9341

// ************************************************
// ************PINS_V2********************************
// ************************************************

#define TFT_DC D4//2//D4
#define TFT_CS D1//5//D1
#define TFT_LED RX//3//RX
//#define TFT_LED D2
//#define DHTPIN D3 // v2 PCB
#define DHTPIN D6 //v3 PCB
#define RELAY_PIN D8//15//D8
#define ButtonUp D2//4//D2
#define ButtonDown D0//16//D0

#define BLYNKTERMINAL V4
#define BLYNKSETINPUT V0
#define BLYNKLED V1
#define BLYNKTEMPERATURE V2
#define BLYNKHUMIDITY V3
#define BLYNKUPDATE V10

// ************************************************
// ************SmartConfig*Variables********************
// ************************************************
//SoftwareSerial Nano(Rx, Tx);
boolean SetupModeStarted = false;
boolean SmartWifiSetupStarted = false;
boolean SmartWifiConfigDone = false;
const unsigned long SmartWifiTimeout = 5 * 60 * 1000;
unsigned long timeEnteredSetupMode = 0;
const int MaxSerialIndex = 31;
char ReceivedData[MaxSerialIndex+2];
char receivedChar;
int indexTCP = 0;
const char startMarker = '<';
const char endMarker = '>';
const int DataLength = 32;
boolean DataReady = false;
boolean DataInProgress = false;
boolean DataCorrupted = false;
//int index = 0;
// OpenWeatherMap Settings
// Sign up here to get an API key:
// https://home.openweathermap.org/users/sign_up
const boolean IS_METRIC = true;
const boolean IS_SERIAL = false;
String OPEN_WEATHER_MAP_APP_ID = "48677113ca1fddc7c18c4dc229ea1eba";
String OPEN_WEATHER_MAP_LOCATION = "Wüstenrot,DE";

// Pick a language code from this list:
// Arabic - ar, Bulgarian - bg, Catalan - ca, Czech - cz, German - de, Greek - el,
// English - en, Persian (Farsi) - fa, Finnish - fi, French - fr, Galician - gl,
// Croatian - hr, Hungarian - hu, Italian - it, Japanese - ja, Korean - kr,
// Latvian - la, Lithuanian - lt, Macedonian - mk, Dutch - nl, Polish - pl,
// Portuguese - pt, Romanian - ro, Russian - ru, Swedish - se, Slovak - sk,
// Slovenian - sl, Spanish - es, Turkish - tr, Ukrainian - ua, Vietnamese - vi,
// Chinese Simplified - zh_cn, Chinese Traditional - zh_tw.

String OPEN_WEATHER_MAP_LANGUAGE = "en";
const uint8_t MAX_FORECASTS = 10;

// Adjust according to your language
const String WDAY_NAMES[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
const String MONTH_NAMES[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
const String MOON_PHASES[] = {"New Moon", "Waxing Crescent", "First Quarter", "Waxing Gibbous",
                              "Full Moon", "Waning Gibbous", "Third quarter", "Waning Crescent"};

#define UTC_OFFSET +1
struct dstRule StartRule = {"CEST", Last, Sun, Mar, 2, 3600}; // Central European Summer Time = UTC/GMT +2 hours
struct dstRule EndRule = {"CET", Last, Sun, Oct, 2, 0};       // Central European Time = UTC/GMT +1 hour

// Settings for Boston
// #define UTC_OFFSET -5
// struct dstRule StartRule = {"EDT", Second, Sun, Mar, 2, 3600}; // Eastern Daylight time = UTC/GMT -4 hours
// struct dstRule EndRule = {"EST", First, Sun, Nov, 1, 0};       // Eastern Standard time = UTC/GMT -5 hour
/* Useful Constants */
//#define SECS_PER_MIN (60UL)
//#define SECS_PER_HOUR (3600UL)
//#define SECS_PER_DAY (SECS_PER_HOUR * 24L)

// Change for 12 Hour/ 24 hour style clock
bool IS_STYLE_12HR = false;

// change for different ntp (time servers)
#define NTP_SERVERS "0.ch.pool.ntp.org", "1.ch.pool.ntp.org", "2.ch.pool.ntp.org"
// #define NTP_SERVERS "us.pool.ntp.org", "time.nist.gov", "pool.ntp.org"
/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) ((_time_ % SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) (_time_ / SECS_PER_DAY)

// ************************************************
// ************TFT********************************
// ************************************************
#define MINI_BLACK 0
#define MINI_WHITE 1
#define MINI_BLUE 2
#define MINI_RED 3

#define MAX_FORECASTS 12

// ************************************************
// Timer Values
// ************************************************
const float timeDriveOutput = 1;         //500ms
const float timeUpdateData = 60;          //60secs
const float timeConnectInterval = 30 * 1; //5mins
//const float timeSyncBlynk = 30;            //30sec
const float timeOTA = 60 * 10;       //10mins
const float timeReadDHT = 10;       //30sec
const float timeChangeHeatIcon = 1; //30sec


// ************************************************
// Sensor & PID Values
// ************************************************
float TemperatureReading;
float HumidityReading;
long FailedReadings = 0;
int oldTemp;
int Output = 0;
double SetTemp = 20; //initial setpoint
//Define Variables we'll be connecting to
float Setpoint = 20;
double SetpointPID = 20, InputPID = 20, OutputPID = 0;
PID myPID(&InputPID, &OutputPID, &SetpointPID, Kp, Ki, Kd, DIRECT);
unsigned long onTime = 0;
unsigned long windowStartTime;

// ************************************************
// ****************Capacitive*Button***************
// ************************************************
int lastButtonStateUp = LOW;
int lastButtonStateDown = LOW;
unsigned long timePressedUp;
unsigned long timePressedDown;
unsigned long debounceTimeUp;
unsigned long debounceTimeDown;
const unsigned long debounceInterval = 50;



/***************************
 * End Settings
 **************************/

