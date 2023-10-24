// ESP 32 Dev Module
// 4 Mb with SPIFSS - 1.2 Mb APP / 1.5 Mb SPIFFS
#include "WiFi.h"
#include <SPI.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "RTClib.h"
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <NeoPixelBus.h>
#include <FS.h>
#include <SD.h>
#include <IRremote.h>
#include "DFRobotDFPlayerMini.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <millisDelay.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "Fsm.h"
#include "RemoteDebug.h"

// LOGGER definition
#define LOGGER_MAX_LEN 128
RemoteDebug Debug;
#define TRACE_NO_REMOTE(format, ...) \
  Serial.printf(format, ##__VA_ARGS__); \
  Serial.println("")
#define TRACE(format, ...) \
  Serial.printf(format, ##__VA_ARGS__); \
  Serial.println(""); \
  remoteDebugTrace(format, ##__VA_ARGS__);
void remoteDebugTrace(const char* format, ...);


// WIFI
#define WIFI_SSID "j-et-j"
#define WIFI_KEY "XXXXXXXXX"
#define CONNECTION_TIMEOUT 10

//
#define MQTT_SERVER "mqtt.beebotte.com"
#define MQTT_PORT 1883
#define MQTT_CHANNEL "pixel"
#define MQTT_TOKEN "token:token_XXXXXXXXXXXXX"
#define MQTT_PWD ""
#define MQTT_KEEP_ALIVE 5
#define MQTT_TIMEOUT 15000
const char CHARS[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
char _id[17];
WiFiClient _espClient;
PubSubClient _mqttClient(_espClient);
// MQTT topics
#define MQTT_RES_CMD "cmd"
#define MQTT_RES_NOTIF "notif"

// RTC
RTC_DS1307 _rtc;

// SD CARD
#define PORT_SD_CS 5
String _currentDir = "/";

//closest NTP Server
#define NTP_SERVER "ntp.laas.fr"
#define GMT_TIME_ZONE 1
WiFiUDP _ntpUDP;
NTPClient _timeClient(_ntpUDP, NTP_SERVER, GMT_TIME_ZONE * 3600, 60000);
// Days
const char* DAYS[] = {
  "Dimanche",
  "Lundi",
  "Mardi",
  "Mercredi",
  "Jeudi",
  "Vendredi",
  "Samedi"
};

// WEBSERVER
WebServer _server(80);
File _fsUploadFile;

// Temp sensor:
#define PORT_ONEWIRE 32
OneWire _oneWire(PORT_ONEWIRE);
DallasTemperature _tempSensor(&_oneWire);
DeviceAddress _insideThermometer;
float _tempC = 0;

// IR remote
#define PORT_IR 35
uint8_t _lastIrCommand;

// MP3 player
#define PORT_RXD2 16
#define PORT_TXD2 17
HardwareSerial _mp3Serial(1);
DFRobotDFPlayerMini _mp3Player;

// art pixel
uint8_t _currentPixelId = 1;

// Which pin on the Arduino is connected to the NeoPixels?
#define PORT_PIXELS 33
#define PORT_LUX 34
#define AVERAGE_LIGHT_MEASURE 10
int _lightMeasures[AVERAGE_LIGHT_MEASURE];
int _lightMeasuresSum = 0;
uint8_t _lightMeasuresIndex = 0;
uint8_t _brightness = 0;
// display switch
#define DISPLAY_FADE_IN 0
#define DISPLAY_FADE_WAIT 1
#define DISPLAY_FADE_OUT 2
uint8_t _displayFadeStatus = DISPLAY_FADE_WAIT;
uint8_t _brightnessTarget = 0;
static void NOTHING(void){};                   // empty function
typedef void (*voidFuncPtr)();                 // Create a type to point to a funciton.
voidFuncPtr _next_display_function = NOTHING;  // function of next display
// matrix definition
#define NUM_PIXELS 256
RgbColor _matrix[16][16];
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> _pixels(NUM_PIXELS, PORT_PIXELS);

const bool NUMBER_l0[] PROGMEM = {
  0, 1, 1, 0,
  1, 0, 0, 1,
  1, 0, 1, 1,
  1, 1, 0, 1,
  1, 0, 0, 1,
  1, 0, 0, 1,
  0, 1, 1, 0
};
const bool NUMBER_l1[] PROGMEM = {
  0, 0, 1, 0,
  0, 1, 1, 0,
  0, 0, 1, 0,
  0, 0, 1, 0,
  0, 0, 1, 0,
  0, 0, 1, 0,
  0, 1, 1, 1
};
const bool NUMBER_l2[] PROGMEM = {
  0, 1, 1, 0,
  1, 0, 0, 1,
  0, 0, 0, 1,
  0, 0, 1, 0,
  0, 1, 0, 0,
  1, 0, 0, 0,
  1, 1, 1, 1
};
const bool NUMBER_l3[] PROGMEM = {
  0, 1, 1, 0,
  1, 0, 0, 1,
  0, 0, 0, 1,
  0, 1, 1, 0,
  0, 0, 0, 1,
  1, 0, 0, 1,
  0, 1, 1, 0
};
const bool NUMBER_l4[] PROGMEM = {
  0, 0, 1, 1,
  0, 1, 0, 1,
  1, 0, 0, 1,
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  0, 0, 0, 1
};
const bool NUMBER_l5[] PROGMEM = {
  1, 1, 1, 1,
  1, 0, 0, 0,
  1, 1, 1, 0,
  0, 0, 0, 1,
  0, 0, 0, 1,
  1, 0, 0, 1,
  0, 1, 1, 0
};
const bool NUMBER_l6[] PROGMEM = {
  0, 0, 1, 0,
  0, 1, 0, 0,
  1, 0, 0, 0,
  1, 1, 1, 0,
  1, 0, 0, 1,
  1, 0, 0, 1,
  0, 1, 1, 0
};
const bool NUMBER_l7[] PROGMEM = {
  1, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 0, 1,
  0, 0, 1, 0,
  0, 1, 0, 0,
  0, 1, 0, 0,
  0, 1, 0, 0
};
const bool NUMBER_l8[] PROGMEM = {
  0, 1, 1, 0,
  1, 0, 0, 1,
  1, 0, 0, 1,
  0, 1, 1, 0,
  1, 0, 0, 1,
  1, 0, 0, 1,
  0, 1, 1, 0
};
const bool NUMBER_l9[] PROGMEM = {
  0, 1, 1, 0,
  1, 0, 0, 1,
  1, 0, 0, 1,
  0, 1, 1, 1,
  0, 0, 0, 1,
  0, 0, 1, 0,
  0, 1, 0, 0
};

const bool NUMBER_s0[] PROGMEM = {
  1, 1, 1,
  1, 0, 1,
  1, 0, 1,
  1, 0, 1,
  1, 1, 1
};
const bool NUMBER_s1[] PROGMEM = {
  0, 1, 0,
  1, 1, 0,
  0, 1, 0,
  0, 1, 0,
  1, 1, 1
};
const bool NUMBER_s2[] PROGMEM = {
  1, 1, 1,
  0, 0, 1,
  0, 1, 1,
  1, 0, 0,
  1, 1, 1
};
const bool NUMBER_s3[] PROGMEM = {
  1, 1, 1,
  0, 0, 1,
  0, 1, 0,
  0, 0, 1,
  1, 1, 1
};
const bool NUMBER_s4[] PROGMEM = {
  1, 0, 1,
  1, 0, 1,
  1, 1, 1,
  0, 0, 1,
  0, 0, 1
};
const bool NUMBER_s5[] PROGMEM = {
  1, 1, 1,
  1, 0, 0,
  1, 1, 1,
  0, 0, 1,
  1, 1, 1
};
const bool NUMBER_s6[] PROGMEM = {
  1, 1, 1,
  1, 0, 0,
  1, 1, 1,
  1, 0, 1,
  1, 1, 1
};
const bool NUMBER_s7[] PROGMEM = {
  1, 1, 1,
  0, 0, 1,
  0, 0, 1,
  0, 0, 1,
  0, 0, 1
};
const bool NUMBER_s8[] PROGMEM = {
  1, 1, 1,
  1, 0, 1,
  1, 1, 1,
  1, 0, 1,
  1, 1, 1
};
const bool NUMBER_s9[] PROGMEM = {
  1, 1, 1,
  1, 0, 1,
  1, 1, 1,
  0, 0, 1,
  1, 1, 1
};

// EEPROM
static const unsigned long STRUCT_MAGIC = 123456789;
static const byte STRUCT_VERSION = 1;
struct EepromStruct {
  unsigned long magic;
  byte struct_version;
  // NAME
  char name[10];
  // NIGHT MODE
  int night_threshold;  // night mode level
  uint8_t night_hour_r;
  uint8_t night_hour_g;
  uint8_t night_hour_b;
  uint8_t night_min_r;
  uint8_t night_min_g;
  uint8_t night_min_b;
  uint8_t night_tempi_r;
  uint8_t night_tempi_g;
  uint8_t night_tempi_b;
  uint8_t night_tempe_r;
  uint8_t night_tempe_g;
  uint8_t night_tempe_b;
  // IR COMMANDS
  uint8_t ir_cmd_temp_sum;  // IR command to display temperature summary
  uint8_t ir_cmd_ip;        // IR command to display IP
  // BRIGHTNESS
  uint8_t bright_max;        // brightness parameter
  uint8_t bright_map[30];    // brightness map
  uint8_t bright_threshold;  // brightness parameter
  // WIFI
  char wifi_ssid[24];
  char wifi_key[24];
  // MQTT
  char mqtt_server[30];
  int mqtt_port;
  char mqtt_channel[12];
  char mqtt_user[30];
  char mqtt_pwd[12];
  char mqtt_res_temp[12];
  // OPENWEATHER
  char weather_appid[40];
  char weather_location[20];
};
EepromStruct _eeprom;

// METEO
#define METEO_CURRENT_API_URL "http://api.openweathermap.org/data/2.5/onecall?%s&exclude=minutely,hourly,alerts,daily&lang=fr&units=metric&appid=%s"
#define METEO_FORECAST_API_URL "http://api.openweathermap.org/data/2.5/onecall?%s&exclude=minutely,hourly,alerts,current&lang=fr&units=metric&appid=%s"
float _currentTemp = 0;       //°C
float _currentFeelsLike = 0;  // °C
float _currentHumidity = 0;   // %
float _currentWindSpeed = 0;  // metre/sec
int _currentWeatherId = 0;
String _currentWeather = "";
String _currentWeatherIcon = "";
float _currentRain = 0;     // mm
float _todayTemp = 0;       //°C
float _todayFeelsLike = 0;  // °C
float _todayHumidity = 0;   // %
float _todayWindSpeed = 0;  // metre/sec
int _todayWeatherId = 0;
String _todayWeather = "";
String _todayWeatherIcon = "";
float _todayRain = 0;          // mm
float _tomorrowTemp = 0;       //°C
float _tomorrowFeelsLike = 0;  // °C
float _tomorrowHumidity = 0;   // %
float _tomorrowWindSpeed = 0;  // metre/sec
int _tomorrowWeatherId = 0;
String _tomorrowWeather = "";
String _tomorrowWeatherIcon = "";
float _tomorrowRain = 0;  // mm

// CMD DATA
char _cmdDataImage[30];
float _cmdDataValue = 0.0;

// STATES
enum fsm_disp_event_e {
  EVENT_DISP_CMD_VALUE,
  EVENT_DISP_TEMP_SUM,
  EVENT_DISP_CONF_IP,
  EVENT_DISP_NIGHT_MODE,
  EVENT_DISP_DAY_MODE
};
State state_disp_loading(&onDisplayLoading, NULL, NULL);
State state_disp_time(&onDisplayTime, NULL, NULL);
State state_disp_temp(&onDisplayTemp, NULL, NULL);
State state_disp_meteo(&onDisplayMeteo, NULL, NULL);
State state_disp_art(&onDisplayArt, NULL, NULL);
State state_disp_temp_sum(&onDisplayNightTemp, NULL, NULL);
State state_disp_cmd_value(&onDisplayCmdValue, NULL, NULL);
State state_disp_night_time(&onDisplayNightTime, NULL, NULL);
State state_disp_night_temp_sum(&onDisplayNightTemp, NULL, NULL);
State state_disp_conf_ip(&onDisplayIp, NULL, NULL);
Fsm fsm(&state_disp_loading);
#define DISPLAY_DURATION 15000
#define DISPLAY_NIGHT_DURATION 60000

// DELAYS
millisDelay _updateWifiDelay;
#define UPDATE_WIFI_DURATION 120000
millisDelay _checkMqttDelay;
#define CHECK_MQTT_DURATION 100
millisDelay _checkMqttConnectionDelay;
#define CHECK_MQTT_CONN_DURATION 5000
millisDelay _checkRemoteDelay;
#define CHECK_REMOTE_DURATION 100
millisDelay _checkWebServerDelay;
#define CHECK_WEB_SERVER_DURATION 100
millisDelay _updateBrightnessDelay;
#define UPDATE_BRIGHTNESS_DURATION 100
millisDelay _checkNightModeDelay;
#define CHECK_NIGHT_MODE_DURATION 1000
millisDelay _runFsmDelay;
#define RUN_FSM_DURATION 100
millisDelay _displayRefreshDelay;
#define DISPLAY_REFRESH_DURATION 5000
millisDelay _checkMeteoDelay;
#define CHECK_METEO_DURATION 3600000
millisDelay _checkNtpDelay;
#define CHECK_NTP_DURATION 3600000
millisDelay _checkRestartDelay;
#define CHECK_RESTART_DURATION 86400000

/*
   SETUP
*/
void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  // Initialise le module RTC - used for TRACE
  _rtc.begin();

  // load EEPROM
  TRACE_NO_REMOTE("> Load EEPROM");
  loadEEPROM();

  // Neopixels init
  TRACE_NO_REMOTE("> Init Neopixels");
  initMatrix();
  initDisplayFsm();
  displayProgress();
  updateProgress(1);

  // init I2C
  TRACE_NO_REMOTE("> Init I2C");
  Wire.begin();
  updateProgress(2);

  // connect WIFI
  TRACE_NO_REMOTE("> Init Wifi");
  connectWifi();
  updateProgress(3);

  // init Remote Debug
  TRACE_NO_REMOTE("> Init debug");
  initDebug();
  updateProgress(4);

  // connect MQTT
  TRACE("> Init MQTT");
  connectMqtt();
  updateProgress(5);

  // NTP init
  TRACE("> Init NTP");
  _timeClient.begin();
  updateProgress(6);

  // connect SD CARD
  TRACE("> Init SD");
  connectSD();
  updateProgress(7);

  // init Temp. sensor
  TRACE("> Init temp. sensor");
  initTempSensor();
  updateProgress(8);

  // init IR
  TRACE("> Init IR remote");
  initIrRemote();
  updateProgress(9);

  // init MP3
  TRACE("> Init MP3");
  initMp3();
  updateProgress(10);

  // init webserver rooting
  TRACE("> Init webserver");
  initWebServer();
  updateProgress(11);

  // finish
  updateProgress(12);
  updateProgress(13);
  updateProgress(14);
  TRACE("-- setup done --");

  // init timers
  _updateWifiDelay.start(0);
  _checkRemoteDelay.start(0);
  _checkMqttDelay.start(0);
  _checkMqttConnectionDelay.start(0);
  _checkWebServerDelay.start(0);
  _updateBrightnessDelay.start(0);
  _checkNightModeDelay.start(0);
  _runFsmDelay.start(0);
  _displayRefreshDelay.start(0);
  _checkMeteoDelay.start(0);
  _checkNtpDelay.start(0);
}

/*
   LOOP
*/
void loop() {

  // remote Debug
  Debug.handle();

  // ----------------------------------
  // UPDATE WIFI
  // ----------------------------------
  if (_updateWifiDelay.justFinished()) {  // don't combine this test with any other condition
    // update wifi
    updateWifi();

    // update timer
    _updateWifiDelay.start(UPDATE_WIFI_DURATION);
  }

  // ----------------------------------
  // CHECK REMOTE LOOP
  // ----------------------------------
  if (_checkRemoteDelay.justFinished()) {  // don't combine this test with any other condition
    // check remote command
    checkRemoteCmd();

    // update timer
    _checkRemoteDelay.start(CHECK_REMOTE_DURATION);
  }

  // ----------------------------------
  // CHECK MQTT CONNECTION LOOP
  // ----------------------------------
  if (_checkMqttConnectionDelay.justFinished()) {  // don't combine this test with any other condition
    // get MQTT messages
    checkMqttConnection();

    // update timer
    _checkMqttConnectionDelay.start(CHECK_MQTT_CONN_DURATION);
  }

  // ----------------------------------
  // CHECK MQTT LOOP
  // ----------------------------------
  if (_checkMqttDelay.justFinished()) {  // don't combine this test with any other condition
    // get MQTT messages
    checkMqtt();

    // update timer
    _checkMqttDelay.start(CHECK_MQTT_DURATION);
  }
  // ----------------------------------
  // UPDATE BRIGHTNESS LOOP
  // ----------------------------------
  if (_updateBrightnessDelay.justFinished()) {  // don't combine this test with any other condition
    // update display brightness
    updateDisplayBrightness();

    // update timer
    _updateBrightnessDelay.start(UPDATE_BRIGHTNESS_DURATION);
  }

  // ----------------------------------
  // CHECK WEB SERVER LOOP
  // ----------------------------------
  if (_checkWebServerDelay.justFinished()) {  // don't combine this test with any other condition
    // webserver client
    _server.handleClient();

    // update timer
    _checkWebServerDelay.start(CHECK_WEB_SERVER_DURATION);
  }

  // ----------------------------------
  // CHECK NIGHT MODE LOOP
  // ----------------------------------
  if (_checkNightModeDelay.justFinished()) {  // don't combine this test with any other condition
    // check night mode
    checkNightMode();

    // update timer
    _checkNightModeDelay.start(CHECK_NIGHT_MODE_DURATION);
  }

  // TODO redisplay & adjust brightness

  // ----------------------------------
  // CHECK FSM LOOP
  // ----------------------------------
  if (_runFsmDelay.justFinished()) {  // don't combine this test with any other condition
    // Call fsm run
    fsm.run_machine();

    // update timer
    _runFsmDelay.start(RUN_FSM_DURATION);
  }

  // ----------------------------------
  // DISPLAY REFRESH LOOP
  // ----------------------------------
  if (_displayRefreshDelay.justFinished()) {  // don't combine this test with any other condition
    // if function found
    if ((_displayFadeStatus == DISPLAY_FADE_WAIT) && (_next_display_function != NOTHING)) {
      // update
      _next_display_function();
    }
    // update timer
    _displayRefreshDelay.start(DISPLAY_REFRESH_DURATION);
  }

  // ----------------------------------
  // CHECK METEO LOOP
  // ----------------------------------
  if (_checkMeteoDelay.justFinished()) {  // don't combine this test with any other condition
    // update meteo
    updateMeteo();

    // update timer
    _checkMeteoDelay.start(CHECK_METEO_DURATION);
  }

  // ----------------------------------
  // CHECK NTP LOOP
  // ----------------------------------
  if (_checkNtpDelay.justFinished()) {  // don't combine this test with any other condition
    // update NTP
    updateNtp();

    // update timer
    _checkNtpDelay.start(CHECK_NTP_DURATION);
  }


  // ----------------------------------
  // CHECK RESTART LOOP
  // ----------------------------------
  if (_checkRestartDelay.justFinished()) {  // don't combine this test with any other condition
    // Reboot
    ESP.restart();

    // update timer
    _checkRestartDelay.start(CHECK_RESTART_DURATION);
  }

  // Give a time for ESP
  yield();
}

/*
   ----------------------- Display -----------------------
*/

/*
   Display manager
*/
void initDisplayFsm() {
  // loading
  fsm.add_timed_transition(&state_disp_loading, &state_disp_time, 0, NULL);

  // loop day
  fsm.add_timed_transition(&state_disp_time, &state_disp_art, DISPLAY_DURATION, NULL);
  fsm.add_timed_transition(&state_disp_art, &state_disp_meteo, DISPLAY_DURATION, NULL);
  fsm.add_timed_transition(&state_disp_meteo, &state_disp_temp, DISPLAY_DURATION, NULL);
  fsm.add_timed_transition(&state_disp_temp, &state_disp_time, DISPLAY_DURATION, NULL);

  // loop night
  fsm.add_timed_transition(&state_disp_night_temp_sum, &state_disp_night_time, DISPLAY_NIGHT_DURATION, NULL);
  fsm.add_timed_transition(&state_disp_night_time, &state_disp_night_temp_sum, DISPLAY_NIGHT_DURATION, NULL);

  // night mode
  fsm.add_transition(&state_disp_time, &state_disp_night_temp_sum, EVENT_DISP_NIGHT_MODE, NULL);
  fsm.add_transition(&state_disp_art, &state_disp_night_temp_sum, EVENT_DISP_NIGHT_MODE, NULL);
  fsm.add_transition(&state_disp_meteo, &state_disp_night_temp_sum, EVENT_DISP_NIGHT_MODE, NULL);
  fsm.add_transition(&state_disp_temp, &state_disp_night_temp_sum, EVENT_DISP_NIGHT_MODE, NULL);

  // day mode
  fsm.add_transition(&state_disp_night_temp_sum, &state_disp_time, EVENT_DISP_DAY_MODE, NULL);
  fsm.add_transition(&state_disp_night_time, &state_disp_time, EVENT_DISP_DAY_MODE, NULL);
}
void onDisplayLoading() {
  TRACE("DISPLAY switch to progress");
  switchTo(displayProgress);
}
void onDisplayTemp() {
  TRACE("DISPLAY switch to temperature");
  switchTo(displayTemp);
}
void onDisplayTime() {
  TRACE("DISPLAY switch to time");
  switchTo(displayTime);
}
void onDisplayMeteo() {
  TRACE("DISPLAY switch to meteo");
  switchTo(displayMeteo);
}
void onDisplayArt() {
  TRACE("DISPLAY switch to art");
  switchTo(displayArt);
}
void onDisplayNightTemp() {
  TRACE("DISPLAY switch to temperature (night mode)");
  switchTo(displayNightTemp);
}
void onDisplayCmdValue() {
  TRACE("DISPLAY switch to command value");
  switchTo(displayCmdValue);
}
void onDisplayNightTime() {
  TRACE("DISPLAY switch to time (night mode)");
  switchTo(displayNightTime);
}
void onDisplayIp() {
  TRACE("DISPLAY switch to IP");
  switchTo(displayIp);
}

/*
   Page init progress
*/
void displayProgress() {
  RgbColor black = RgbColor(0);
  RgbColor grey = RgbColor(23, 29, 52);
  clearAll(black);
  displayMatrix();
  for (int i = 0; i < 16; ++i) {
    for (int j = 6; j < 11; ++j) {
      colorPixel(i, j, grey);
    }
  }
  colorPixel(0, 6, black);
  colorPixel(0, 10, black);
  colorPixel(15, 6, black);
  colorPixel(15, 10, black);
  displayMatrix();
}
void updateProgress(const uint8_t step) {
  colorPixel(step, 7, RgbColor(random(255), random(255), random(255)));
  colorPixel(step, 8, RgbColor(random(255), random(255), random(255)));
  colorPixel(step, 9, RgbColor(random(255), random(255), random(255)));
  displayMatrix();
}

/*
   Page night time
*/
void displayNightTime() {
  DateTime now = _rtc.now();  //Récupère l'heure et le date courante

  // time
  clearAll(RgbColor(0));
  drawBmp(0, 0, "/common/fond_night_time.bmp");
  drawSmallInt(0, 5, now.hour(), 2, RgbColor(_eeprom.night_hour_r, _eeprom.night_hour_g, _eeprom.night_hour_b));
  drawSmallInt(0, 11, now.minute(), 2, RgbColor(_eeprom.night_min_r, _eeprom.night_min_g, _eeprom.night_min_b));
  displayMatrix();
}

/*
   Page night temperature
*/
void displayNightTemp() {
  float tempInt = getTemperature();

  // temp
  clearAll(RgbColor(0));
  drawBmp(0, 0, "/common/fond_night_temp.bmp");
  // intern
  drawSmallInt(3, 11, (int8_t)round(tempInt), 2, RgbColor(_eeprom.night_tempi_r, _eeprom.night_tempi_g, _eeprom.night_tempi_b));
  // extern
  int8_t tempExt = round(_currentTemp);
  RgbColor color = RgbColor(_eeprom.night_tempe_r, _eeprom.night_tempe_g, _eeprom.night_tempe_b);
  uint8_t pos = 0;
  if (abs(tempExt) < 10) {
    pos = 7;
    drawSmallInt(pos, 0, abs(tempExt), 1, color);
  } else {
    pos = 3;
    drawSmallInt(pos, 0, abs(tempExt), 2, color);
  }
  if (tempExt < 0) {
    colorPixel(pos - 3, 2, color);
    colorPixel(pos - 2, 2, color);
  }
  displayMatrix();

  // no refresh update
  _next_display_function = NOTHING;
}

/*
   Page time
*/
void displayTime() {
  DateTime now = _rtc.now();  //Récupère l'heure et le date courante

  // time
  clearAll(RgbColor(0));
  RgbColor colorText = RgbColor(255, 178, 0);
  drawBmp(0, 0, "/common/fond_time.bmp");
  drawSmallInt(6, 2, now.hour(), 2, colorText);
  colorPixel(3, 10, colorText);
  colorPixel(3, 12, colorText);
  drawSmallInt(6, 9, now.minute(), 2, colorText);
  displayMatrix();
}

/*
   Page temperature
*/
void displayTemp() {
  float tempC = getTemperature();

  //  temperature
  clearAll(RgbColor(0));
  drawBmp(0, 0, "/common/fond_temp.bmp");
  drawLargeInt(6, 1, (int8_t)round(tempC), 2, RgbColor(0));
  displayMatrix();

  // no refresh update
  _next_display_function = NOTHING;
}

/*
   Page cmd value
*/
void displayCmdValue() {
  float tempC = getTemperature();

  //  temperature
  clearAll(RgbColor(0));
  drawBmp(0, 0, "/common/fond_temp.bmp");
  drawLargeInt(6, 1, (int8_t)round(tempC), 2, RgbColor(0));
  displayMatrix();

  // no refresh update
  _next_display_function = NOTHING;
}

/*
   Page art
*/
void displayArt() {
  // select file
  File fsRoot = SD.open("/pixels");
  uint8_t count = 1;
  String path;
  while (true) {
    File fsPixel = fsRoot.openNextFile();
    if (fsPixel) {
      char filename[32];
      sprintf(filename, "/pixels/%s", fsPixel.name());
      path = String(filename);
      fsPixel.close();
    } else {
      // no more files
      break;
    }

    // is selected pixel
    if (_currentPixelId == count) {
      // prepare next
      _currentPixelId++;
      // back to begin if no next file
      File fsNextPixel = fsRoot.openNextFile();
      if (!fsNextPixel) {
        _currentPixelId = 1;
      } else {
        fsNextPixel.close();
      }
      // display
      break;
    }
    count++;
  }
  fsRoot.close();

  if (path) {
    // display
    clearAll(RgbColor(0));
    drawBmp(0, 0, path.c_str());
    TRACE("Display image : %s", path.c_str());
    displayMatrix();
  }

  // no refresh update - new each loop
  _next_display_function = NOTHING;
}

/*
   Display meteo
*/
void displayMeteo() {
  clearAll(RgbColor(0));

  float temp = _currentTemp;
  char file[80];
  sprintf(file, "/meteo/%s.bmp", _currentWeatherIcon);
  /*DateTime now = _rtc.now(); //Récupère l'heure et le date courante
    float temp = _currentFeelsLike;
    char file[80];
    // if before 6pm
    if (now.hour() <= 18) {
    sprintf(file, "/meteo/%s.bmp", _todayWeatherIcon);
    } else  {
    sprintf(file, "/meteo/%s.bmp", _tomorrowWeatherIcon);
    temp = _tomorrowFeelsLike;
    colorPixel(0, 0, RgbColor(50));
    }*/
  drawBmp(0, 0, file);
  TRACE("Meteo icon : %s", file);

  // display temperature
  RgbColor color = RgbColor(48, 255, 103);
  int8_t tempInt = round(temp);
  uint8_t pos = 0;
  if (tempInt < 0) {
    colorPixel(pos, 13, color);
    pos++;
    colorPixel(pos, 13, color);
    pos++;
    tempInt = -tempInt;
  }
  //pos++;
  if (tempInt < 10) {
    drawSmallInt(pos, 11, tempInt, 1, color);
    pos += 4;
  } else {
    drawSmallInt(pos, 11, tempInt, 2, color);
    pos += 8;
  }
  colorPixel(pos, 11, color);
  pos++;
  colorPixel(pos, 13, color);
  colorPixel(pos, 14, color);
  colorPixel(pos, 15, color);
  pos++;
  colorPixel(pos, 13, color);
  colorPixel(pos, 15, color);

  displayMatrix();

  // no refresh update
  _next_display_function = NOTHING;
}

/*
   Page IP
*/
void displayIp() {
  IPAddress ip = WiFi.localIP();  //Récupère l'IP

  // time
  clearAll(RgbColor(0));
  RgbColor colorText = RgbColor(255, 255, 255);
  drawBmp(0, 0, "/common/fond_ip.bmp");
  drawSmallInt(0, 0, ip[0], 3, colorText);
  Serial.println(ip[0]);
  Serial.println(ip[1]);
  Serial.println(ip[2]);
  Serial.println(ip[3]);
  drawSmallInt(0, 5, ip[1], 3, colorText);
  //drawSmallInt(10, 0, ip[2], 3, colorText);
  drawSmallInt(0, 10, ip[3], 3, colorText);
  //colorPixel(3, 10, colorText);
  displayMatrix();
}


/*
   -------------- TECHNICAL ------------------------
*/

/*
  Init debug
*/
void initDebug() {
  char name[15];
  sprintf(name, "pixel_%s", _eeprom.name);
  Debug.begin(name);
  Debug.setResetCmdEnabled(true);
  Debug.showProfiler(false);
  Debug.showColors(true);
  Debug.setPassword("password");
}
/*
  Remote Debug functions
*/
void remoteDebugTrace(const char* format, ...) {
  if (Debug.isActive(Debug.INFO)) {
    char buffer[LOGGER_MAX_LEN];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, LOGGER_MAX_LEN, format, args);
    va_end(args);
    Debug.println(buffer);
  }
}

/*
  Load EEPORM
*/
void loadEEPROM() {
  // Init EEPROM
  size_t eeprom_size = sizeof(_eeprom);
  EEPROM.begin(eeprom_size);
  TRACE("Read EEPROM data - size=%d", eeprom_size);
  // Lit la mémoire EEPROM
  EEPROM.get(0, _eeprom);

  // Détection d'une mémoire non initialisée
  byte erreur = _eeprom.magic != STRUCT_MAGIC;

  // Valeurs par défaut struct_version == 1
  if (erreur) {
    TRACE("Reinit EEPROM data...");
    // name
    strcpy(_eeprom.name, "demo");
    // night mode
    _eeprom.night_threshold = 1800;
    _eeprom.night_hour_r = 48;
    _eeprom.night_hour_g = 255;
    _eeprom.night_hour_b = 103;
    _eeprom.night_min_r = 90;
    _eeprom.night_min_g = 90;
    _eeprom.night_min_b = 90;
    _eeprom.night_tempi_r = 205;
    _eeprom.night_tempi_g = 107;
    _eeprom.night_tempi_b = 73;
    _eeprom.night_tempe_r = 73;
    _eeprom.night_tempe_g = 107;
    _eeprom.night_tempe_b = 205;
    // IR commands
    _eeprom.ir_cmd_temp_sum = 0xD;
    _eeprom.ir_cmd_ip = 0x16;
    // brightness parameters
    _eeprom.bright_max = 11;
    _eeprom.bright_threshold = 10;
    _eeprom.bright_map[0] = 0;
    _eeprom.bright_map[1] = 0;
    _eeprom.bright_map[2] = 0;
    _eeprom.bright_map[3] = 0;
    _eeprom.bright_map[4] = 1;
    _eeprom.bright_map[5] = 1;
    _eeprom.bright_map[6] = 1;
    _eeprom.bright_map[7] = 2;
    _eeprom.bright_map[8] = 2;
    _eeprom.bright_map[9] = 3;
    _eeprom.bright_map[10] = 3;
    _eeprom.bright_map[11] = 4;
    _eeprom.bright_map[12] = 4;
    _eeprom.bright_map[13] = 5;
    _eeprom.bright_map[14] = 5;
    _eeprom.bright_map[15] = 6;
    _eeprom.bright_map[16] = 6;
    _eeprom.bright_map[17] = 7;
    _eeprom.bright_map[18] = 7;
    _eeprom.bright_map[19] = 8;
    _eeprom.bright_map[20] = 8;
    _eeprom.bright_map[21] = 9;
    _eeprom.bright_map[22] = 9;
    _eeprom.bright_map[23] = 10;
    _eeprom.bright_map[24] = 10;
    _eeprom.bright_map[25] = 11;
    _eeprom.bright_map[26] = 11;
    _eeprom.bright_map[27] = 11;
    _eeprom.bright_map[28] = 11;
    _eeprom.bright_map[29] = 11;
    // WIFI
    strcpy(_eeprom.wifi_ssid, WIFI_SSID);
    strcpy(_eeprom.wifi_key, WIFI_KEY);
    // MQTT
    strcpy(_eeprom.mqtt_server, MQTT_SERVER);
    _eeprom.mqtt_port = MQTT_PORT;
    strcpy(_eeprom.mqtt_channel, MQTT_CHANNEL);
    strcpy(_eeprom.mqtt_user, MQTT_TOKEN);
    strcpy(_eeprom.mqtt_pwd, MQTT_PWD);
    strcpy(_eeprom.mqtt_res_temp, "temp");
    // OPENWEATHER
    strcpy(_eeprom.weather_appid, "87b2d45da5888635d6f65fb749a41b5b");
    strcpy(_eeprom.weather_location, "lat=43.439&lon=1.6");

    // Sauvegarde les nouvelles données
    saveEEPROM();
  }

  //strcpy(_eeprom.wifi_ssid, WIFI_SSID);
  // strcpy(_eeprom.wifi_key, WIFI_KEY);
}

/*
   Save EEPROM
*/
void saveEEPROM() {
  // Met à jour le nombre magic et le numéro de version avant l'écriture
  _eeprom.magic = STRUCT_MAGIC;
  _eeprom.struct_version = STRUCT_VERSION;
  EEPROM.put(0, _eeprom);
  EEPROM.commit();
  TRACE("EEPROM data saved");
}


/*
   Connect WIFI
*/
void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(_eeprom.wifi_ssid, _eeprom.wifi_key);
  TRACE("Connecting wifi : %s", _eeprom.wifi_ssid);
  int timeout_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    timeout_counter++;
    if (timeout_counter >= CONNECTION_TIMEOUT * 5) {
      ESP.restart();
    }
  }
  Serial.println("");
  IPAddress ip = WiFi.localIP();
  TRACE("Connected to WiFi network with IP Address: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

  // declare mDNS
  char name[15];
  sprintf(name, "pixel_%s", _eeprom.name);
  if (!MDNS.begin(name)) {
    TRACE("Error setting up mDNS responder!");
  } else {
    MDNS.addService("http", "tcp", 80);
    TRACE("mDNS responder started with %s", name);
  }
}
/*
   Update wifi connection
*/
void updateWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    TRACE("Error Wifi not connected!");
    // Visual indicator
    colorPixel(0, 0, RgbColor(255, 0, 0));
    displayMatrix();
    // try to reconnect
    WiFi.disconnect();
    WiFi.begin(_eeprom.wifi_ssid, _eeprom.wifi_key);
  }
  // update mDNS
  MDNS.queryService("http", "tcp");
}

/*
   Connect mqtt
*/
void connectMqtt() {
  TRACE("MQTT server: %s", _eeprom.mqtt_server);
  // start mqtt
  _mqttClient.setServer(_eeprom.mqtt_server, _eeprom.mqtt_port);
  _mqttClient.setCallback(onMqttMessage);
  _mqttClient.setKeepAlive(MQTT_KEEP_ALIVE);
  _mqttClient.setSocketTimeout(MQTT_TIMEOUT);

  // call reconnect to start
  reconnectMqtt();
}

bool reconnectMqtt() {
  // Check before
  if (!_mqttClient.connected()) {
    TRACE("MQTT : %s", convertMqttState(_mqttClient.state()));
    // Visual indicator
    colorPixel(1, 0, RgbColor(255, 0, 0));
    displayMatrix();
    // Attempt to connect, just a name to identify the client
    if (_mqttClient.connect(generateID(), _eeprom.mqtt_user, _eeprom.mqtt_pwd)) {
      // reconnected
      TRACE(" - Connected");
      TRACE(" - Suscribe all topics....");
      // suscribe to all requested topics
      subscribeMqtt(MQTT_RES_CMD);
      //subscribeMqtt(MQTT_RES_ACT_CMD);
      TRACE(" - Subscribed");
      return true;
    }
    TRACE(" - failed, rc=%d", _mqttClient.state());
    // fail to reconnect
    return false;
  }
  // already connected
  return true;
}
const char* convertMqttState(int state) {
  const char* desc;
  switch (state) {
    case MQTT_CONNECTION_TIMEOUT:
      desc = "connection timeout";
      break;
    case MQTT_CONNECTION_LOST:
      desc = "connection lost";
      break;
    case MQTT_CONNECT_FAILED:
      desc = "connection failed";
      break;
    case MQTT_DISCONNECTED:
      desc = "disconnected";
      break;
    case MQTT_CONNECTED:
      desc = "connected";
      break;
    case MQTT_CONNECT_BAD_PROTOCOL:
      desc = "bad protocol";
      break;
    case MQTT_CONNECT_BAD_CLIENT_ID:
      desc = "bad client ID";
      break;
    case MQTT_CONNECT_UNAVAILABLE:
      desc = "bad unvailable";
      break;
    case MQTT_CONNECT_BAD_CREDENTIALS:
      desc = "bad credentials";
      break;
    case MQTT_CONNECT_UNAUTHORIZED:
      desc = "unauthorized";
      break;
    default:
      desc = "unknown state";
      break;
  }
  return desc;
}
const char* generateID() {
  /*  randomSeed(analogRead(0));
  int i = 0;
  for (i = 0; i < sizeof(_id) - 1; i++) {
    _id[i] = CHARS[random(sizeof(CHARS))];
  }
  _id[sizeof(_id) - 1] = '\0';

  return _id;*/
  return "pixel";
}
void checkMqtt() {
  // get messages
  _mqttClient.loop();
}
void checkMqttConnection() {
  // MQTT loop
  if (!_mqttClient.connected()) {
    TRACE("MQTT server not connected !");
    reconnectMqtt();
  }
  // publish si alive message
  publishMqtt("alive", "device ping", false);
}
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  TRACE("MQTT MSG < [%s] %.*s", topic, length, (char*)payload);

  // decode the JSON payload
  StaticJsonDocument<128> root;
  DeserializationError status = deserializeJson(root, payload, length);
  // Test if parsing succeeds.
  if (status) {
    TRACE("deserializeJson() failed: %s", status.c_str());
    return;
  }

  String procTopic = String(topic);
  // Process CMD
  // channel=pixel resource=cmd data={'image':2','value':12.3}
  if (procTopic.endsWith(String("/") + MQTT_RES_CMD)) {
    // led resource is a boolean read it accordingly
    //JsonObject data = root["data"];
    TRACE("TEST A");
    if (root.containsKey("image") && root.containsKey("value")) {
      TRACE("TEST OK");
      // get arguments
      //strcpy(_cmdDataImage, data["image"]);
      //_cmdDataValue = data["value"];
      //fsm.trigger(EVENT_DISP_CMD_VALUE);
      //const uint8_t folderNumber = data["folder"];
      //const uint8_t fileNumber = data["file"];
      //playMp3(folderNumber, fileNumber);
    }
  }
}

/*
   Subscribe Mqtt
*/
void subscribeMqtt(const char* resource) {
  char topic[64];
  sprintf(topic, "%s/%s", _eeprom.mqtt_channel, resource);
  TRACE("MQTT SUB > [%s]", topic);
  _mqttClient.subscribe(topic);
}

/*
   Publish Mqtt
*/
void publishMqtt(const char* resource, const StaticJsonDocument<64>& data, bool persist) {
  StaticJsonDocument<128> root;
  root["data"] = data;
  internalPubMqtt(root, resource, persist);
}
void publishMqtt(const char* resource, const char* value, bool persist) {
  StaticJsonDocument<128> root;
  root["data"] = value;
  internalPubMqtt(root, resource, persist);
}
void publishMqtt(const char* resource, const float value, bool persist) {
  StaticJsonDocument<128> root;
  root["data"] = value;
  internalPubMqtt(root, resource, persist);
}
void publishMqttDate() {
  DateTime now = _rtc.now();
  char buffer[25];
  sprintf(buffer, "%04u-%02u-%02uT%02u:%02u:%02uZ", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  publishMqtt("date", buffer, true);
}
void internalPubMqtt(StaticJsonDocument<128>& root, const char* resource, bool persist) {
  //root["channel"] = _eeprom.mqtt_channel;
  //root["resource"] = resource;
  if (persist) {
    root["write"] = true;
  }

  // Now print the JSON into a char buffer
  String buffer;
  serializeJson(root, buffer);

  // Create the topic to publish to
  char topic[64];
  sprintf(topic, "%s/%s", _eeprom.mqtt_channel, resource);

  TRACE("MQTT PUB > [%s] %s", topic, buffer.c_str());
  // Now publish the char buffer to Beebotte
  _mqttClient.publish(topic, buffer.c_str(), persist);
}

/*
   Connect SD CARD
*/
void connectSD() {
  if (!SD.begin(PORT_SD_CS)) {
    TRACE("Card Mount Failed");
    return;
  } else {
    TRACE("SD Card mounted with success");
  }
}

/*
   Init IR
*/
void initIrRemote() {
  IrReceiver.begin(PORT_IR, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
}

/*
   Check if remote command received
*/
void checkRemoteCmd() {
  if (IrReceiver.decode()) {
    TRACE("Remote IR command received");
    // Print a short summary of received data
    IrReceiver.printIRResultShort(&Serial);
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      // We have an unknown protocol here, print more info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
    }
    IrReceiver.resume();  // Enable receiving of the next value

    // ignore repeat
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      // copy the command
      _lastIrCommand = IrReceiver.decodedIRData.command;
      // detect command TEMP. SUMMARY
      if (_lastIrCommand == _eeprom.ir_cmd_temp_sum) {
        fsm.trigger(EVENT_DISP_TEMP_SUM);
        // detect command TEMP. SUMMARY
      } else if (_lastIrCommand == _eeprom.ir_cmd_ip) {
        fsm.trigger(EVENT_DISP_CONF_IP);
        // detect command UP
      } else if (_lastIrCommand == 0x18) {  // TODO
        _brightness += 1;
        displayMatrix();
        // detect command DOWN
      } else if (_lastIrCommand == 0x52) {  // TODO
        _brightness -= 1;
        displayMatrix();
      }
    }
  }
}

/*
   Init MP3
*/
void initMp3() {
  _mp3Serial.begin(9600, SERIAL_8N1, PORT_RXD2, PORT_TXD2);  // speed, type, RX, TX
  delay(1000);
  if (!_mp3Player.begin(_mp3Serial)) {  //Use softwareSerial to communicate with mp3.
    TRACE("Unable to init MP3 player. Please insert the SD card!");
  } else {
    TRACE("MP3 player online.");
    _mp3Player.setTimeOut(500);
    _mp3Player.volume(10);
    _mp3Player.EQ(DFPLAYER_EQ_NORMAL);
    _mp3Player.outputDevice(DFPLAYER_DEVICE_SD);
    _mp3Player.disableLoop();
  }
}
/*
   Play MP3
*/
void playMp3(uint8_t folderNumber, uint8_t fileNumber) {
  _mp3Player.playFolder(folderNumber, fileNumber);
}
/*
   Set volume UP
*/
void setVolumeUp() {
  _mp3Player.volumeUp();
}
/*
   Set volume DOWN
*/
void setVolumeDown() {
  _mp3Player.volumeDown();
}
/*
   Set volume
*/
void setVolume(uint8_t volume) {
  _mp3Player.volume(volume);
}
/*
   Get volume
*/
int getVolume() {
  return _mp3Player.readVolume();
}

/*
   Update NTP
*/
void updateNtp() {
  _timeClient.update();

  DateTime actualTime = DateTime(_timeClient.getEpochTime());

  // calculate winter hour
  // last sunday of march
  int beginDSTDate = (31 - (5 * actualTime.year() / 4 + 4) % 7);
  int beginDSTMonth = 3;
  // last sunday of october
  int endDSTDate = (31 - (5 * actualTime.year() / 4 + 1) % 7);
  int endDSTMonth = 10;
  // DST is valid as:
  if (((actualTime.month() > beginDSTMonth) && (actualTime.month() < endDSTMonth))
      || ((actualTime.month() == beginDSTMonth) && (actualTime.day() >= beginDSTDate))
      || ((actualTime.month() == endDSTMonth) && (actualTime.day() <= endDSTDate))) {
    TRACE("Detect summer hour");
    actualTime = DateTime(actualTime.year(), actualTime.month(), actualTime.day(), actualTime.hour() + 1, actualTime.minute(), actualTime.second());
  }

  TRACE("NTP - Internet Epoch Time: %02u/%02u/%04u %02u:%02u", actualTime.day(), actualTime.month(), actualTime.year(), actualTime.hour(), actualTime.minute());

  // update RTC
  _rtc.adjust(actualTime);
}

/*
   Init temperature sensor
*/
void initTempSensor() {
  // locate devices on the bus
  TRACE("Locating devices...");
  _tempSensor.begin();
  TRACE("Found %d devices.", _tempSensor.getDeviceCount());

  // report parasite power requirements
  TRACE("Parasite power is: %d", _tempSensor.isParasitePowerMode());

  // get adress
  if (!_tempSensor.getAddress(_insideThermometer, 0)) Serial.println(F("Unable to find address for Device 0"));

  // resolution
  TRACE("Device 0 Resolution: %d", _tempSensor.getResolution(_insideThermometer));
}

/*
   Get temperatuire
*/
float getTemperature() {
  // Read temperature
  _tempSensor.requestTemperatures();
  float tempC = _tempSensor.getTempC(_insideThermometer);
  // Check if any reads failed and exit early (to try again).
  if (isnan(tempC)) {
    TRACE("Failed to read from sensor!");
    return 0;
  }
  TRACE("Temperature: %f *C", tempC);

  // publish on MQTT
  publishMqtt(_eeprom.mqtt_res_temp, tempC, true);
  publishMqttDate();


  return tempC;
}

/*
   Init matrix
*/
void initMatrix() {
  _pixels.Begin();
  adjustBrightness();
  clearAll(RgbColor(0));
  displayMatrix();
}

/*
   Check night mode
*/
void checkNightMode() {
  // read light value
  int lightVal = getLightVal();
  // check night
  if (lightVal < _eeprom.night_threshold) {
    // activate night
    fsm.trigger(EVENT_DISP_NIGHT_MODE);
  }
  // check day
  if (lightVal > _eeprom.night_threshold + 50) {
    // activate day
    fsm.trigger(EVENT_DISP_DAY_MODE);
  }
}

/*
   Display matrix
*/
void displayMatrix() {
  // protect overload of power
  if (_brightness > _eeprom.bright_max) {
    _brightness = _eeprom.bright_max;
  }
  uint16_t i = 0;
  // row
  for (uint8_t y = 0; y < 16; y++) {
    // column
    for (uint8_t x = 0; x < 16; x++) {
      // copy color and apply brightness
      RgbColor color = RgbColor(_matrix[x][y].R * _brightness / 255, _matrix[x][y].G * _brightness / 255, _matrix[x][y].B * _brightness / 255);
      // ligne paire
      i = y * 16 + (15 - x);
      // ligne impaire
      if (y % 2 == 1) {
        i = y * 16 + x;
      }
      _pixels.SetPixelColor(i, color);
    }
  }
  _pixels.Show();
}


/*
   Update display brightness
*/
void updateDisplayBrightness() {
  // fade management
  switch (_displayFadeStatus) {
    case DISPLAY_FADE_IN:
      // increase brightness
      _brightness += 1;
      // check target reached
      if (_brightness >= _brightnessTarget) {
        _displayFadeStatus = DISPLAY_FADE_WAIT;
      }
      break;
    case DISPLAY_FADE_OUT:
      // decrease brightness
      _brightness -= 1;
      // check target reached
      if (_brightness == 0) {
        if (_next_display_function != NOTHING) {
          // draw new page
          _next_display_function();
          // fade in
          _brightnessTarget = evalBrightness();
          _displayFadeStatus = DISPLAY_FADE_IN;
        } else {
          _displayFadeStatus = DISPLAY_FADE_WAIT;
        }
      }
      break;
    default:
      break;
  }
  // apply
  displayMatrix();
}


/*
   switch to new display
*/
void switchTo(void (*new_display_func)()) {
  _next_display_function = new_display_func;
  _brightnessTarget = 0;
  _displayFadeStatus = DISPLAY_FADE_OUT;
}


/*
   Adjust brightness
*/
void adjustBrightness() {
  uint8_t b = evalBrightness();
  if ((b > (100.0 + _eeprom.bright_threshold) / 100 * _brightness) || (b < (100.0 - _eeprom.bright_threshold) / 100 * _brightness)) {
    _brightness = b;
    TRACE("Brightness: %d", _brightness);
    displayMatrix();
  }
}

/*
   Get light val
*/
int getLightVal() {
  // calculate new index
  _lightMeasuresIndex++;
  if (_lightMeasuresIndex >= AVERAGE_LIGHT_MEASURE) {
    _lightMeasuresIndex = 0;
  }

  // remove old measure
  _lightMeasuresSum -= _lightMeasures[_lightMeasuresIndex];
  // read the current light levels
  _lightMeasures[_lightMeasuresIndex] = analogRead(PORT_LUX);
  _lightMeasuresSum += _lightMeasures[_lightMeasuresIndex];

  // calculate average
  int lightVal = _lightMeasuresSum / AVERAGE_LIGHT_MEASURE;
  //TRACE("Light measure: %d - Light average: %d", _lightMeasures[_lightMeasuresIndex], lightVal);
  return lightVal;
}
/*
   Eval brightness
*/
uint8_t evalBrightness() {
  // read the current light levels
  int lightVal = getLightVal();

  // norm light value
  if (lightVal < 1000) {
    lightVal = 1000;
  } else if (lightVal > 3900) {
    lightVal = 3900;
  }
  uint8_t index = round((lightVal - 1000) / 100);
  uint8_t b = _eeprom.bright_map[index];

  return b;
}

/*
   Clear all
*/
void clearAll(const RgbColor color) {
  // row
  for (uint8_t y = 0; y < 16; y++) {
    // column
    for (uint8_t x = 0; x < 16; x++) {
      _matrix[x][y] = color;
    }
  }
}

/*
   Color pixel on the matrix
*/
void colorPixel(const uint8_t x, const uint8_t y, const RgbColor color) {
  _matrix[x][y] = color;
}

/*
   Draw BMP on the matrix
*/
void drawBmp(const uint8_t x0, const uint8_t y0, const uint8_t w, const uint8_t h, const unsigned long bmp[]) {
  // row
  for (uint8_t y = 0; y < h; y++) {
    // column
    for (uint8_t x = 0; x < w; x++) {
      uint16_t i = y * w + x;
      uint8_t red = (bmp[i] & 0x00ff0000) >> 16;
      uint8_t green = (bmp[i] & 0x0000ff00) >> 8;
      uint8_t blue = (bmp[i] & 0x000000ff);
      colorPixel(x0 + x, y0 + y, RgbColor(red, green, blue));
    }
  }
}

/*
   Draw BMP from SD
*/
void drawBmp(const uint8_t x0, const uint8_t y0, const char filename[]) {
  File bmpImage = SD.open(filename, FILE_READ);
  bmpImage.seek(0x12);  // width
  int32_t width = bmpImage.read();
  bmpImage.seek(0x16);  // height
  int32_t height = bmpImage.read();
  bmpImage.seek(0x1C);  // pixel size
  int16_t pixelsize = bmpImage.read();
  //Serial.println(width);
  //Serial.println(height);
  //Serial.println(pixelsize); //24

  if (pixelsize == 24) {
    clearAll(RgbColor(0));
    int imageSize = height * width;
    bmpImage.seek(0x36);  //skip bitmap header
    for (uint8_t i = 0; i < height; i++) {
      for (uint8_t j = 0; j < width; j++) {
        uint8_t blue = bmpImage.read();
        uint8_t green = bmpImage.read();
        uint8_t red = bmpImage.read();
        colorPixel(x0 + j, y0 + height - i - 1, RgbColor(red, green, blue));
      }
    }
    bmpImage.close();
  }
}

/*
   Draw number on the matrix
*/
void drawLargeInt(const uint8_t x0, const uint8_t y0, const int number, const uint8_t digit, const RgbColor color) {
  //
  uint8_t multiple = pow(10, digit - 1);
  int8_t tmp_number = number;
  uint8_t tmp_x0 = x0;
  while (multiple >= 1) {
    uint8_t chiff = tmp_number / multiple;
    switch (chiff) {
      case 0:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l0, color);
        break;
      case 1:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l1, color);
        break;
      case 2:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l2, color);
        break;
      case 3:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l3, color);
        break;
      case 4:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l4, color);
        break;
      case 5:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l5, color);
        break;
      case 6:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l6, color);
        break;
      case 7:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l7, color);
        break;
      case 8:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l8, color);
        break;
      case 9:
        drawCharacter(tmp_x0, y0, 4, 7, NUMBER_l9, color);
        break;
      default:
        break;
    }
    tmp_number = tmp_number - chiff * multiple;
    multiple = multiple / 10;
    tmp_x0 = tmp_x0 + 5;
  }
}

/*
   Draw number on the matrix
*/
void drawSmallInt(const uint8_t x0, const uint8_t y0, const int number, const uint8_t digit, const RgbColor color) {
  //
  uint8_t multiple = pow(10, digit - 1);
  int8_t tmp_number = number;
  uint8_t tmp_x0 = x0;
  while (multiple >= 1) {
    uint8_t chiff = tmp_number / multiple;
    switch (chiff) {
      case 0:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s0, color);
        break;
      case 1:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s1, color);
        break;
      case 2:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s2, color);
        break;
      case 3:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s3, color);
        break;
      case 4:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s4, color);
        break;
      case 5:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s5, color);
        break;
      case 6:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s6, color);
        break;
      case 7:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s7, color);
        break;
      case 8:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s8, color);
        break;
      case 9:
        drawCharacter(tmp_x0, y0, 3, 5, NUMBER_s9, color);
        break;
      default:
        break;
    }
    tmp_number = tmp_number - chiff * multiple;
    multiple = multiple / 10;
    tmp_x0 = tmp_x0 + 4;
  }
}

/*
   Draw character
*/
void drawCharacter(const uint8_t x0, const uint8_t y0, const uint8_t w, const uint8_t h, const bool character[], const RgbColor color) {
  for (uint8_t y = 0; y < h; y++) {
    // column
    for (uint8_t x = 0; x < w; x++) {
      uint16_t i = y * w + x;
      if (character[i]) {
        colorPixel(x0 + x, y0 + y, color);
      }
    }
  }
}


/*
   Get meteo
*/
void updateMeteo() {
  // call current meteo API
  {
    TRACE("Load current meteo");
    char serviceUrl[180];
    sprintf(serviceUrl, METEO_CURRENT_API_URL, _eeprom.weather_location, _eeprom.weather_appid);
    Serial.println(serviceUrl);
    String jsonBuffer = callApiGet(serviceUrl);
    //Serial.println(jsonBuffer);
    StaticJsonDocument<5000> json;
    //DynamicJsonDocument json(5000);
    DeserializationError error = deserializeJson(json, jsonBuffer);
    if (error) {
      TRACE("Parsing meteo input failed! %s", error.c_str());
      return;
    }

    // current weather
    _currentTemp = json["current"]["temp"];
    _currentFeelsLike = json["current"]["feels_like"];
    _currentHumidity = json["current"]["humidity"];
    _currentWindSpeed = ((float)json["current"]["wind_speed"]) * 3.6;
    _currentWeatherId = json["current"]["weather"][0]["id"];
    _currentWeather = json["current"]["weather"][0]["description"].as<String>();
    _currentWeather.replace("é", "e");
    _currentWeather.replace("è", "e");
    _currentWeatherIcon = json["current"]["weather"][0]["icon"].as<String>();
    _currentRain = json["current"]["rain"]["1h"];
  }

  // call forecast API
  {
    TRACE("Load forecast meteo");
    char serviceUrl[180];
    sprintf(serviceUrl, METEO_FORECAST_API_URL, _eeprom.weather_location, _eeprom.weather_appid);
    Serial.println(serviceUrl);
    String jsonBuffer = callApiGet(serviceUrl);
    //Serial.println(jsonBuffer);
    StaticJsonDocument<5000> json;
    //DynamicJsonDocument json(5000);
    DeserializationError error = deserializeJson(json, jsonBuffer);
    if (error) {
      TRACE("Parsing meteo input failed! %s", error.c_str());
      return;
    }
    // today weather
    _todayTemp = json["daily"][0]["temp"]["day"];
    _todayFeelsLike = json["daily"][0]["feels_like"]["day"];
    _todayHumidity = json["daily"][0]["humidity"];
    _todayWindSpeed = ((float)json["daily"][0]["wind_speed"]) * 3.6;
    _todayWeatherId = json["daily"][0]["weather"][0]["id"];
    _todayWeather = json["daily"][0]["weather"][0]["description"].as<String>();
    _todayWeather.replace("é", "e");
    _todayWeather.replace("è", "e");
    _todayWeatherIcon = json["daily"][0]["weather"][0]["icon"].as<String>();
    _todayRain = json["daily"][0]["rain"];

    // tomorrow weather
    _tomorrowTemp = json["daily"][1]["temp"]["day"];
    _tomorrowFeelsLike = json["daily"][1]["feels_like"]["day"];
    _tomorrowHumidity = json["daily"][1]["humidity"];
    _tomorrowWindSpeed = ((float)json["daily"][1]["wind_speed"]) * 3.6;
    _tomorrowWeatherId = json["daily"][1]["weather"][0]["id"];
    _tomorrowWeather = json["daily"][1]["weather"][0]["description"].as<String>();
    _tomorrowWeather.replace("é", "e");
    _tomorrowWeather.replace("è", "e");
    _tomorrowWeatherIcon = json["daily"][1]["weather"][0]["icon"].as<String>();
    _tomorrowRain = json["daily"][1]["rain"];
  }
}

/*
   Call API
*/
String callApiGet(const char* serverName) {
  HTTPClient http;
  // Your IP address with path or Domain name with URL path
  http.begin(serverName);
  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";
  if (httpResponseCode > 0) {
    payload = http.getString();
  } else {
    TRACE("HTTP Response code: %d", httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

/*
   Init webserver
*/
void initWebServer() {
  _server.on("/", HTTP_GET, webOnRoot);
  _server.on("/mp3", HTTP_GET, webOnMp3);
  _server.on("/art", HTTP_GET, webOnArt);
  _server.on(
    "/pixel_upload", HTTP_POST, []() {
      // args
      _currentDir = "/";
      if (_server.hasArg("dir")) {
        _currentDir = _server.arg("dir");
      }
      _server.send(200, "text/plain", "");
    },
    webOnPixelUpload);
  _server.on("/pixel_download", HTTP_GET, webOnPixelDownload);
  _server.on("/pixel_display", HTTP_GET, webOnPixelDisplay);
  _server.on("/pixel_delete", HTTP_GET, webOnPixelDelete);
  _server.on("/properties", HTTP_GET, webOnProperties);
  _server.on("/configure", HTTP_POST, webOnConfigure);
  _server.on("/firmware", HTTP_GET, webOnFirmware);
  _server.on(
    "/firmware_upload", HTTP_POST, []() {
      _server.sendHeader("Connection", "close");
      _server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    },
    webOnFirmwareUpload);
  _server.onNotFound(webNotFound);
  _server.begin();
  TRACE("HTTP server started");
}
String createHeadHtml(String title) {
  String ptr = "<head>\n";
  ptr += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<title>" + title + " [" + String(_eeprom.name) + "]</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr += ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr += ".button-on {background-color: #3498db;}\n";
  ptr += ".button-on:active {background-color: #2980b9;}\n";
  ptr += ".button-off {background-color: #34495e;}\n";
  ptr += ".button-off:active {background-color: #2c3e50;}\n";
  ptr += "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr += ".meteo {display: inline-block; margin: 10px;}\n";
  ptr += ".form_area {text-align: left;max-width: 600px;margin-left: auto;margin-right: auto;}\n";
  ptr += ".form_line {margin: 5px;}\n";
  ptr += ".area_bt {display: inline-block; margin: 10px;}\n";
  ptr += "fieldset  {margin-top: 25px;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  return ptr;
}
void webOnRoot() {
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += createHeadHtml("Pixel");
  ptr += "<body>\n";
  ptr += "<h1>Pixel Web Server [" + String(_eeprom.name) + "]</h1>\n";
  // Date
  DateTime now = _rtc.now();  //Récupère l'heure et le date courante
  ptr += "<p>" + String(DAYS[now.dayOfTheWeek()]) + " " + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()) + " " + String(now.hour()) + ":" + String(now.minute()) + "</p>\n";

  // METEO
  ptr += "<div>\n";
  ptr += "<div class=\"meteo\">\n";
  ptr += "<p>Aujourd'hui</p>\n";
  ptr += "<img src=\"http://openweathermap.org/img/wn/" + _todayWeatherIcon + "@2x.png\" /><br/>\n";
  ptr += "" + String(_todayTemp) + " &deg;C | ressenti : " + String(_todayFeelsLike) + " &deg;C<br/>\n";
  ptr += "humidit&eacute; : " + String(_todayHumidity) + " % | vent : " + String(_todayWindSpeed) + " km/h | pluie : " + String(_todayRain) + " mm\n";
  ptr += "</div>\n";
  ptr += "<div class=\"meteo\">\n";
  ptr += "<p style=\"font-size: 148px;\">&gt;</p>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"meteo\">\n";
  ptr += "<p>Demain</p>\n";
  ptr += "<img src=\"http://openweathermap.org/img/wn/" + _tomorrowWeatherIcon + "@2x.png\" /><br/>\n";
  ptr += "" + String(_tomorrowTemp) + " &deg;C | ressenti : " + String(_tomorrowFeelsLike) + " &deg;C<br/>\n";
  ptr += "humidit&eacute; : " + String(_tomorrowHumidity) + " % | vent : " + String(_tomorrowWindSpeed) + " km/h | pluie : " + String(_tomorrowRain) + " mm\n";
  ptr += "</div>\n";
  ptr += "</div>\n";

  // LIGHT
  int lightVal = getLightVal();
  uint8_t evalB = evalBrightness();
  ptr += "<p>Light measure = " + String(lightVal) + " - Evaluated brightness = " + String(evalB) + "</p>\n";
  // TEMPERATURE
  float tempC = getTemperature();
  ptr += "<p>Temperature = " + String(tempC) + " &deg;C</p>\n";
  // IR REMOTE
  ptr += "<p>Last IR command = " + String(_lastIrCommand) + "</p>\n";
  // FREE MEMORY
  ptr += "<p>Free heap size = " + String(ESP.getFreeHeap() / 1024) + " kB</p>\n";

  // To MP3
  ptr += "<a class=\"\" href=\"/mp3\">MP3 player</a> | \n";
  // To ART
  ptr += "<a class=\"\" href=\"/art\">Arts folder</a> | \n";
  // PROPERTIES
  ptr += "<a class=\"\" href=\"/properties\">Properties</a> | \n";
  // FIRMWARE
  ptr += "<a class=\"\" href=\"/firmware\">Firmware</a>\n";

  ptr += "</body>\n";
  ptr += "</html>\n";

  _server.send(200, "text/html", ptr);
}
void webOnMp3() {
  // arags
  if (_server.arg("cmd") == "play") {
    uint8_t folderNumber = _server.arg("folder").toInt();
    uint8_t fileNumber = _server.arg("file").toInt();
    playMp3(folderNumber, fileNumber);
  } else if (_server.arg("cmd") == "vol_up") {
    setVolumeUp();
  } else if (_server.arg("cmd") == "vol_down") {
    setVolumeDown();
  } else if (_server.arg("cmd") == "vol") {
    uint8_t volume = _server.arg("value").toInt();
    setVolume(volume);
  }

  // html
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += createHeadHtml("Pixel MP3");
  ptr += "<body>\n";
  ptr += "<h1>Pixel MP3 player</h1>\n";
  ptr += "<div class=\"form_area\">\n";

  /*ptr += "<fieldset><legend>Play</legend>\n";
    for (uint8_t folder = 1; folder <= 6; ++folder) {
    int nb = _mp3Player.readFileCountsInFolder(folder);
    String nb_str = String(nb) + " files";
    if (nb < 0 ) {
      nb_str = "empty";
    }
    ptr += "<p>Folder " + String(folder) + " = " + nb_str + "</p>\n";
    }*/
  ptr += "<form action=\"/mp3\" method=\"get\">\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-folder\">Folder</label>\n";
  ptr += "<input id=\"POST-folder\" type=\"number\" name=\"folder\" step=\"1\" value=\"1\">\n";
  ptr += "<label for=\"POST-file\">File</label>\n";
  ptr += "<input id=\"POST-file\" type=\"number\" name=\"file\" step=\"1\" value=\"1\">\n";
  ptr += "<input type=\"hidden\" name=\"cmd\" value=\"play\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<input class=\"button\" type=\"submit\" value=\"Play\" style=\"width:120px\">\n";
  ptr += "</div>\n";
  ptr += "</form>\n";
  ptr += "</fieldset>\n";

  ptr += "<fieldset><legend>Volume</legend>\n";
  int volume = getVolume();
  ptr += "<p>Current volume = " + String(volume) + "</p>\n";
  ptr += "<form action=\"/mp3\" method=\"get\">\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-vol\">Volume</label>\n";
  ptr += "<input id=\"POST-vol\" type=\"number\" name=\"value\" step=\"1\" value=\"" + String(volume) + "\">\n";
  ptr += "<input type=\"hidden\" name=\"cmd\" value=\"vol\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<input class=\"button\" type=\"submit\" value=\"Set\" style=\"width:120px\">\n";
  ptr += "</div>\n";
  ptr += "</form>\n";
  ptr += "<a class=\"button button-on\" href=\"/mp3?cmd=vol_up\">Vol+</a>\n";
  ptr += "<a class=\"button button-on\" href=\"/mp3?cmd=vol_down\">Vol-</a>\n";
  ptr += "</fieldset>\n";

  ptr += "</div>\n";
  ptr += "<a class=\"\" href=\"/\">Home</a>\n";

  ptr += "</body>\n";
  ptr += "</html>\n";

  _server.send(200, "text/html", ptr);
}
void webOnArt() {
  // args
  _currentDir = "/";
  if (_server.hasArg("dir")) {
    _currentDir = _server.arg("dir");
  }

  // html
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += createHeadHtml("Pixel Arts");
  ptr += "<body>\n";
  ptr += "<h1>Pixel Arts folder [" + String(_eeprom.name) + "]</h1>\n";
  ptr += "<div class=\"form_area\">\n";

  ptr += "<fieldset style=\"text-align:left;\"><legend>Files of " + _currentDir + "</legend>\n";
  File fsDir = SD.open((char*)_currentDir.c_str());
  fsDir.rewindDirectory();
  for (int cnt = 0; true; ++cnt) {
    File fsEntry = fsDir.openNextFile();
    if (!fsEntry) {
      break;
    }
    char filename[32];
    sprintf(filename, "%s/%s", _currentDir.c_str(), fsEntry.name());
    String path = String(filename);    
    if (fsEntry.isDirectory()) {
      ptr += "<div>[DIR] <a href=\"/art?dir=" + path + "\">" + path + "</a></div>";
    } else if (path.endsWith(".bmp") || path.endsWith(".BMP")) {
      ptr += "<div>[IMG] <img src=\"/pixel_download?path=" + path + "\" /> " + path + " (<a href=\"/pixel_delete?path=" + path + "\">delete</a> <a href=\"/pixel_display?path=" + path + "\">show</a>)</div> ";
    } else {
      ptr += "<div>[FILE] " + path + "</div>";
    }
    fsEntry.close();
  }
  fsDir.close();
  ptr += "</fieldset>\n";

  ptr += "<form action=\"/pixel_upload\" method=\"post\" enctype=\"multipart/form-data\">\n";
  ptr += "<label for=\"new\">+ </label>";
  ptr += "<input type=\"file\" id=\"new\" name=\"new\" accept=\"image/bmp\">";
  ptr += "<input type=\"hidden\" id=\"dir\" name=\"dir\" value=\"" + _currentDir + "\">";
  ptr += "<input type=\"submit\" value=\"upload\">\n";
  ptr += "</form>\n";

  ptr += "</div>\n";

  ptr += "<div class=\"form_line\">\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<a class=\"button button-off\" href=\"/\">Back</a>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<a class=\"button button-on\" href=\"/\">Home</a>\n";
  ptr += "</div>\n";
  ptr += "</div>\n";

  ptr += "</body>\n";
  ptr += "</html>\n";

  _server.send(200, "text/html", ptr);
}
void webOnPixelUpload() {
  // args
  // loaded in 'on' function

  // upload file
  HTTPUpload& upload = _server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String path = _currentDir + upload.filename;
    if (!upload.filename.startsWith("/")) {
      path = _currentDir + "/" + upload.filename;
    }
    TRACE("Upload file to %s", path);
    // open file / create if it doesn't exist
    _fsUploadFile = SD.open(path, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (_fsUploadFile) {
      // Write the received bytes to the file
      _fsUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    // If the file was successfully created
    if (_fsUploadFile) {
      // Close the file again
      _fsUploadFile.close();
      TRACE(" - size = %d", upload.totalSize);
      // Redirect the client to the success page
      _server.sendHeader("Location", "/art?dir=" + _currentDir);
      _server.send(303);
    }
  } else {
    _server.send(500, "text/plain", "500: couldn't create file");
  }
}
void webOnPixelDownload() {
  // args
  String path = _server.arg("path");

  String dataType = "text/plain";
  if (path.endsWith(".bmp") || path.endsWith(".BMP")) dataType = "image/bmp";
  File fsFile = SD.open(path.c_str());
  _server.streamFile(fsFile, dataType);
  fsFile.close();
}
void webOnPixelDelete() {
  // args
  String path = _server.arg("path");
  // get parent directory
  _currentDir = path.substring(0, path.lastIndexOf("/"));
  if (_currentDir.length() == 0) {
    _currentDir = "/";
  }

  TRACE("Delete %s", path);
  if (path == "/") {
    return _server.send(500, "text/plain", "BAD PATH");
  }
  if (!SD.exists(path)) {
    return _server.send(404, "text/plain", "FileNotFound");
  }
  SD.remove(path);

  // Redirect the client to the success page
  _server.sendHeader("Location", "/art?dir=" + _currentDir);
  _server.send(303);
}
void webOnPixelDisplay() {
  // args
  String path = _server.arg("path");
  // get parent directory
  _currentDir = path.substring(0, path.lastIndexOf("/"));
  if (_currentDir.length() == 0) {
    _currentDir = "/";
  }

  TRACE("Force display of %s", path.c_str());
  // display pixel
  clearAll(RgbColor(0));
  drawBmp(0, 0, path.c_str());
  displayMatrix();

  // Redirect the client to the success page
  _server.sendHeader("Location", "/art?dir=" + _currentDir);
  _server.send(303);
}
void webOnProperties() {
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += createHeadHtml("Pixel properties");
  ptr += "<body>\n";
  ptr += "<h1>Pixel properties [" + String(_eeprom.name) + "]</h1>\n";
  ptr += "<form action=\"/configure\" method=\"post\">\n";
  ptr += "<div class=\"form_area\">\n";

  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-name\">Name</label>\n";
  ptr += "<input id=\"POST-name\" type=\"text\" name=\"name\" size=\"8\" value=\"" + String(_eeprom.name) + "\">\n";
  ptr += "</div>\n";

  ptr += "<fieldset><legend>WIFI</legend>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-wifi_ssid\">SSID : </label>\n";
  ptr += "<input id=\"POST-wifi_ssid\" type=\"text\" name=\"wifi_ssid\" size=\"24\" value=\"" + String(_eeprom.wifi_ssid) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-wifi_key\">Key : </label>\n";
  ptr += "<input id=\"POST-wifi_key\" type=\"text\" name=\"wifi_key\" size=\"24\" value=\"" + String(_eeprom.wifi_key) + "\">\n";
  ptr += "</div>\n";
  ptr += "</fieldset>\n";

  ptr += "<fieldset><legend>MQTT</legend>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-mqtt_server\">Server : </label>\n";
  ptr += "<input id=\"POST-mqtt_server\" type=\"text\" name=\"mqtt_server\" value=\"" + String(_eeprom.mqtt_server) + "\">\n";
  ptr += "<input id=\"POST-mqtt_server\" type=\"number\" name=\"mqtt_port\" min=\"0\" max=\"49151\" step=\"1\" value=\"" + String(_eeprom.mqtt_port) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-mqtt_user\">User : </label>\n";
  ptr += "<input id=\"POST-mqtt_user\" type=\"text\" name=\"mqtt_user\" size=\"30\" value=\"" + String(_eeprom.mqtt_user) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-mqtt_pwd\">Password : </label>\n";
  ptr += "<input id=\"POST-mqtt_pwd\" type=\"text\" name=\"mqtt_pwd\" size=\"30\" value=\"" + String(_eeprom.mqtt_pwd) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-mqtt_channel\">Channel : </label>\n";
  ptr += "<input id=\"POST-mqtt_channel\" type=\"text\" name=\"mqtt_channel\" value=\"" + String(_eeprom.mqtt_channel) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-mqtt_res_temp\">Resource temperature : </label>\n";
  ptr += "<input id=\"POST-mqtt_res_temp\" type=\"text\" name=\"mqtt_res_temp\" size=\"8\" value=\"" + String(_eeprom.mqtt_res_temp) + "\">\n";
  ptr += "</div>\n";
  ptr += "</fieldset>\n";

  ptr += "<fieldset><legend>Open weather</legend>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-weather_appid\">App ID : </label>\n";
  ptr += "<input id=\"POST-weather_appid\" type=\"text\" name=\"weather_appid\" size=\"32\" value=\"" + String(_eeprom.weather_appid) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-weather_location\">Location : </label>\n";
  ptr += "<input id=\"POST-weather_location\" type=\"text\" name=\"weather_location\" value=\"" + String(_eeprom.weather_location) + "\">\n";
  ptr += "</div>\n";
  ptr += "</fieldset>\n";

  ptr += "<fieldset><legend>Night mode</legend>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-night_threshold\">Night threshold : </label>\n";
  ptr += "<input id=\"POST-night_threshold\" type=\"number\" name=\"night_threshold\" min=\"0\" max=\"4095\" step=\"1\" value=\"" + String(_eeprom.night_threshold) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-night_hour\">Night hour RGB : </label>\n";
  ptr += "<input id=\"POST-night_hour\" type=\"number\" name=\"night_hour_r\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_hour_r) + "\">\n";
  ptr += "<input id=\"POST-night_hour\" type=\"number\" name=\"night_hour_g\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_hour_g) + "\">\n";
  ptr += "<input id=\"POST-night_hour\" type=\"number\" name=\"night_hour_b\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_hour_b) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-night_min\">Night minutes RGB : </label>\n";
  ptr += "<input id=\"POST-night_min\" type=\"number\" name=\"night_min_r\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_min_r) + "\">\n";
  ptr += "<input id=\"POST-night_min\" type=\"number\" name=\"night_min_g\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_min_g) + "\">\n";
  ptr += "<input id=\"POST-night_min\" type=\"number\" name=\"night_min_b\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_min_b) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-night_tempi\">Night IN temp. RGB : </label>\n";
  ptr += "<input id=\"POST-night_tempi\" type=\"number\" name=\"night_tempi_r\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_tempi_r) + "\">\n";
  ptr += "<input id=\"POST-night_tempi\" type=\"number\" name=\"night_tempi_g\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_tempi_g) + "\">\n";
  ptr += "<input id=\"POST-night_tempi\" type=\"number\" name=\"night_tempi_b\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_tempi_b) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-night_tempe\">Night OUT temp. RGB : </label>\n";
  ptr += "<input id=\"POST-night_tempe\" type=\"number\" name=\"night_tempe_r\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_tempe_r) + "\">\n";
  ptr += "<input id=\"POST-night_tempe\" type=\"number\" name=\"night_tempe_g\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_tempe_g) + "\">\n";
  ptr += "<input id=\"POST-night_tempe\" type=\"number\" name=\"night_tempe_b\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.night_tempe_b) + "\">\n";
  ptr += "</div>\n";
  ptr += "</fieldset>\n";

  ptr += "<fieldset><legend>Brightness</legend>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-bright_max\">Brightness max. (do not change) : </label>\n";
  ptr += "<input id=\"POST-bright_max\" type=\"number\" name=\"bright_max\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.bright_max) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-bright_threshold\">Update threshold : </label>\n";
  ptr += "<input id=\"POST-bright_threshold\" type=\"number\" name=\"bright_threshold\" min=\"0\" max=\"100\" step=\"1\" value=\"" + String(_eeprom.bright_threshold) + "\">%\n";
  ptr += "</div>\n";
  for (int i = 0; i < 30; i++) {
    String iStr = String(i);
    ptr += "<div class=\"form_line\">\n";
    ptr += "<label for=\"POST-bright_map" + iStr + "\">Brightness for " + String(1000 + 100 * i) + " : </label>\n";
    ptr += "<input id=\"POST-bright_map" + iStr + "\" type=\"number\" name=\"bright_map" + iStr + "\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.bright_map[i]) + "\">\n";
    ptr += "</div>\n";
  }
  ptr += "</fieldset>\n";

  ptr += "<fieldset><legend>IR commands</legend>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-ir_cmd_temp_sum\">Temperature command : </label>\n";
  ptr += "<input id=\"POST-ir_cmd_temp_sum\" type=\"number\" name=\"ir_cmd_temp_sum\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.ir_cmd_temp_sum) + "\">\n";
  ptr += "</div>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<label for=\"POST-ir_cmd_ip\">Test command : </label>\n";
  ptr += "<input id=\"POST-ir_cmd_ip\" type=\"number\" name=\"ir_cmd_ip\" min=\"0\" max=\"255\" step=\"1\" value=\"" + String(_eeprom.ir_cmd_ip) + "\">\n";
  ptr += "</div>\n";
  ptr += "</fieldset>\n";

  ptr += "<div class=\"form_line\">\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<a class=\"button button-off\" href=\"/\">Back</a>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<input class=\"button button-on\" type=\"submit\" value=\"Save\" style=\"width:120px\">\n";
  ptr += "</div>\n";
  ptr += "</div>\n";

  ptr += "</form>\n";
  ptr += "</body>\n";
  ptr += "</html>\n";

  _server.send(200, "text/html", ptr);
}
void webOnConfigure() {
  // update data
  strcpy(_eeprom.name, _server.arg("name").c_str());
  // night mode
  _eeprom.night_threshold = _server.arg("night_threshold").toInt();
  _eeprom.night_hour_r = _server.arg("night_hour_r").toInt();
  _eeprom.night_hour_g = _server.arg("night_hour_g").toInt();
  _eeprom.night_hour_b = _server.arg("night_hour_b").toInt();
  _eeprom.night_min_r = _server.arg("night_min_r").toInt();
  _eeprom.night_min_g = _server.arg("night_min_g").toInt();
  _eeprom.night_min_b = _server.arg("night_min_b").toInt();
  _eeprom.night_tempi_r = _server.arg("night_tempi_r").toInt();
  _eeprom.night_tempi_g = _server.arg("night_tempi_g").toInt();
  _eeprom.night_tempi_b = _server.arg("night_tempi_b").toInt();
  _eeprom.night_tempe_r = _server.arg("night_tempe_r").toInt();
  _eeprom.night_tempe_g = _server.arg("night_tempe_g").toInt();
  _eeprom.night_tempe_b = _server.arg("night_tempe_b").toInt();
  // IR commands
  _eeprom.ir_cmd_temp_sum = _server.arg("ir_cmd_temp_sum").toInt();
  _eeprom.ir_cmd_ip = _server.arg("ir_cmd_ip").toInt();
  // brightness parameters
  _eeprom.bright_max = _server.arg("bright_max").toInt();
  for (int i = 0; i < 30; i++) {
    _eeprom.bright_map[i] = _server.arg("bright_map" + String(i)).toInt();
  }
  _eeprom.bright_threshold = _server.arg("bright_threshold").toInt();
  // WIFI
  strcpy(_eeprom.wifi_ssid, _server.arg("wifi_ssid").c_str());
  strcpy(_eeprom.wifi_key, _server.arg("wifi_key").c_str());
  // MQTT
  strcpy(_eeprom.mqtt_server, _server.arg("mqtt_server").c_str());
  _eeprom.mqtt_port = _server.arg("mqtt_port").toInt();
  strcpy(_eeprom.mqtt_channel, _server.arg("mqtt_channel").c_str());
  strcpy(_eeprom.mqtt_user, _server.arg("mqtt_user").c_str());
  strcpy(_eeprom.mqtt_pwd, _server.arg("mqtt_pwd").c_str());
  strcpy(_eeprom.mqtt_res_temp, _server.arg("mqtt_res_temp").c_str());
  // OPEN WEATHER
  strcpy(_eeprom.weather_appid, _server.arg("weather_appid").c_str());
  strcpy(_eeprom.weather_location, _server.arg("weather_location").c_str());
  // save
  saveEEPROM();

  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += createHeadHtml("Pixel properties");
  ptr += "<body>\n";
  ptr += "<h1>Properties saved [" + String(_eeprom.name) + "]</h1>\n";
  ptr += "<div class=\"form_line\">\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<a class=\"button button-off\" href=\"/properties\">Back</a>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<a class=\"button button-on\" href=\"/\">Home</a>\n";
  ptr += "</div>\n";
  ptr += "</div>\n";

  ptr += "</body>\n";
  ptr += "</html>\n";

  _server.send(200, "text/html", ptr);
}

void webOnFirmware() {
  // html
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += createHeadHtml("Pixel firmware");
  ptr += "<body>\n";
  ptr += "<h1>Pixel Firmware [" + String(_eeprom.name) + "]</h1>\n";
  ptr += "<div class=\"form_area\">\n";
  ptr += "<form action=\"/firmware_upload\" method=\"post\" enctype=\"multipart/form-data\">\n";
  ptr += "<label for=\"firmware\">Firmware file : </label>";
  ptr += "<input type=\"file\" id=\"firmware\" name=\"firmware\">";
  ptr += "<input type=\"submit\" value=\"upload\">\n";
  ptr += "</form>\n";
  ptr += "</div>\n";

  ptr += "<div class=\"form_line\">\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<a class=\"button button-off\" href=\"/\">Back</a>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"area_bt\">\n";
  ptr += "<a class=\"button button-on\" href=\"/\">Home</a>\n";
  ptr += "</div>\n";
  ptr += "</div>\n";

  ptr += "</body>\n";
  ptr += "</html>\n";

  _server.send(200, "text/html", ptr);
}
void webOnFirmwareUpload() {
  // display pixel
  clearAll(RgbColor(0));
  drawBmp(0, 0, "/common/alert_firmware.bmp");
  displayMatrix();

  // upload file
  HTTPUpload& upload = _server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.setDebugOutput(true);
    TRACE("Updating firmware with %s", upload.filename);
    if (!Update.begin()) {  //start with max available size
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {  //true to set the size to the current progress
      TRACE("Rebooting...");
    } else {
      Update.printError(Serial);
    }
    Serial.setDebugOutput(false);
  } else {
    TRACE("Update Failed Unexpectedly (likely broken connection): status=%d", upload.status);
  }
}
void webNotFound() {
  _server.send(404, "text/plain", "Not found");
}
