#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <ESPAsyncTCP.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>

#ifdef ENABLE_MQTT
#include <PubSubClient.h>
#endif

#ifdef ENABLE_NETWORK
ArduinoOTAClass arduinoOTA;
ESP8266WebServer httpServer(80);
#endif

#if defined(ENABLE_NETWORK) && defined(ENABLE_CO2_SENSOR)
bool do_s8_calibration = false;
int ignore_s8_iteration_count = 0;
bool s8_calibration_failed = false;
#endif

#define SERVER_HOSTNAME_MAX_LEN 64
char serverHostname[SERVER_HOSTNAME_MAX_LEN] = "air-quality-server.local";
int serverPort = 1234;
float temperatureOffset = -3.4f;

#define MQTT_HOST_MAX_LEN 64
#define MQTT_USER_MAX_LEN 64
#define MQTT_PASS_MAX_LEN 64
char mqttHost[MQTT_HOST_MAX_LEN] = "";
int mqttPort = 1883;
char mqttUser[MQTT_USER_MAX_LEN] = "";
char mqttPass[MQTT_PASS_MAX_LEN] = "";
bool tcpLoggingEnabled = false;
bool mqttEnabled = false;
bool mqttHaDiscoveryEnabled = false;
bool serialLoggingEnabled = false;

#ifdef ENABLE_SERIAL_DEBUGGING
#define DEBUG_PRINT(X) Serial.print(X)
#define DEBUG_PRINT_2(X, Y) Serial.print(X, Y)
#else
#define DEBUG_PRINT(X)
#define DEBUG_PRINT_2(X, Y)
#endif

#ifdef ENABLE_CO2_SENSOR
// CO2 sensor RX/ESP8266 TX: D0
// CO2 sensor TX/ESP8266 RX: D7
SoftwareSerial co2Sensor(D7, D0);
#endif

#ifdef ENABLE_PARTICLE_SENSOR
// Particle sensor RX/ESP8266 TX: D5
// Particle sensor TX/ESP8266 RX: D6
SoftwareSerial particleCounter(D6, D5);
#endif

uint16_t co2SensorStatus = -1;
uint16_t co2SensorPpm = -1;
uint16_t particleSensorPm1_0 = -1;
uint16_t particleSensorPm2_5 = -1;
uint16_t particleSensorPm10_0 = -1;
uint16_t particleSensorParticle0_3um = -1;
uint16_t particleSensorParticle0_5um = -1;
uint16_t particleSensorParticle1_0um = -1;
uint16_t particleSensorParticle2_5um = -1;
uint16_t particleSensorParticle5_0um = -1;
uint16_t particleSensorParticle10_0um = -1;
uint16_t particleSensorChecksumOk = -1;
float sht31Humidity = -1.0f;
float sht31Temperature = -1.0f;
uint16_t sht31HumidityChecksumOk = -1;
uint16_t sht31TemperatureChecksumOk = -1;
uint16_t sgp30co2eqPpm = -1;
uint16_t sgp30tvocPpb = -1;
uint16_t sgp30co2eqChecksumOk = -1;
uint16_t sgp30tvocChecksumOk = -1;

String macAddress("");
#ifdef ENABLE_NETWORK_LOGGING
AsyncClient asyncClient;
#define MESSAGE_MAX_LEN 256
char messageBuffer[MESSAGE_MAX_LEN + 1] = {0};
#endif

void enableSht31Heater() {
#ifdef ENABLE_SHT31_HEATER
  Wire.beginTransmission(0x44);
  Wire.write(0x30);
  Wire.write(0x6D);
  Wire.endTransmission();
#endif
}

void disableSht31Heater() {
  Wire.beginTransmission(0x44);
  Wire.write(0x30);
  Wire.write(0x66);
  Wire.endTransmission();
}

void writeParticleCounterCommand(uint8_t cmd, uint8_t dataH, uint8_t dataL) {
#ifdef ENABLE_PARTICLE_SENSOR
  uint16_t checksum = 0x42 + 0x4D + cmd + dataH + dataL;
  particleCounter.write(0x42);
  particleCounter.write(0x4D);
  particleCounter.write(cmd);
  particleCounter.write(dataH);
  particleCounter.write(dataL);
  particleCounter.write((checksum & 0xFF00) >> 8);
  particleCounter.write(checksum & 0xFF);
#endif
}

void writeSingleLineStatusItem(uint16_t value, bool isLast = false) {
#ifdef ENABLE_SERIAL_LOGGING
  Serial.print(value, DEC);
  if (!isLast) {
    Serial.print(",");
  } else {
    Serial.print("\n");
  }
#endif
}

void writeSingleLineStatusItem(float value, bool isLast = false) {
#ifdef ENABLE_SERIAL_LOGGING
  Serial.print(value);
  if (!isLast) {
    Serial.print(",");
  } else {
    Serial.print("\n");
  }
#endif
}

void appendSingleLineStatusItem(String& s, String& s2, bool isLast = false) {
  s += s2;
  if (!isLast) {
    s += ",";
  } else {
    s += "\n";
  }
}

void appendSingleLineStatusItem(String& s, uint16_t value, bool isLast = false) {
  String s2(value, DEC);
  appendSingleLineStatusItem(s, s2, isLast);
}

void appendSingleLineStatusItem(String& s, uint32_t value, bool isLast = false) {
  String s2(value, DEC);
  appendSingleLineStatusItem(s, s2, isLast);
}

void appendSingleLineStatusItem(String& s, unsigned long value, bool isLast = false) {
  String s2(value, DEC);
  appendSingleLineStatusItem(s, s2, isLast);
}

void appendSingleLineStatusItem(String& s, float value, bool isLast = false) {
  String s2(value);
  appendSingleLineStatusItem(s, s2, isLast);
}

void resyncStream() {
#ifdef ENABLE_PARTICLE_SENSOR
  while(particleCounter.available()) {
    particleCounter.read();
  }
#endif
}

const uint8_t updateCrc(const uint8_t crc, const uint8_t value, const uint8_t polynomial) {
  uint8_t wcrc = crc ^ value;

  for(int i = 8; 0 < i; i--) {
    wcrc = (wcrc & 0x80) ? (wcrc << 1) ^ polynomial : (wcrc << 1);
  }

  return wcrc;
}

uint8_t sensirionCrc8(uint8_t msb, uint8_t lsb) {
  const uint8_t polynomial = 0x31;
  uint8_t crc = 0xFF;

  crc = updateCrc(crc, msb, polynomial);
  crc = updateCrc(crc, lsb, polynomial);

  return crc;
}

uint16_t modbusCrc16(uint8_t* data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void loadConfig() {
  if (!LittleFS.exists("/config.txt")) return;

  File f = LittleFS.open("/config.txt", "r");
  if (!f) return;

  String firstLine = f.readStringUntil('\n');
  firstLine.trim();

  if (firstLine.indexOf('=') == -1) {
    // Old 3-line positional format
    if (firstLine.length() > 0 && firstLine.length() < SERVER_HOSTNAME_MAX_LEN) {
      strncpy(serverHostname, firstLine.c_str(), SERVER_HOSTNAME_MAX_LEN - 1);
      serverHostname[SERVER_HOSTNAME_MAX_LEN - 1] = '\0';
    }
    String port = f.readStringUntil('\n');
    port.trim();
    if (port.length() > 0) {
      serverPort = port.toInt();
    }
    String tempOffset = f.readStringUntil('\n');
    tempOffset.trim();
    if (tempOffset.length() > 0) {
      temperatureOffset = tempOffset.toFloat();
    }
    f.close();
    return;
  }

  // New key-value format
  auto processLine = [](const String& line) {
    int eqPos = line.indexOf('=');
    if (eqPos == -1) return;
    String key = line.substring(0, eqPos);
    String value = line.substring(eqPos + 1);
    key.trim();
    value.trim();

    if (key == "server_hostname" && value.length() > 0 && value.length() < SERVER_HOSTNAME_MAX_LEN) {
      strncpy(serverHostname, value.c_str(), SERVER_HOSTNAME_MAX_LEN - 1);
      serverHostname[SERVER_HOSTNAME_MAX_LEN - 1] = '\0';
    } else if (key == "server_port" && value.length() > 0) {
      serverPort = value.toInt();
    } else if (key == "temperature_offset" && value.length() > 0) {
      temperatureOffset = value.toFloat();
    } else if (key == "mqtt_host" && value.length() < MQTT_HOST_MAX_LEN) {
      strncpy(mqttHost, value.c_str(), MQTT_HOST_MAX_LEN - 1);
      mqttHost[MQTT_HOST_MAX_LEN - 1] = '\0';
    } else if (key == "mqtt_port" && value.length() > 0) {
      mqttPort = value.toInt();
    } else if (key == "mqtt_user" && value.length() < MQTT_USER_MAX_LEN) {
      strncpy(mqttUser, value.c_str(), MQTT_USER_MAX_LEN - 1);
      mqttUser[MQTT_USER_MAX_LEN - 1] = '\0';
    } else if (key == "mqtt_pass" && value.length() < MQTT_PASS_MAX_LEN) {
      strncpy(mqttPass, value.c_str(), MQTT_PASS_MAX_LEN - 1);
      mqttPass[MQTT_PASS_MAX_LEN - 1] = '\0';
    } else if (key == "tcp_logging_enabled") {
      tcpLoggingEnabled = (value == "1");
    } else if (key == "mqtt_enabled") {
      mqttEnabled = (value == "1");
    } else if (key == "mqtt_ha_discovery") {
      mqttHaDiscoveryEnabled = (value == "1");
    } else if (key == "serial_logging_enabled") {
      serialLoggingEnabled = (value == "1");
    }
  };

  processLine(firstLine);
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      processLine(line);
    }
  }
  f.close();
}

void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (!f) return;

  f.print("server_hostname="); f.println(serverHostname);
  f.print("server_port="); f.println(serverPort);
  f.print("temperature_offset="); f.println(temperatureOffset);
  f.print("mqtt_host="); f.println(mqttHost);
  f.print("mqtt_port="); f.println(mqttPort);
  f.print("mqtt_user="); f.println(mqttUser);
  f.print("mqtt_pass="); f.println(mqttPass);
  f.print("tcp_logging_enabled="); f.println(tcpLoggingEnabled ? "1" : "0");
  f.print("mqtt_enabled="); f.println(mqttEnabled ? "1" : "0");
  f.print("mqtt_ha_discovery="); f.println(mqttHaDiscoveryEnabled ? "1" : "0");
  f.print("serial_logging_enabled="); f.println(serialLoggingEnabled ? "1" : "0");

  f.close();
}

#ifdef ENABLE_NETWORK
void handleRoot() {
  String html = "<html><body><h2>Air Quality Monitor</h2>"
    "<ul>"
    "<li><a href='/status'>Status</a></li>"
    "<li><a href='/config'>Configuration</a></li>"
    "<li><a href='/calibrate'>Calibrate CO2 Sensor</a></li>"
    "<li><a href='/factoryreset'>Factory Reset</a></li>"
    "</ul></body></html>";
  httpServer.send(200, "text/html", html);
}

void handleFactoryReset() {
  httpServer.send(200, "text/plain", "Factory reset, rebooting...\n");
  LittleFS.format();
  ESP.eraseConfig();
  ESP.restart();
}

void handleStatus() {
  String json = "{\"config\":{\"hostname\":\"" + String(serverHostname) + "\","
    "\"port\":" + String(serverPort) + ","
    "\"temperatureOffset\":" + String(temperatureOffset) + ","
    "\"mqttHost\":\"" + String(mqttHost) + "\","
    "\"mqttPort\":" + String(mqttPort) + ","
    "\"tcpLoggingEnabled\":" + String(tcpLoggingEnabled ? "true" : "false") + ","
    "\"mqttEnabled\":" + String(mqttEnabled ? "true" : "false") + ","
    "\"mqttHaDiscoveryEnabled\":" + String(mqttHaDiscoveryEnabled ? "true" : "false") + ","
    "\"serialLoggingEnabled\":" + String(serialLoggingEnabled ? "true" : "false") + "},"
    "\"status\":{"
    "\"macAddress\":\"" + macAddress + "\","
    "\"uptimeMs\":" + String(millis()) + ","
    "\"co2SensorStatus\":" + String(co2SensorStatus) + ","
    "\"co2SensorPpm\":" + String(co2SensorPpm) + ","
    "\"particleSensorChecksumOk\":" + String(particleSensorChecksumOk) + ","
    "\"particleSensorPm1_0\":" + String(particleSensorPm1_0) + ","
    "\"particleSensorPm2_5\":" + String(particleSensorPm2_5) + ","
    "\"particleSensorPm10_0\":" + String(particleSensorPm10_0) + ","
    "\"particleSensorParticle0_3um\":" + String(particleSensorParticle0_3um) + ","
    "\"particleSensorParticle0_5um\":" + String(particleSensorParticle0_5um) + ","
    "\"particleSensorParticle1_0um\":" + String(particleSensorParticle1_0um) + ","
    "\"particleSensorParticle2_5um\":" + String(particleSensorParticle2_5um) + ","
    "\"particleSensorParticle5_0um\":" + String(particleSensorParticle5_0um) + ","
    "\"particleSensorParticle10_0um\":" + String(particleSensorParticle10_0um) + ","
    "\"sht31Temperature\":" + String(sht31Temperature) + ","
    "\"sht31Humidity\":" + String(sht31Humidity) + ","
    "\"sht31TemperatureChecksumOk\":" + String(sht31TemperatureChecksumOk) + ","
    "\"sht31HumidityChecksumOk\":" + String(sht31HumidityChecksumOk) + ","
    "\"sgp30co2eqPpm\":" + String(sgp30co2eqPpm) + ","
    "\"sgp30tvocPpb\":" + String(sgp30tvocPpb) + ","
    "\"sgp30co2eqChecksumOk\":" + String(sgp30co2eqChecksumOk) + ","
    "\"sgp30tvocChecksumOk\":" + String(sgp30tvocChecksumOk) + "}}";
  httpServer.send(200, "application/json", json);
}

void handleConfigGet() {
  String html = "<html><body><h2>Sensor Configuration</h2>"
    "<form method='POST' action='/config'>"
    "<h3>Server</h3>"
    "<label>Server hostname: <input name='hostname' value='" + String(serverHostname) + "' maxlength='63'></label><br><br>"
    "<label>Server port: <input name='port' type='number' value='" + String(serverPort) + "'></label><br><br>"
    "<label>Temperature offset: <input name='tempoffset' type='number' step='0.1' value='" + String(temperatureOffset) + "'></label><br><br>"
    "<h3>MQTT</h3>"
    "<label>MQTT host: <input name='mqtt_host' value='" + String(mqttHost) + "' maxlength='63'></label><br><br>"
    "<label>MQTT port: <input name='mqtt_port' type='number' value='" + String(mqttPort) + "'></label><br><br>"
    "<label>MQTT username: <input name='mqtt_user' value='" + String(mqttUser) + "' maxlength='63'></label><br><br>"
    "<label>MQTT password: <input name='mqtt_pass' type='password' value='" + String(mqttPass) + "' maxlength='63'></label><br><br>"
    "<h3>Logging</h3>"
    "<label><input name='tcp_logging' type='checkbox'" + String(tcpLoggingEnabled ? " checked" : "") + "> TCP logging</label><br><br>"
    "<label><input name='mqtt_logging' type='checkbox'" + String(mqttEnabled ? " checked" : "") + "> MQTT publishing</label><br><br>"
    "<label><input name='mqtt_ha' type='checkbox'" + String(mqttHaDiscoveryEnabled ? " checked" : "") + "> MQTT HA discovery</label><br><br>"
    "<label><input name='serial_logging' type='checkbox'" + String(serialLoggingEnabled ? " checked" : "") + "> Serial logging</label><br><br>"
    "<input type='submit' value='Save'>"
    "</form></body></html>";
  httpServer.send(200, "text/html", html);
}

void handleConfigPost() {
  if (httpServer.hasArg("hostname") && httpServer.arg("hostname").length() > 0 &&
      httpServer.hasArg("port") && httpServer.arg("port").length() > 0 &&
      httpServer.hasArg("tempoffset") && httpServer.arg("tempoffset").length() > 0) {
    String hostname = httpServer.arg("hostname");
    if (hostname.length() < SERVER_HOSTNAME_MAX_LEN) {
      strncpy(serverHostname, hostname.c_str(), SERVER_HOSTNAME_MAX_LEN - 1);
      serverHostname[SERVER_HOSTNAME_MAX_LEN - 1] = '\0';
    }
    serverPort = httpServer.arg("port").toInt();
    temperatureOffset = httpServer.arg("tempoffset").toFloat();

    if (httpServer.hasArg("mqtt_host")) {
      String mh = httpServer.arg("mqtt_host");
      if (mh.length() < MQTT_HOST_MAX_LEN) {
        strncpy(mqttHost, mh.c_str(), MQTT_HOST_MAX_LEN - 1);
        mqttHost[MQTT_HOST_MAX_LEN - 1] = '\0';
      }
    }
    if (httpServer.hasArg("mqtt_port") && httpServer.arg("mqtt_port").length() > 0) {
      mqttPort = httpServer.arg("mqtt_port").toInt();
    }
    if (httpServer.hasArg("mqtt_user")) {
      String mu = httpServer.arg("mqtt_user");
      if (mu.length() < MQTT_USER_MAX_LEN) {
        strncpy(mqttUser, mu.c_str(), MQTT_USER_MAX_LEN - 1);
        mqttUser[MQTT_USER_MAX_LEN - 1] = '\0';
      }
    }
    if (httpServer.hasArg("mqtt_pass")) {
      String mp = httpServer.arg("mqtt_pass");
      if (mp.length() < MQTT_PASS_MAX_LEN) {
        strncpy(mqttPass, mp.c_str(), MQTT_PASS_MAX_LEN - 1);
        mqttPass[MQTT_PASS_MAX_LEN - 1] = '\0';
      }
    }

    // Checkboxes: present in POST = checked, absent = unchecked
    tcpLoggingEnabled = httpServer.hasArg("tcp_logging");
    mqttEnabled = httpServer.hasArg("mqtt_logging");
    mqttHaDiscoveryEnabled = httpServer.hasArg("mqtt_ha");
    serialLoggingEnabled = httpServer.hasArg("serial_logging");

    saveConfig();
    httpServer.send(200, "text/html", "<html><body><h2>Configuration saved</h2>"
      "<p>Rebooting...</p></body></html>");
    delay(500);
    ESP.restart();
  } else {
    httpServer.send(400, "text/plain", "Missing hostname, port, or temperature offset\n");
  }
}
#endif

#if defined(ENABLE_NETWORK) && defined(ENABLE_CO2_SENSOR)
void handleCalibrate() {
  do_s8_calibration = true;
  httpServer.send(200, "text/plain", "Calibration scheduled\n");
}
#endif

bool shouldSaveConfig = false;

void setup() {
  Serial.begin(9600);

  LittleFS.begin();
  loadConfig();

#if defined(ENABLE_SHT31) || defined(ENABLE_SGP30)
  Wire.begin();
#endif

  // Init SHT31
#ifdef ENABLE_SHT31
  disableSht31Heater();
#else
  DEBUG_PRINT("SHT31 disabled\n");
#endif

#ifdef ENABLE_SGP30
  Wire.beginTransmission(0x58);
  // Init_air_quality: 0x2003
  Wire.write(0x20);
  Wire.write(0x03);
  Wire.endTransmission();
#else
  DEBUG_PRINT("SGP30 disabled\n");
#endif

  // Init software UARTs
#ifdef ENABLE_CO2_SENSOR
  co2Sensor.begin(9600);
#else
  DEBUG_PRINT("CO2 sensor disabled\n");
#endif

  DEBUG_PRINT("CRC");
  DEBUG_PRINT_2(sensirionCrc8(0xBE, 0xEF), HEX);
  DEBUG_PRINT("\n");

#ifdef ENABLE_PARTICLE_SENSOR
  particleCounter.begin(9600);
  // Set particle counter to passive mode
  writeParticleCounterCommand(0xE1, 0x00, 0x00); // 0xE1 change mode -> 0x00 passive mode
#else
  DEBUG_PRINT("Particle sensor disabled\n");
#endif

#ifdef ENABLE_NETWORK
  WiFiManager wifiManager;

  char portStr[6];
  snprintf(portStr, sizeof(portStr), "%d", serverPort);
  char tempOffsetStr[8];
  snprintf(tempOffsetStr, sizeof(tempOffsetStr), "%.1f", temperatureOffset);
  WiFiManagerParameter hostnameParam("hostname", "Server hostname", serverHostname, SERVER_HOSTNAME_MAX_LEN);
  WiFiManagerParameter portParam("port", "Server port", portStr, 6);
  WiFiManagerParameter tempOffsetParam("tempoffset", "Temperature offset", tempOffsetStr, 8);
  wifiManager.addParameter(&hostnameParam);
  wifiManager.addParameter(&portParam);
  wifiManager.addParameter(&tempOffsetParam);
  wifiManager.setSaveConfigCallback([]() { shouldSaveConfig = true; });

  if(!wifiManager.autoConnect()) {
    Serial.println("Failed to connect to wifi");
    ESP.restart();
    delay(1000);
  }

  if (shouldSaveConfig) {
    strncpy(serverHostname, hostnameParam.getValue(), SERVER_HOSTNAME_MAX_LEN - 1);
    serverHostname[SERVER_HOSTNAME_MAX_LEN - 1] = '\0';
    serverPort = atoi(portParam.getValue());
    temperatureOffset = atof(tempOffsetParam.getValue());
    saveConfig();
  }

  arduinoOTA.begin();

  httpServer.on("/", handleRoot);
  httpServer.on("/status", handleStatus);
  httpServer.on("/config", HTTP_GET, handleConfigGet);
  httpServer.on("/config", HTTP_POST, handleConfigPost);
  httpServer.on("/factoryreset", handleFactoryReset);
#ifdef ENABLE_CO2_SENSOR
  httpServer.on("/calibrate", handleCalibrate);
#endif
  httpServer.begin();
#endif

#ifdef ENABLE_NETWORK_LOGGING
  uint8_t hwAddr[6];
  WiFi.macAddress(hwAddr);
  macAddress.reserve(12);
  for(int i = 0; i < 6; ++i) {
    macAddress += String(hwAddr[i], HEX);
  }
#endif

  // Wait for hardware to initialize (~15s for SGP30)
  delay(20000);
}

void loop() {
  co2SensorStatus = -1;
  co2SensorPpm = -1;
  particleSensorPm1_0 = -1;
  particleSensorPm2_5 = -1;
  particleSensorPm10_0 = -1;
  particleSensorParticle0_3um = -1;
  particleSensorParticle0_5um = -1;
  particleSensorParticle1_0um = -1;
  particleSensorParticle2_5um = -1;
  particleSensorParticle5_0um = -1;
  particleSensorParticle10_0um = -1;
  particleSensorChecksumOk = -1;
  sht31Humidity = -1.0f;
  sht31Temperature = -1.0f;
  sht31HumidityChecksumOk = -1;
  sht31TemperatureChecksumOk = -1;
  sgp30co2eqPpm = -1;
  sgp30tvocPpb = -1;
  sgp30co2eqChecksumOk = -1;
  sgp30tvocChecksumOk = -1;

#ifdef ENABLE_CO2_SENSOR
  if(co2Sensor.available()) {
    while(co2Sensor.available()) {
      co2Sensor.read();
    }
  }
#endif

#if defined(ENABLE_NETWORK) && defined(ENABLE_CO2_SENSOR)
  if (do_s8_calibration) {
    do_s8_calibration = false;
    DEBUG_PRINT("Sending S8 background calibration command\n");
    co2Sensor.listen();
    // Background calibration command: write HR1 = 0x7C06
    co2Sensor.write(0xFE);
    co2Sensor.write(0x06);
    co2Sensor.write(0x00);
    co2Sensor.write(0x01);
    co2Sensor.write(0x7C);
    co2Sensor.write(0x06);
    co2Sensor.write(0x6C);
    co2Sensor.write(0xC7);
    ignore_s8_iteration_count = 5;
  }

  if (ignore_s8_iteration_count > 0) {
    ignore_s8_iteration_count--;
    if (ignore_s8_iteration_count == 0) {
      // Read HR1 to check calibration acknowledgement
      co2Sensor.listen();
      co2Sensor.write(0xFE);
      co2Sensor.write(0x03);
      co2Sensor.write(0x00);
      co2Sensor.write(0x00);
      co2Sensor.write(0x00);
      co2Sensor.write(0x01);
      co2Sensor.write(0x90);
      co2Sensor.write(0x05);
      delay(100);

      if (co2Sensor.available() && co2Sensor.available() >= 7) {
        uint8_t resp[7];
        for (int i = 0; i < 7; ++i) {
          resp[i] = co2Sensor.read();
        }
        uint16_t hr1 = (resp[3] << 8) | resp[4];
        // Bit 5 of HR1 indicates acknowledgement
        if (hr1 & 0x0020) {
          DEBUG_PRINT("S8 calibration acknowledged\n");
          s8_calibration_failed = false;
        } else {
          DEBUG_PRINT("S8 calibration NOT acknowledged\n");
          s8_calibration_failed = true;
        }
      } else {
        DEBUG_PRINT("S8 calibration status read failed\n");
        s8_calibration_failed = true;
      }
    }
  }
#endif

  // Send read to SHT31
#ifdef ENABLE_SHT31
  DEBUG_PRINT("I2C to 0x44: 0x2400 (read SHT31 immediate)\n");
#ifdef ENABLE_SHT31_HEATER
  enableSht31Heater();
  delay(100);
  disableSht31Heater();
  delay(10);
#endif
  Wire.beginTransmission(0x44);
  Wire.write(0x24);
  Wire.write(0x16);
  Wire.endTransmission();
  Wire.requestFrom(0x44, 6);
#else
  DEBUG_PRINT("Skipping SHT31 read\n");
#endif

  // Read sensor status
#ifdef ENABLE_CO2_SENSOR
#if defined(ENABLE_NETWORK)
  if (ignore_s8_iteration_count > 0) {
    DEBUG_PRINT("Skipping CO2 sensor read (calibration cooldown)\n");
  } else if (s8_calibration_failed) {
    DEBUG_PRINT("Skipping CO2 sensor read (calibration failed)\n");
    co2SensorStatus = 65535;
    co2SensorPpm = 65535;
  } else
#endif
  {
    DEBUG_PRINT("Reading from CO2 sensor\n");
    co2Sensor.listen();
    co2Sensor.write(0xFE);
    co2Sensor.write(0x04);
    co2Sensor.write(0x00);
    co2Sensor.write(0x00);
    co2Sensor.write(0x00);
    co2Sensor.write(0x04);
    co2Sensor.write(0xE5);
    co2Sensor.write(0xC6);
    delay(100);

    // There's a bug in co2Sensor.available(), it won't return the right number of bytes the first time it's called :-/
    if(co2Sensor.available() && co2Sensor.available() >= 13) {
      uint8_t sensorPacket[13];
      for(int i = 0; i < 13; ++i) {
        sensorPacket[i] = co2Sensor.read();
      }
      // Modbus CRC is little-endian: low byte first
      uint16_t receivedCrc = sensorPacket[11] | (sensorPacket[12] << 8);
      uint16_t computedCrc = modbusCrc16(sensorPacket, 11);
      if (sensorPacket[0] == 0xFE && sensorPacket[1] == 0x04 && sensorPacket[2] == 0x08 &&
          receivedCrc == computedCrc) {
        uint16_t sensorStatus = (sensorPacket[3] << 8) | sensorPacket[4];
        uint16_t co2Ppm = (sensorPacket[9] << 8) | sensorPacket[10];
        DEBUG_PRINT("CO2 sensor status: ");
        DEBUG_PRINT_2(sensorStatus, HEX);
        DEBUG_PRINT("\nCO2 ppm: ");
        DEBUG_PRINT_2(co2Ppm, DEC);
        DEBUG_PRINT("\n");

        co2SensorStatus = sensorStatus;
        co2SensorPpm = co2Ppm;
      } else {
        DEBUG_PRINT("Corrupted CO2 sensor frame\n");

        co2SensorStatus = -1;
        co2SensorPpm = -1;
      }
    } else {
      DEBUG_PRINT("Less than 13 bytes available: ");
      DEBUG_PRINT_2(co2Sensor.available(), HEX);
      DEBUG_PRINT("\n");
    }
  }
#else
  DEBUG_PRINT("Skipping CO2 sensor read\n");
#endif

#ifdef ENABLE_PARTICLE_SENSOR
  // Read from particle counter
  DEBUG_PRINT("Reading from particle counter\n");
  particleCounter.listen();
  writeParticleCounterCommand(0xE2, 0x00, 0x00); // 0xE2 passive mode read
  delay(100);

  // At least one whole frame?
  bool headerOk = true;
  if(particleCounter.available() && particleCounter.available() >= 32) {
    // Skip until header 0x42 0x4D 0x00 0x1C (28)
    while(particleCounter.available() && particleCounter.read() != 0x42);
    if (particleCounter.peek() != 0x4D) {
      resyncStream();
      headerOk = false;
    } else {
      particleCounter.read();
      if (particleCounter.peek() != 0x00) {
        resyncStream();
        headerOk = false;
      }
      particleCounter.read();
      if (particleCounter.peek() != 0x1C) {
        resyncStream();
        headerOk = false;
      }
      particleCounter.read();
    }

    if (headerOk) {
      uint16_t data[14];
      for(int i = 0; i < 14; ++i) {
        uint16_t msb = particleCounter.read();
        uint16_t lsb = particleCounter.read();
        data[i] = (msb << 8) | lsb;
        /* DEBUG_PRINT("Data ");
        DEBUG_PRINT(i);
        DEBUG_PRINT(": ");
        DEBUG_PRINT_2(data[i] & 0xFFFF, HEX);
        DEBUG_PRINT("\n"); */
      }

      DEBUG_PRINT("PM1.0 ug/m^3: ");
      DEBUG_PRINT(data[3]);
      DEBUG_PRINT("\n");
      DEBUG_PRINT("PM2.5 ug/m^3: ");
      DEBUG_PRINT(data[4]);
      DEBUG_PRINT("\n");
      DEBUG_PRINT("PM10.0 ug/m^3: ");
      DEBUG_PRINT(data[5]);
      DEBUG_PRINT("\n");

      DEBUG_PRINT("Particle count per 0.1L air\n");
      DEBUG_PRINT(" > 0.3 um: ");
      DEBUG_PRINT(data[6]);
      DEBUG_PRINT("\n");
      DEBUG_PRINT(" > 0.5 um: ");
      DEBUG_PRINT(data[7]);
      DEBUG_PRINT("\n");
      DEBUG_PRINT(" > 1.0 um: ");
      DEBUG_PRINT(data[8]);
      DEBUG_PRINT("\n");
      DEBUG_PRINT(" > 2.5 um: ");
      DEBUG_PRINT(data[9]);
      DEBUG_PRINT("\n");
      DEBUG_PRINT(" > 5.0 um: ");
      DEBUG_PRINT(data[10]);
      DEBUG_PRINT("\n");
      DEBUG_PRINT("> 10.0 um: ");
      DEBUG_PRINT(data[11]);
      DEBUG_PRINT("\n");
                  
      uint16_t checksum = 0x42 + 0x4D + 0x00 + 0x1C;
      for(int i = 0; i < 13; ++i) {
        checksum += data[i] >> 8;
        checksum += data[i] & 0xFF;
      }
      if ((checksum & 0xFFFF) == data[13]) {
        DEBUG_PRINT("Checksum ok\n");
        particleSensorChecksumOk = 1;
      } else {
        DEBUG_PRINT("Checksum fail\n");
        particleSensorChecksumOk = 0;
      }

      uint8_t sensorErrorCode = data[12] & 0xFF;
      if (sensorErrorCode != 0) {
        DEBUG_PRINT("Particle sensor error code: ");
        DEBUG_PRINT_2(sensorErrorCode, HEX);
        DEBUG_PRINT("\n");
        particleSensorChecksumOk = 0;
      }

      particleSensorPm1_0 = data[3];
      particleSensorPm2_5 = data[4];
      particleSensorPm10_0 = data[5];
      particleSensorParticle0_3um = data[6];
      particleSensorParticle0_5um = data[7];
      particleSensorParticle1_0um = data[8];
      particleSensorParticle2_5um = data[9];
      particleSensorParticle5_0um = data[10];
      particleSensorParticle10_0um = data[11];
      
    } else {
      DEBUG_PRINT("Failed to read header\n");
      
      particleSensorChecksumOk = -1;
      particleSensorPm1_0 = -1;
      particleSensorPm2_5 = -1;
      particleSensorPm10_0 = -1;
      particleSensorParticle0_3um = -1;
      particleSensorParticle0_5um = -1;
      particleSensorParticle1_0um = -1;
      particleSensorParticle2_5um = -1;
      particleSensorParticle5_0um = -1;
      particleSensorParticle10_0um = -1;
    }
  } else {
    DEBUG_PRINT("No particle sensor data available\n");
  }
#else
  DEBUG_PRINT("Skipping particle sensor read\n");
#endif

#ifdef ENABLE_SHT31
  // Read from SHT31
  if(Wire.available() == 6) {
    uint8_t tempMsb = Wire.read();
    uint8_t tempLsb = Wire.read();
    uint8_t tempCrc = Wire.read();
    uint8_t humMsb = Wire.read();
    uint8_t humLsb = Wire.read();
    uint8_t humCrc = Wire.read();
    
    uint16_t tempRaw = (tempMsb << 8) | tempLsb;
    uint16_t humRaw = (humMsb << 8) | humLsb;

    float humidity = (100.0f * humRaw) / 65535;

    DEBUG_PRINT("Humidity : ");
    DEBUG_PRINT(humidity);
    DEBUG_PRINT(" ");
    DEBUG_PRINT_2(humMsb & 0xFF, HEX);
    DEBUG_PRINT_2(humLsb & 0xFF, HEX);    
    DEBUG_PRINT("\n");

    float temperature = ((175.0f * tempRaw) / 65535) - 45 + temperatureOffset;

    DEBUG_PRINT("Temperature : ");
    DEBUG_PRINT(temperature);
    DEBUG_PRINT(" ");
    DEBUG_PRINT_2(tempMsb & 0xFF, HEX);
    DEBUG_PRINT_2(tempLsb & 0xFF, HEX);
    DEBUG_PRINT("\n");

    sht31Temperature = temperature;
    sht31Humidity = humidity;
    sht31HumidityChecksumOk = (humCrc == sensirionCrc8(humMsb, humLsb)) ? 1 : 0;
    sht31TemperatureChecksumOk = (tempCrc == sensirionCrc8(tempMsb, tempLsb)) ? 1 : 0;
  } else {
    DEBUG_PRINT("Unexpected number of bytes:");
    DEBUG_PRINT(Wire.available());
    DEBUG_PRINT("\n");
    while(Wire.available()) { Wire.read(); }

    sht31Temperature = -1;
    sht31Humidity = -1;
  }
#endif

#ifdef ENABLE_SGP30

#ifdef ENABLE_SHT31
  // If we have SHT31 we can set the humidity compensation on the SGP30
  // SGP30 expects the humidity in g/m^3
  // SHT31 gives the humidity as RH%
  // Calculations from https://www.cactus2000.de/js/calchum.pdf

  // Quick sanity check
  if (10.0f < sht31Temperature && sht31Temperature < 50.0f &&
      1.0f < sht31Humidity && sht31Humidity < 100.0f &&
      sht31HumidityChecksumOk == 1 && sht31TemperatureChecksumOk == 1)
  {
    // Since we don't have an air pressure sensor, just assume a default of 101.325 kPa.
    const float airPressureHpa = 1013.25f;

    // Gas constant
    const float r = 8.31447215f;

    // Molar mass of water
    const float mH2O = 18.01534f;

    // From Lowe, P.R. and J.M. Ficke, 1974: The computation of saturation vapor pressure. Tech.
    // Paper No. 4-74, Environmental Prediction Research Facility, Naval Postgraduate School,
    // Monterey, CA, 27 pp
    const float a0w = 6.107799961f;
    const float a1w = 4.436518521f * 1e-1f;
    const float a2w = 1.428945805f * 1e-2f;
    const float a3w = 2.650648471f * 1e-4f;
    const float a4w = 3.031240396f * 1e-6f;
    const float a5w = 2.034080948f * 1e-8f;
    const float a6w = 6.136820929f * 1e-11f;
    const float t = sht31Temperature;

    // Water vapor pressure at current temperature
    float e = a0w + t * (a1w + t * (a2w + t * (a3w + t * (a4w + t * (a5w + t * a6w)))));

    // Air density
    float nAir = airPressureHpa * 100.0f / (r * (t + 273.15f));

    // Partial water pressure
    float pH2O = sht31Humidity * e / 100.0f;

    // Volume mixing ratio
    float xH2O = pH2O / airPressureHpa;

    // Mass concentration (g/m^3)
    float cH2O = xH2O * nAir * mH2O;
    DEBUG_PRINT("Mass concentration: ");
    DEBUG_PRINT(cH2O);
    DEBUG_PRINT("\n");

    // Calculate binary format
    uint8_t humidityMsb = min(floor(cH2O), 255.0f);
    uint8_t humidityLsb = (cH2O - floor(cH2O)) * 255.0f;
    DEBUG_PRINT("Humidity : ");
    DEBUG_PRINT_2(humidityMsb, DEC);
    DEBUG_PRINT(" ");
    DEBUG_PRINT_2(humidityLsb, DEC);
    DEBUG_PRINT("\n");

    Wire.beginTransmission(0x58);
    // Set_humidity: 0x2061 + humidity (msb, lsb) + crc
    Wire.write(0x20);
    Wire.write(0x61);
    Wire.write(humidityMsb);
    Wire.write(humidityLsb);
    Wire.write(sensirionCrc8(humidityMsb, humidityLsb));
    Wire.endTransmission();

    delay(25);
  }
#endif

  // Measure_air_quality: 0x2008
  Wire.beginTransmission(0x58);
  Wire.write(0x20);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom(0x58, 6);
  if(Wire.available() == 6) {
    uint8_t co2eqMsb = Wire.read();
    uint8_t co2eqLsb = Wire.read();
    uint8_t co2eqCrc = Wire.read();
    uint8_t tvocMsb = Wire.read();
    uint8_t tvocLsb = Wire.read();
    uint8_t tvocCrc = Wire.read();

    uint16_t co2eqPpm = (co2eqMsb << 8) | co2eqLsb;
    uint16_t tvocPpb = (tvocMsb << 8) | tvocLsb;

    sgp30co2eqPpm = co2eqPpm;
    sgp30tvocPpb = tvocPpb;
    sgp30co2eqChecksumOk = (co2eqCrc == sensirionCrc8(co2eqMsb, co2eqLsb)) ? 1 : 0;    
    sgp30tvocChecksumOk = (tvocCrc == sensirionCrc8(tvocMsb, tvocLsb)) ? 1 : 0;    

    DEBUG_PRINT("CO2eq ppm: ");
    DEBUG_PRINT(co2eqPpm);
    DEBUG_PRINT(" checksumOk? ");
    DEBUG_PRINT(sgp30co2eqChecksumOk);
    DEBUG_PRINT("\n");

    DEBUG_PRINT("tVOC ppb: ");
    DEBUG_PRINT(tvocPpb);
    DEBUG_PRINT(" checksumOk? ");
    DEBUG_PRINT(sgp30tvocChecksumOk);
    DEBUG_PRINT("\n");
  } else {
    DEBUG_PRINT("Unexpected number of bytes\n");
    while(Wire.available()) { Wire.read(); }
  }
#endif

#ifdef ENABLE_SERIAL_LOGGING
  if (serialLoggingEnabled) {
    // co2Status,co2Ppm,particleChecksumOk,particlePm1_0,particlePm2_5,particlePm10_0,particleCount0_3um,particleCount0_5um,particleCount1_0um,particleCount2_5um,particleCount5_0um,particleCount10_0um,temp,humidity,tempChecksumOk,humidityChecksumOk,co2eqPpm,tvocPpb,co2eqChecksumOk,tvocChecksumOk
    writeSingleLineStatusItem(co2SensorStatus);
    writeSingleLineStatusItem(co2SensorPpm);
    writeSingleLineStatusItem(particleSensorChecksumOk);
    writeSingleLineStatusItem(particleSensorPm1_0);
    writeSingleLineStatusItem(particleSensorPm2_5);
    writeSingleLineStatusItem(particleSensorPm10_0);
    writeSingleLineStatusItem(particleSensorParticle0_3um);
    writeSingleLineStatusItem(particleSensorParticle0_5um);
    writeSingleLineStatusItem(particleSensorParticle1_0um);
    writeSingleLineStatusItem(particleSensorParticle2_5um);
    writeSingleLineStatusItem(particleSensorParticle5_0um);
    writeSingleLineStatusItem(particleSensorParticle10_0um);
    writeSingleLineStatusItem(sht31Temperature);
    writeSingleLineStatusItem(sht31Humidity);
    writeSingleLineStatusItem(sht31TemperatureChecksumOk);
    writeSingleLineStatusItem(sht31HumidityChecksumOk);
    writeSingleLineStatusItem(sgp30co2eqPpm);
    writeSingleLineStatusItem(sgp30tvocPpb);
    writeSingleLineStatusItem(sgp30co2eqChecksumOk);
    writeSingleLineStatusItem(sgp30tvocChecksumOk, true);
  }
#endif

#ifdef ENABLE_NETWORK_LOGGING
  if (tcpLoggingEnabled) {
    if(asyncClient.connected()) {
      // macAddress,millis,co2Status,co2Ppm,particleChecksumOk,particlePm1_0,particlePm2_5,particlePm10_0,particleCount0_3um,particleCount0_5um,particleCount1_0um,particleCount2_5um,particleCount5_0um,particleCount10_0um,temp,humidity,tempChecksumOk,humidityChecksumOk,co2eqPpm,tvocPpb,co2eqChecksumOk,tvocChecksumOk
      String message("");
      message.reserve(MESSAGE_MAX_LEN);
      appendSingleLineStatusItem(message, macAddress);
      appendSingleLineStatusItem(message, millis());
      appendSingleLineStatusItem(message, co2SensorStatus);
      appendSingleLineStatusItem(message, co2SensorPpm);
      appendSingleLineStatusItem(message, particleSensorChecksumOk);
      appendSingleLineStatusItem(message, particleSensorPm1_0);
      appendSingleLineStatusItem(message, particleSensorPm2_5);
      appendSingleLineStatusItem(message, particleSensorPm10_0);
      appendSingleLineStatusItem(message, particleSensorParticle0_3um);
      appendSingleLineStatusItem(message, particleSensorParticle0_5um);
      appendSingleLineStatusItem(message, particleSensorParticle1_0um);
      appendSingleLineStatusItem(message, particleSensorParticle2_5um);
      appendSingleLineStatusItem(message, particleSensorParticle5_0um);
      appendSingleLineStatusItem(message, particleSensorParticle10_0um);
      appendSingleLineStatusItem(message, sht31Temperature);
      appendSingleLineStatusItem(message, sht31Humidity);
      appendSingleLineStatusItem(message, sht31TemperatureChecksumOk);
      appendSingleLineStatusItem(message, sht31HumidityChecksumOk);
      appendSingleLineStatusItem(message, sgp30co2eqPpm);
      appendSingleLineStatusItem(message, sgp30tvocPpb);
      appendSingleLineStatusItem(message, sgp30co2eqChecksumOk);
      appendSingleLineStatusItem(message, sgp30tvocChecksumOk, true);
      strncpy(messageBuffer, message.c_str(), MESSAGE_MAX_LEN);
      asyncClient.write(messageBuffer);
    } else if(!asyncClient.connecting()) {
      DEBUG_PRINT("Connecting to server\n");
      asyncClient.connect(serverHostname, serverPort);
    } else {
      DEBUG_PRINT("Awaiting connection\n");
    }
  }
#endif

  delay(900);
  DEBUG_PRINT("==========\n");

#ifdef ENABLE_NETWORK
  arduinoOTA.handle();
  httpServer.handleClient();
#endif
}