#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_LTR390.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <ESP32Servo.h>

// WiFi and MQTT credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "[REDACTED].s1.eu.hivemq.cloud";

unsigned long pumpStartTime = 0;
unsigned long lastPumpEndTime = 0;
bool pumpTimerRunning = false;

const unsigned long MAX_PUMP_DURATION = 7500;        // 7.5 seconds
const unsigned long MIN_PUMP_INTERVAL = 60000 * 10;  // 60000ms = 1 minute

bool doorsOpen;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// Define GPIO pins
#define FAN_PIN 2     // GPIO2 for fan
#define HEATER_PIN 4  // GPIO4 for heater
#define PUMP_PIN 5    // GPIO5 for pump
#define SERVO_PIN 23  // GPIO23 for servo

// Moisture sensor pin and calibration
#define MOISTURE_PIN 34
#define SENSOR_ADDR 0x76
const int AIR_VALUE = 4095;
const int WATER_VALUE = 1000;

// Relay logic (use LOW for active-low relays)
#define RELAY_ON LOW
#define RELAY_OFF HIGH

// Default optimal values
double OptimalTemperature = 16.0;
double OptimalAirHumidity = 60.0;
double OptimalGroundMoisture = 65.0;

// Auto mode
bool autoMode = true;

const char* filename = "/config.json";

// Sensors
Adafruit_BME280 bme;

// Second I2C bus for LTR390 (I2C port 1)
TwoWire ltrWire = TwoWire(1);
Adafruit_LTR390 ltr390;

bool hasHumidity = false;
bool hasLightSensor = false;

// Servo control
Servo myServo;
int currentServoAngle = 0;

// ✅ Update or create config file in LittleFS
bool updateConfig(const char* key, const char* value) {
  DynamicJsonDocument doc(1024);
  if (LittleFS.exists(filename)) {
    File file = LittleFS.open(filename, "r");
    if (!file) return false;
    DeserializationError err = deserializeJson(doc, file);
    file.close();
    if (err) doc.clear();
  }

  doc[key] = value;

  File file = LittleFS.open(filename, "w");
  if (!file) return false;
  serializeJsonPretty(doc, file);
  file.close();

  Serial.print("Updated config: ");
  Serial.print(key);
  Serial.print(" = ");
  Serial.println(value);
  return true;
}

String readConfig() {
  if (!LittleFS.exists(filename)) return "{}";

  File file = LittleFS.open(filename, "r");
  if (!file) return "{}";

  String content;
  while (file.available()) content += (char)file.read();
  file.close();
  return content;
}

void loadConfig() {
  String json = readConfig();
  if (json == "{}") {
    Serial.println("No config found, using defaults");
    return;
  }

  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print("Failed to parse config: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("optimal_temp")) OptimalTemperature = doc["optimal_temp"].as<double>();
  if (doc.containsKey("optimal_humidity")) OptimalAirHumidity = doc["optimal_humidity"].as<double>();
  if (doc.containsKey("optimal_moisture")) OptimalGroundMoisture = doc["optimal_moisture"].as<double>();

  Serial.print("Loaded config - Temp: ");
  Serial.print(OptimalTemperature);
  Serial.print("°C, Humidity: ");
  Serial.print(OptimalAirHumidity);
  Serial.print("%, Moisture: ");
  Serial.print(OptimalGroundMoisture);
  Serial.println("%");
}

void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("\nGetting time from Time Server...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  while (time(nullptr) < 100000) delay(100);

  Serial.println("Time synced!");
  espClient.setInsecure();  // Disable SSL certificate verification
}

void setup_sensors() {
  // Default I2C for BME280 (pins 21-SDA, 22-SCL)
  Wire.begin(21, 22);

  // Second I2C bus for LTR390 (pins 19-SDA, 18-SCL)
  ltrWire.begin(19, 18);

  // Initialize BME280
  if (!bme.begin(SENSOR_ADDR)) {
    Serial.println("Could not find BME280 sensor!");
    while (1) delay(10);
  }

  uint8_t chipID = bme.sensorID();
  if (chipID == 0x60) {
    Serial.println("BME280 detected (humidity supported)");
    hasHumidity = true;
  } else if (chipID == 0x58) {
    Serial.println("BMP280 detected (no humidity)");
  }

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X2,
                  Adafruit_BME280::SAMPLING_X16,
                  hasHumidity ? Adafruit_BME280::SAMPLING_X16 : Adafruit_BME280::SAMPLING_NONE,
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_500);

  // Initialize LTR390 on second I2C bus
  if (!ltr390.begin(&ltrWire)) {
    Serial.println("LTR390 not found");
  } else {
    Serial.println("LTR390 detected");
    ltr390.setMode(LTR390_MODE_UVS);
    ltr390.setGain(LTR390_GAIN_3);
    ltr390.setResolution(LTR390_RESOLUTION_18BIT);
    hasLightSensor = true;
  }
}

void rotateLeft() {
  currentServoAngle = max(0, currentServoAngle - 90);
  myServo.write(currentServoAngle);
  Serial.print("Rotated LEFT to: ");
  Serial.print(currentServoAngle);
  Serial.println("°");
  client.publish("sklenik/log", "servo::rotate_left");
}

void rotateRight() {
  currentServoAngle = min(360, currentServoAngle + 90);
  myServo.write(currentServoAngle);
  Serial.print("Rotated RIGHT to: ");
  Serial.print(currentServoAngle);
  Serial.println("°");
  client.publish("sklenik/log", "servo::rotate_right");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String payloadString;
  for (int i = 0; i < length; i++) payloadString += (char)payload[i];

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, payloadString);
  if (error) return;

  // Skip if it's sensor data (to avoid loops)
  if (doc.containsKey("temp") && doc.containsKey("pressure") && doc.containsKey("humidity") && doc.containsKey("moisture") && doc.containsKey("light")) return;
  Serial.println(payloadString);

  // factoryReset is currently unachievable because its not JSON so it will end before it gets here...
  if (payloadString == "factoryReset") {
    LittleFS.remove(filename);
    Serial.println("Factory Reset done!");
  }
  // Process settings updates
  if (doc.containsKey("type") && doc["type"] == "settings") {
    if (doc.containsKey("targetTemp")) {
      OptimalTemperature = doc["targetTemp"];
      char tempStr[10];
      dtostrf(OptimalTemperature, 1, 1, tempStr);
      updateConfig("optimal_temp", tempStr);
    }
    if (doc.containsKey("targetHumidity")) {
      OptimalAirHumidity = doc["targetHumidity"];
      char humidityStr[10];
      dtostrf(OptimalAirHumidity, 1, 1, humidityStr);
      updateConfig("optimal_humidity", humidityStr);
    }
    if (doc.containsKey("targetMoisture")) {
      OptimalGroundMoisture = doc["targetMoisture"];
      char moistureStr[10];
      dtostrf(OptimalGroundMoisture, 1, 1, moistureStr);
      updateConfig("optimal_moisture", moistureStr);
    }
    if (doc.containsKey("autoMode")) {
      autoMode = doc["autoMode"];
    }

    Serial.println("Settings updated:");
    Serial.println(payloadString);
  }
  if (doc.containsKey("servoLeft") && doc["servoLeft"] == true) {
    rotateLeft();
    return;
  }
  if (doc.containsKey("servoRight") && doc["servoRight"] == true) {
    rotateRight();
    return;
  }
  // Manual control (only when auto mode is off)
  if (doc.containsKey("type") && doc["type"] == "manual") {
    if (!autoMode) {
      Serial.println("Manual controls:");
      if (doc.containsKey("fan")) {
        setFanActive(doc["fan"]);
        Serial.print("Fan: ");
        Serial.println(doc["fan"] ? "ON" : "OFF");
      }
      if (doc.containsKey("waterPump")) {
        setPumpActive(doc["waterPump"]);
        Serial.print("Water Pump: ");
        Serial.println(doc["waterPump"] ? "ON" : "OFF");
      }
      if (doc.containsKey("heating")) {
        setHeaterActive(doc["heating"]);
        Serial.print("Heating: ");
        Serial.println(doc["heating"] ? "ON" : "OFF");
      }
      if (doc.containsKey("lightLevel")) {
        Serial.print("Light Level: ");
        Serial.print(doc["lightLevel"].as<int>());
        Serial.println("%");
        // TODO: Add light control if needed
      }
      // Servo control commands

    } else {
      Serial.println("Manual control ignored - Auto mode is ON");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", "sklenik", "Puntik2010")) {
      Serial.println("connected");
      client.subscribe("sklenik/sensor");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

// Temperature reading function
float readTemperature() {
  return bme.readTemperature();
}

// Pressure reading function
float readPressure() {
  return bme.readPressure() / 100.0;  // Convert to hPa
}

// Humidity reading function
float readHumidity() {
  return hasHumidity ? bme.readHumidity() : 0;
}

// Soil moisture raw reading
int readMoistureRaw() {
  return analogRead(MOISTURE_PIN);
}

// Soil moisture percentage
int readMoisturePercent() {
  int rawMoisture = readMoistureRaw();
  int moisturePercent = map(rawMoisture, AIR_VALUE, WATER_VALUE, 0, 100);
  return constrain(moisturePercent, 0, 100);
}

// Light reading function
uint32_t readLightValue() {
  return hasLightSensor ? ltr390.readUVS() : 0;
}

// Automatic control with priorities
void autoControl() {
  if (!autoMode) return;

  float temperature = readTemperature();
  float humidity = readHumidity();
  int moisturePercent = readMoisturePercent();
  unsigned long now = millis();

  // Priority 1: Pump (if active or meets conditions)
  if (isPumpActive() || (!pumpTimerRunning && moisturePercent < (OptimalGroundMoisture - 5) && now - lastPumpEndTime >= MIN_PUMP_INTERVAL)) {
    if (!isPumpActive()) {
      setPumpActive(true);
      pumpStartTime = now;
      pumpTimerRunning = true;
      Serial.println("AUTO: Moisture low - Pump ON for max 7.5s");
      client.publish("sklenik/log", "moisture_low::pump_on");
    } else if (now - pumpStartTime >= MAX_PUMP_DURATION) {
      setPumpActive(false);
      pumpTimerRunning = false;
      lastPumpEndTime = now;
      Serial.println("AUTO: Pump stopped after max 7.5s run time");
      client.publish("sklenik/log", "pump_stopped::timeout");
    }
    // If pump is running, turn off other devices
    if (isHeaterActive()) setHeaterActive(false);
    if (isFanActive()) setFanActive(false);
    return;  // Exit function since pump has highest priority
  }

  // Priority 2: Heater (if needed)
  if (temperature < (OptimalTemperature - 2)) {
    if (!isHeaterActive()) {
      setHeaterActive(true);
      // Turn off fan when heater starts
      if (isFanActive()) setFanActive(false);
    }
    // If heater is running, turn off pump (just in case)
    if (isPumpActive()) setPumpActive(false);
    return;  // Exit since heater has higher priority than fan
  }

  // Priority 3: Fan (if needed)
  if (temperature > (OptimalTemperature + 2)) {
    if (!isFanActive()) {
      setFanActive(true);
    }
    // If fan is running, turn off other devices
    if (isHeaterActive()) setHeaterActive(false);
    if (isPumpActive()) setPumpActive(false);
    return;
  }

  // Turn off all devices if not needed
  if (isHeaterActive() && temperature >= OptimalTemperature + 2) {
    setHeaterActive(false);
  }
  if (isFanActive() && temperature <= OptimalTemperature + 2) {
    setFanActive(false);
  }
  if (isPumpActive() && moisturePercent > (OptimalGroundMoisture + 5)) {
    setPumpActive(false);
    pumpTimerRunning = false;
    lastPumpEndTime = now;
    Serial.println("AUTO: Moisture high - Pump OFF");
    client.publish("sklenik/log", "moisture_high::pump_off");
  }
}

// Sensor data publishing
void publishSensorData() {
  int rawMoisture = readMoistureRaw();
  float moisturePercent = readMoisturePercent();
  float temperature = readTemperature();
  float pressure = readPressure();
  float humidity = readHumidity();
  uint32_t lightValue = readLightValue();

  DynamicJsonDocument doc(512);
  doc["temp"] = temperature;
  doc["pressure"] = pressure;
  doc["humidity"] = humidity;
  doc["moisture"] = moisturePercent;
  doc["light"] = lightValue;
  doc["autoMode"] = autoMode;
  doc["fanActive"] = isFanActive();
  doc["heaterActive"] = isHeaterActive();
  doc["pumpActive"] = isPumpActive();
  doc["servoAngle"] = currentServoAngle;  // Include current servo position

  String jsonStr;
  serializeJson(doc, jsonStr);

  Serial.println("Publishing: " + jsonStr);
  client.publish("sklenik/sensor", jsonStr.c_str());
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(2000);

  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("Failed to mount LittleFS");
    return;
  }

  // Load configuration
  loadConfig();

  // Set GPIO pins as outputs
  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  // Turn off all relays
  digitalWrite(FAN_PIN, RELAY_OFF);
  digitalWrite(HEATER_PIN, RELAY_OFF);
  digitalWrite(PUMP_PIN, RELAY_OFF);

  // Initialize servo
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(180);
  Serial.println("Servo initialized at 180");
  doorsOpen = false;

  Serial.println("ESP32 Relay Control System Initialized");
  Serial.println("Fan Pin: " + String(FAN_PIN));
  Serial.println("Heater Pin: " + String(HEATER_PIN));
  Serial.println("Pump Pin: " + String(PUMP_PIN));
  Serial.println("Servo Pin: " + String(SERVO_PIN));

  delay(1000);

  // Connect to WiFi
  setup_wifi();

  // Setup MQTT
  client.setServer(mqtt_server, 8883);
  client.setCallback(callback);

  // Setup sensors
  setup_sensors();

  Serial.println("All systems ready!");
  Serial.println("Auto mode: " + String(autoMode ? "ON" : "OFF"));
}

void closeDoors() {
  if (doorsOpen) {
    myServo.write(180);
    doorsOpen = !doorsOpen;
  }
  else {
    Serial.println("close - no");
  }
}

void openDoors() {
  if (!doorsOpen) {
    myServo.write(0);
    doorsOpen = !doorsOpen;
  }
  else {
    Serial.println("open - no");
  }
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    setup_wifi();
  }

  // Check MQTT connection
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Automatic control
  autoControl();

  // Publish sensor data
  publishSensorData();

  if (Serial.available() > 0) {
    // Read the incoming string until newline
    String input = Serial.readStringUntil('\n');
    // Trim any extra whitespace or carriage return (especially if sent from Windows)
    input.trim();
    // Now you can process the input string
    Serial.print("Echo: ");
    Serial.println(input);

    if (input == "right") {
      rotateRight();
    }
    else if (input == "left") {
      rotateLeft();
    }
  }

  delay(5000);  // Wait 5 seconds before next measurement
}

// Fan control function
void setFanActive(bool active) {
  if (active) {
    openDoors();
  }
  else {
    closeDoors();
  }
  digitalWrite(FAN_PIN, active ? RELAY_ON : RELAY_OFF);
  Serial.println("Fan: " + String(active ? "ON" : "OFF"));
  const char* state = active ? "ON" : "OFF";
  String msg = "fan::" + String(state);
  client.publish("sklenik/log", msg.c_str());
}

// Heater control function
void setHeaterActive(bool active) {
  digitalWrite(HEATER_PIN, active ? RELAY_ON : RELAY_OFF);
  Serial.println("Heater: " + String(active ? "ON" : "OFF"));
  const char* state = active ? "ON" : "OFF";
  String msg = "heater::" + String(state);
  client.publish("sklenik/log", msg.c_str());
}

// Pump control function
void setPumpActive(bool active) {
  /*digitalWrite(PUMP_PIN, active ? RELAY_ON : RELAY_OFF);
  Serial.println("Pump: " + String(active ? "ON" : "OFF"));
  const char* state = active ? "ON" : "OFF";
  String msg = "pump::" + String(state);
  client.publish("sklenik/log", msg.c_str());

  // Update pump timer
  if (active) {
    pumpStartTime = millis();
    pumpTimerRunning = true;
  } else if (pumpTimerRunning) {
    pumpTimerRunning = false;
    lastPumpEndTime = millis();
  }*/
  Serial.println("Temporarily disabled!");
}

// Device status functions
bool isFanActive() {
  return digitalRead(FAN_PIN) == RELAY_ON;
}

bool isHeaterActive() {
  return digitalRead(HEATER_PIN) == RELAY_ON;
}

bool isPumpActive() {
  return digitalRead(PUMP_PIN) == RELAY_ON;
}
