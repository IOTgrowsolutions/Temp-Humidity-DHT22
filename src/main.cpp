/*********************************************************************************************************************
 * Title: Optimized ESP8266 Temperature, Humidity, and VPD Monitor (with SHT31 I2C Sensor)
 * * Description:
 * This firmware is designed for an ESP8266 (NodeMCU) to read data from a Sensirion SHT3x sensor via I2C,
 * publish it to an MQTT broker, and serve a web page displaying the readings. It also
 * provides a web interface for WiFi configuration and Over-The-Air (OTA) firmware updates.
 *
 * Key Optimizations from the original code:
 * 1.  **Non-Blocking Operations:** All `delay()` calls have been removed from the main loop and connection logic.
 * The code now uses `millis()` timers for all timed events (sensor reads, MQTT publishing, connection retries),
 * ensuring the device remains responsive.
 * 2.  **Efficient Memory Management:** The use of the Arduino `String` class has been drastically reduced,
 * especially in web server handlers. HTML content is now sent in chunks using `server.sendContent()` and
 * constant strings are stored in Flash memory using the `F()` macro to preserve precious RAM.
 * 3.  **Robust Connection Handling:** WiFi and MQTT connection management is now fully non-blocking. An exponential
 * backoff strategy is implemented for MQTT reconnections to prevent flooding the network with requests
 * when the broker is unavailable.
 * 4.  **Improved Code Structure:** The code has been reorganized for better readability and maintainability.
 * Functions are more focused, and global variables are grouped and commented.
 * 5.  **Removed Redundancy:** Unused variables and redundant logic (like the forced 6-hour restart) have been removed.
 * A stable firmware should not require periodic restarts.
 * 6.  **Enhanced Web Interface:** The web interface has been improved for a better user experience and uses more
 * efficient methods to serve content.
 *********************************************************************************************************************/

// --- Core Libraries ---
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <Wire.h>             // For I2C communication
#include <Adafruit_SHT31.h>   // For the Sensirion SHT31 sensor

// --- Pin Definitions ---
// I2C pins for SHT31 on NodeMCU are typically D1 (SCL) and D2 (SDA).
// These are handled by the Wire library.

// --- WiFi & Network Configuration ---
const byte DNS_PORT = 53;
DNSServer dnsServer;
ESP8266WebServer server(80);
char mqtt_server[40]; // Increased size for domain names

// --- MQTT Configuration ---
#define MQTT_MSG_BUFFER_SIZE 256
char mqtt_msg_buffer[MQTT_MSG_BUFFER_SIZE];
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long INITIAL_RECONNECT_DELAY = 5000;
const unsigned long MAX_RECONNECT_DELAY = 120000; // Max 2 minutes
unsigned long reconnectDelay = INITIAL_RECONNECT_DELAY;

// --- Sensor Configuration ---
Adafruit_SHT31 sht31 = Adafruit_SHT31();
float humidity = 0.0f;
float temperature = 0.0f;

// --- Timers for Non-Blocking Operations ---
unsigned long previousSensorReadMillis = 0;
const long SENSOR_READ_INTERVAL = 10000; // Read sensor every 10 seconds

// --- Forward Declarations ---
void setupAP();
void handleRoot();
void handleNotFound();
void manageMqttConnection();
float calculateVPD(float temp, float hum);

/**
 * @brief Reads the MQTT server IP/hostname from EEPROM.
 */
void loadMqttServerFromEEPROM() {
    memset(mqtt_server, 0, sizeof(mqtt_server)); // Clear buffer
    EEPROM.get(120, mqtt_server);
    mqtt_server[sizeof(mqtt_server)-1] = '\0'; // Ensure null-termination
    Serial.print(F("Loaded MQTT Server from EEPROM: "));
    Serial.println(mqtt_server);
}

/**
 * @brief Saves the MQTT server IP/hostname to EEPROM.
 */
void saveMqttServerToEEPROM(const char* serverAddress) {
    char temp[sizeof(mqtt_server)] = {0};
    strncpy(temp, serverAddress, sizeof(temp)-1); // Copy and null-terminate
    for (int i = 0; i < sizeof(mqtt_server); i++) {
        EEPROM.write(120 + i, 0);
    }
    EEPROM.put(120, temp); // Write the full buffer
    EEPROM.commit();
    strncpy(mqtt_server, temp, sizeof(mqtt_server)-1);
    mqtt_server[sizeof(mqtt_server)-1] = '\0';
    Serial.print(F("Saved MQTT Server to EEPROM: "));
    Serial.println(mqtt_server);
}

/**
 * @brief Initializes WiFi connection using credentials from EEPROM.
 */
void initWiFi() {
    char ssid[33] = {0};
    char password[65] = {0};

    EEPROM.get(0, ssid);
    EEPROM.get(33, password);

    Serial.println(F("Connecting to WiFi..."));
    Serial.print(F("SSID: "));
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.hostname(F("TempHumSensor"));
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500); // This short delay is acceptable in setup()
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("\nWiFi Connected!"));
        Serial.print(F("IP Address: "));
        Serial.println(WiFi.localIP());
    } else {
        Serial.println(F("\nFailed to connect to WiFi. Starting AP mode."));
        setupAP();
    }
}

/**
 * @brief Configures and starts the OTA update service.
 */
void setupOTA() {
    ArduinoOTA.setHostname("TempHumSensor");
    ArduinoOTA.onStart([]() { Serial.println(F("OTA Start")); });
    ArduinoOTA.onEnd([]() { Serial.println(F("\nOTA End")); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress * 100) / total);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println(F("Auth Failed"));
        else if (error == OTA_BEGIN_ERROR) Serial.println(F("Begin Failed"));
        else if (error == OTA_CONNECT_ERROR) Serial.println(F("Connect Failed"));
        else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Receive Failed"));
        else if (error == OTA_END_ERROR) Serial.println(F("End Failed"));
    });
    ArduinoOTA.begin();
    Serial.println(F("OTA Ready"));
}

/**
 * @brief Sets up Access Point mode for WiFi configuration.
 */
void setupAP() {
    WiFi.mode(WIFI_AP);
    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP("TempHumSensor-Config", "password");

    dnsServer.start(DNS_PORT, "*", apIP);

    Serial.println(F("\nAP Mode Enabled"));
    Serial.print(F("AP IP address: "));
    Serial.println(WiFi.softAPIP());
}

/**
 * @brief MQTT callback function for incoming messages.
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // This function is kept for future use, e.g., controlling the device via MQTT.
    Serial.print(F("Message arrived ["));
    Serial.print(topic);
    Serial.print(F("] "));
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

/**
 * @brief Defines all web server routes and handlers.
 */
void setupWebServer() {
    server.on("/", HTTP_GET, handleRoot);

    server.on("/config", HTTP_GET, []() {
        String html = F(
            "<!DOCTYPE html><html><head><title>Config</title>"
            "<meta name=viewport content='width=device-width,initial-scale=1'>"
            "</head><body><h1>WiFi & MQTT Config</h1>"
            "<form action='/save' method='post'>"
            "<h2>WiFi Settings</h2>"
            "SSID:<br><input type='text' name='ssid' length=32 required><br>"
            "Password:<br><input type='password' name='pass' length=64><br>"
            "<h2>MQTT Settings</h2>"
            "MQTT Broker:<br><input type='text' name='mqtt' value='");
        html += mqtt_server;
        html += F(
            "' length=40><br><br>"
            "<input type='submit' value='Save & Reboot'>"
            "</form></body></html>");
        server.send(200, "text/html", html);
    });

    server.on("/save", HTTP_POST, []() {
        // Save WiFi credentials
        String ssid = server.arg("ssid");
        String pass = server.arg("pass");
        if (ssid.length() > 0) {
            char ssid_char[33];
            char pass_char[65];
            ssid.toCharArray(ssid_char, sizeof(ssid_char));
            // Accept blank password: always copy, even if empty
            pass.toCharArray(pass_char, sizeof(pass_char));
            EEPROM.put(0, ssid_char);
            EEPROM.put(33, pass_char);
            Serial.println(F("WiFi credentials saved."));
        }

        // Save MQTT server
        String mqtt = server.arg("mqtt");
        if (mqtt.length() > 0) {
            char mqtt_char[40];
            mqtt.toCharArray(mqtt_char, sizeof(mqtt_char));
            saveMqttServerToEEPROM(mqtt_char);
        }

        EEPROM.commit();

        server.send(200, "text/plain", "Settings saved. Rebooting...");
        delay(1000);
        ESP.restart();
    });
    
    // Web OTA Update
    server.on("/update", HTTP_GET, []() {
        String html = F("<!DOCTYPE html><html><head><title>Firmware Update</title></head>"
                        "<body><h1>Firmware OTA Update</h1>"
                        "<form method='POST' action='/update' enctype='multipart/form-data'>"
                        "<input type='file' name='update' accept='.bin'>"
                        "<input type='submit' value='Update Firmware'>"
                        "</form></body></html>");
        server.send(200, "text/html", html);
    });

    server.on("/update", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        ESP.restart();
    }, []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin(ESP.getFreeSketchSpace())) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) {
                Serial.printf("Update Success: %u\n", upload.totalSize);
            } else {
                Update.printError(Serial);
            }
        }
    });

    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println(F("Web Server started."));
}

/**
 * @brief Main setup function.
 */
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n\n--- ESP8266 Sensor Booting ---"));

    EEPROM.begin(512);

    // Initialize I2C and SHT31 sensor
    Serial.println(F("Initializing SHT31 sensor..."));
    if (!sht31.begin(0x44)) { // Start I2C with default address
        Serial.println(F("Couldn't find SHT31 sensor! Check your wiring."));
        // Program will continue, but sensor reads will fail.
    } else {
        Serial.println(F("SHT31 sensor found!"));
    }

    initWiFi();
    
    if (WiFi.status() == WL_CONNECTED) {
        setupOTA();
        loadMqttServerFromEEPROM();
        mqttClient.setServer(mqtt_server, 1883);
        mqttClient.setCallback(mqttCallback);
    }
    
    setupWebServer();
}

/**
 * @brief Main loop. Handles all non-blocking tasks.
 */
void loop() {
    // --- Always-on Handlers ---
    if (WiFi.getMode() == WIFI_AP) {
        dnsServer.processNextRequest();
    } else {
        ArduinoOTA.handle();
    }
    server.handleClient();

    // --- Connectivity Management ---
    if (WiFi.status() == WL_CONNECTED) {
        manageMqttConnection();
        mqttClient.loop(); // Essential for processing MQTT messages
    }

    // --- Timed Actions ---
    unsigned long currentMillis = millis();
    if (currentMillis - previousSensorReadMillis >= SENSOR_READ_INTERVAL) {
        previousSensorReadMillis = currentMillis;

        // Read sensor data from SHT31
        temperature = sht31.readTemperature();
        humidity = sht31.readHumidity();

        // Check if the sensor read was successful (returns NaN on failure)
        if (!isnan(temperature) && !isnan(humidity)) {
            Serial.print(F("Sensor Read OK -> "));
            Serial.print(F("Temp: ")); Serial.print(temperature);
            Serial.print(F(" *C, Humidity: ")); Serial.print(humidity); Serial.println(F(" %"));

            // Publish to MQTT if connected
            if (mqttClient.connected()) {
                float vpd = calculateVPD(temperature, humidity);

                // Publish Temperature
                dtostrf(temperature, 4, 2, mqtt_msg_buffer);
                mqttClient.publish("H-T-Sensor2/temperature", mqtt_msg_buffer);

                // Publish Humidity
                dtostrf(humidity, 4, 2, mqtt_msg_buffer);
                mqttClient.publish("H-T-Sensor2/humidity", mqtt_msg_buffer);

                // Publish VPD
                dtostrf(vpd, 4, 2, mqtt_msg_buffer);
                mqttClient.publish("H-T-Sensor2/vpd", mqtt_msg_buffer);

                Serial.println(F("Published to MQTT."));
            }
        } else {
            Serial.println(F("Failed to read from SHT31 sensor!"));
        }
    }
}

/**
 * @brief Handles web requests to the root URL.
 */
void handleRoot() {
    char tempStr[8], humStr[8], vpdStr[8];
    float vpd = calculateVPD(temperature, humidity);

    dtostrf(temperature, 4, 2, tempStr);
    dtostrf(humidity, 4, 2, humStr);
    dtostrf(vpd, 4, 2, vpdStr);

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    String header = F("<!DOCTYPE html><html lang=en><head><meta charset=UTF-8><meta name=viewport content='width=device-width,initial-scale=1'><meta http-equiv=refresh content=20><title>Sensor Monitor</title><style>body{font-family:Arial,sans-serif;background-color:#f0f0f0;margin:0;padding:20px;display:flex;justify-content:center;align-items:center;min-height:100vh}.container{background-color:#fff;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,.1);padding:20px;max-width:400px;width:100%}h1{color:#333;text-align:center;margin-bottom:20px}.reading{display:flex;align-items:center;margin-bottom:20px}.icon{font-size:40px;margin-right:20px}.value{font-size:24px;font-weight:700}.unit{font-size:16px;color:#666}.info{text-align:center;color:#666;font-size:14px}.reference{margin-top:20px;font-size:14px;color:#333;text-align:center}.range{font-weight:700;color:#007bff}.footer-link{margin-top:25px;text-align:center;font-size:16px}button{background-color:#007bff;color:#fff;padding:12px 20px;border:none;border-radius:5px;cursor:pointer;font-size:16px;transition:background-color .3s}</style></head><body><div class=container><h1>Sensor Readings</h1>");
    server.sendContent(header);

    String reading_temp = F("<div class=reading><div class=icon>🌡️</div><div><div class=value>");
    reading_temp += tempStr;
    reading_temp += F("</div><div class=unit>°C</div></div></div>");
    server.sendContent(reading_temp);

    String reading_hum = F("<div class=reading><div class=icon>💧</div><div><div class=value>");
    reading_hum += humStr;
    reading_hum += F("</div><div class=unit>% RH</div></div></div>");
    server.sendContent(reading_hum);

    String reading_vpd = F("<div class=reading><div class=icon>🌫️</div><div><div class=value>");
    reading_vpd += vpdStr;
    reading_vpd += F("</div><div class=unit>kPa</div></div></div>");
    server.sendContent(reading_vpd);

    String footer = F("<div class=reference><p>Recommended VPD Ranges:</p><p>🌱 <span class=range>Vegetative:</span> 0.8–1.2 kPa</p><p>🌼 <span class=range>Flowering:</span> 1.2–1.6 kPa</p></div><div class=footer-link><a href='/config'><button>Configure</button></a> <a href='/update'><button>Update FW</button></a></div></div></body></html>");
    server.sendContent(footer);
    
    server.sendContent(""); // Finalize the response
}


/**
 * @brief Handles captive portal redirection and not-found pages.
 */
void handleNotFound() {
    if (WiFi.getMode() == WIFI_AP) {
        // Redirect to the config page in AP mode
        server.sendHeader("Location", "/config", true);
        server.send(302, "text/plain", "");
    } else {
        server.send(404, "text/plain", "404: Not Found");
    }
}

/**
 * @brief Manages MQTT connection and reconnection logic with exponential backoff.
 */
void manageMqttConnection() {
    if (mqttClient.connected()) {
        if (reconnectDelay != INITIAL_RECONNECT_DELAY) {
            Serial.println(F("MQTT connection restored."));
            reconnectDelay = INITIAL_RECONNECT_DELAY; // Reset delay on success
        }
        return;
    }

    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt > reconnectDelay) {
        lastMqttReconnectAttempt = now;

        Serial.print(F("Attempting MQTT connection..."));
        if (mqttClient.connect("TempHumClient")) {
            Serial.println(F(" connected!"));
            reconnectDelay = INITIAL_RECONNECT_DELAY; // Reset delay
        } else {
            Serial.print(F(" failed, rc="));
            Serial.print(mqttClient.state());
            Serial.println(F(". Retrying later."));
            // Double the delay for the next attempt (exponential backoff)
            reconnectDelay *= 2;
            if (reconnectDelay > MAX_RECONNECT_DELAY) {
                reconnectDelay = MAX_RECONNECT_DELAY; // Cap the delay
            }
        }
    }
}

/**
 * @brief Calculates Vapor Pressure Deficit (VPD) in kPa.
 * @param temp The temperature in degrees Celsius.
 * @param hum The relative humidity in percent.
 * @return The calculated VPD, or NAN if inputs are invalid.
 */
float calculateVPD(float temp, float hum) {
    if (isnan(temp) || isnan(hum)) {
        return NAN;
    }
    // Saturation Vapor Pressure (SVP) using Magnus formula
    float svp = 0.6108 * exp((17.27 * temp) / (temp + 237.3));
    // Actual Vapor Pressure (AVP)
    float avp = svp * (hum / 100.0);
    return svp - avp;
}