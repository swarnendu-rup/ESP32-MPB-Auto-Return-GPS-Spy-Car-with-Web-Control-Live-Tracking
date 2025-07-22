/*🔌 Wiring & Connection Steps
1. ESP32 to L298N Motor Driver (Dual DC Motors):
ENA → GPIO 25 (PWM for left motor)

IN1 → GPIO 26

IN2 → GPIO 27

IN3 → GPIO 14

IN4 → GPIO 12

ENB → GPIO 33 (PWM for right motor)

L298N GND → ESP32 GND

L298N VCC (12V) → External Power Supply (not ESP32)

L298N 5V → Keep jumper if needed for logic power

2. GPS Module (NEO-6M) to ESP32:
TX (GPS) → GPIO 16 (RX of ESP32 UART2)

RX (GPS) → GPIO 17 (TX of ESP32 UART2)

VCC → 3.3V

GND → GND

3. Compass (HMC5883L) to ESP32 (I2C):
SCL → GPIO 22

SDA → GPIO 21

VCC → 3.3V

GND → GND

4. Web Interface:
ESP32 creates a Wi-Fi hotspot (MySpyCar)

Connect your phone/PC → Go to 192.168.4.1

Use WebSocket UI to control: "Explore", "Return", "Stop"*/

// --- Core Libraries ---
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <ArduinoJson.h>
#include <vector>

// --- Sensor Libraries ---
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// --- WiFi Credentials ---
const char* ssid = "MySpyCar"; // Your WiFi SSID
const char* password = "123456789"; // Your WiFi Password

// --- GPS Configuration ---
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(2);

// --- Motor Driver (L298N) Pin Configuration ---
#define ENA 25 // Left motor speed
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENB 33 // Right motor speed
const int motorSpeed = 150; // Speed from 0-255

// --- Compass Sensor ---
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// --- Navigation & State Management ---
enum BotState { IDLE, EXPLORING, BACKTRACKING, EMERGENCY_STOP };
BotState currentState = IDLE;

struct GPSCoordinate {
  double lat;
  double lon;
};

std::vector<GPSCoordinate> loggedPath;
int backtrackingWaypointIndex = -1;
const double WAYPOINT_RADIUS_METERS = 2.0; // How close to get to a waypoint
const double LOG_DISTANCE_METERS = 3.0;   // How far to travel before logging a new point

// --- Web Server & WebSocket ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =======================================================================
// MOTOR CONTROL FUNCTIONS
// =======================================================================
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnRight() {
  digitalWrite(IN1, HIGH); // Left wheels forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Right wheels backward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnLeft() {
  digitalWrite(IN1, LOW); // Left wheels backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); // Right wheels forward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// =======================================================================
// NAVIGATION LOGIC
// =======================================================================
void navigateToWaypoint() {
    if (backtrackingWaypointIndex < 0 || !gps.location.isValid()) {
        stopMotors();
        currentState = IDLE;
        return;
    }

    GPSCoordinate target = loggedPath[backtrackingWaypointIndex];
    double distanceToTarget = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), target.lat, target.lon);

    // Check if we have arrived at the waypoint
    if (distanceToTarget < WAYPOINT_RADIUS_METERS) {
        backtrackingWaypointIndex--; // Target the next point in the path
        if (backtrackingWaypointIndex < 0) { // We have arrived home
            currentState = IDLE;
            stopMotors();
            return;
        }
    }

    // --- Navigation Calculation ---
    double targetBearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), target.lat, target.lon);

    sensors_event_t event;
    mag.getEvent(&event);
    float currentHeading = atan2(event.magnetic.y, event.magnetic.x) * (180 / PI);
    if (currentHeading < 0) currentHeading += 360;

    double headingError = targetBearing - currentHeading;
    // Normalize the error to be between -180 and 180
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    // --- Motor Commands based on Heading Error ---
    if (abs(headingError) < 15) { // If we are pointing in the right direction
        moveForward();
    } else if (headingError > 0) {
        turnRight();
    } else {
        turnLeft();
    }
}


// =======================================================================
// WEBSOCKET AND DATA HANDLING
// =======================================================================
void broadcastData() {
    String jsonString;
    StaticJsonDocument<1024> doc; // Increased size to handle path data

    doc["lat"] = gps.location.lat();
    doc["lon"] = gps.location.lng();
    doc["sats"] = gps.satellites.value();
    doc["speed"] = gps.speed.kmph();
    doc["valid"] = gps.location.isValid();
    
    // Convert enum state to string
    switch(currentState) {
        case IDLE: doc["state"] = "IDLE"; break;
        case EXPLORING: doc["state"] = "EXPLORING"; break;
        case BACKTRACKING: doc["state"] = "BACKTRACKING"; break;
        case EMERGENCY_STOP: doc["state"] = "STOPPED"; break;
    }

    doc["points"] = loggedPath.size();

    // Send the full path only to a newly connected client
    if (loggedPath.size() > 0) {
        JsonArray path = doc.createNestedArray("path");
        for(const auto& p : loggedPath) {
            JsonObject point = path.createNestedObject();
            point["lat"] = p.lat;
            point["lon"] = p.lon;
        }
    }

    serializeJson(doc, jsonString);
    ws.textAll(jsonString);
}


void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("Client #%u connected\n", client->id());
        broadcastData(); // Send initial full data dump
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("Client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo info = (AwsFrameInfo)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            StaticJsonDocument<64> doc;
            if (deserializeJson(doc, (char*)data) == DeserializationError::Ok) {
                const char* command = doc["command"];
                if (strcmp(command, "explore") == 0) {
                    currentState = EXPLORING;
                } else if (strcmp(command, "return") == 0) {
                    if (!loggedPath.empty()) {
                        backtrackingWaypointIndex = loggedPath.size() - 1; // Start from the last point
                        currentState = BACKTRACKING;
                    }
                } else if (strcmp(command, "stop") == 0) {
                    currentState = EMERGENCY_STOP;
                }
            }
        }
    }
}

// =======================================================================
// SETUP AND LOOP
// =======================================================================

// Include the separate file for the HTML page
#include "webpage.h"

void setup() {
    Serial.begin(115200);
    ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    Wire.begin();

    // Initialize Motor Pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    stopMotors();

    // Initialize Compass
    if(!mag.begin()){
      Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
      while(1);
    }

    // Initialize WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());

    // Initialize Web Server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    ElegantOTA.begin(&server);
    server.begin();
}

unsigned long lastBroadcastTime = 0;

void loop() {
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }

    // Main State Machine Logic
    switch(currentState) {
        case EXPLORING:
            moveForward();
            if (gps.location.isValid()) {
                if (loggedPath.empty()) {
                    // Log the very first point (home)
                    loggedPath.push_back({gps.location.lat(), gps.location.lng()});
                } else {
                    double distFromLast = TinyGPSPlus::distanceBetween(
                        gps.location.lat(), gps.location.lng(),
                        loggedPath.back().lat, loggedPath.back().lon
                    );
                    if (distFromLast > LOG_DISTANCE_METERS) {
                        loggedPath.push_back({gps.location.lat(), gps.location.lng()});
                    }
                }
            }
            break;

        case BACKTRACKING:
            navigateToWaypoint();
            break;
            
        case IDLE:
        case EMERGENCY_STOP:
            stopMotors();
            break;
    }

    // Broadcast data periodically
    if (millis() - lastBroadcastTime > 1000) {
        lastBroadcastTime = millis();
        broadcastData();
    }
    ElegantOTA.loop();
}