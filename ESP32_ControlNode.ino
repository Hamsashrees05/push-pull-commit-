/**
 * ESP32 CONTROL NODE - COMPLETE VERSION
 * Fixes:
 *   - receiveFromLocalizationNode() now called in loop()
 *   - DECOY mode: random movement + buzzer pattern implemented
 *   - GHOST mode: speed cap enforced on this node
 *   - Smooth differential turning (instead of full pivot)
 *   - Buzzer non-blocking tone pattern for DECOY
 */

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

// ============= PINOUT =============
#define MOTOR_LEFT_IN1   26
#define MOTOR_LEFT_IN2   25
#define MOTOR_LEFT_PWM   33
#define MOTOR_RIGHT_IN1  27
#define MOTOR_RIGHT_IN2  14
#define MOTOR_RIGHT_PWM  32

#define ULTRASONIC_TRIG  12
#define ULTRASONIC_ECHO  13

#define SERVO_PIN        19
#define BUZZER_PIN       18
#define DRONE_MOSFET     17
#define TOUCH_SENSOR_PIN 16

// ============= CONFIGURATION =============
const char*    SSID             = "YourSSID";
const char*    PASSWORD         = "YourPassword";
const char*    WEBSOCKET_SERVER = "192.168.1.100";  // Laptop IP
const uint16_t WEBSOCKET_PORT   = 81;

#define COMMAND_TIMEOUT_MS    2000
#define ULTRASONIC_THRESHOLD  20    // cm – stop forward if closer than this
#define MAX_SPEED             255
#define NORMAL_SPEED          200
#define GHOST_SPEED           100   // enforced regardless of command value
#define TURN_SPEED_INNER      60    // inner wheel speed during smooth turn

// DECOY mode random movement
#define DECOY_MIN_INTERVAL_MS 1500
#define DECOY_MAX_INTERVAL_MS 4000

// Buzzer DECOY pattern (ms ON, ms OFF)
#define BUZZER_ON_MS   150
#define BUZZER_OFF_MS  350

// ============= STATE =============
struct RobotState {
  String  mode           = "RECON";
  int     speed          = NORMAL_SPEED;
  bool    obstacle       = false;
  bool    buzzer_on      = false;
  bool    drone_on       = false;
  unsigned long lastCmdTime    = 0;
  // DECOY
  unsigned long decoyNextChange = 0;
  int           decoyMoveIdx   = 0;
  // Buzzer non-blocking pattern
  unsigned long buzzerToggleAt  = 0;
  bool          buzzerState     = false;
  // Radar sweep
  int           servoAngle      = 90;
  int           servoDirection  = 1;  // 1 = increasing, -1 = decreasing
  unsigned long lastServoMove   = 0;
};

RobotState     state;
WebSocketsClient webSocket;
Servo          servo;
HardwareSerial SerialNode(2);   // UART2 – Localization Node

const char* DECOY_MOVES[] = { "FORWARD", "BACKWARD", "LEFT", "RIGHT" };
const int   DECOY_MOVE_COUNT = 4;

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  SerialNode.begin(9600, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17

  pinMode(MOTOR_LEFT_IN1,  OUTPUT);
  pinMode(MOTOR_LEFT_IN2,  OUTPUT);
  pinMode(MOTOR_LEFT_PWM,  OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(ULTRASONIC_TRIG,  OUTPUT);
  pinMode(ULTRASONIC_ECHO,  INPUT);
  pinMode(BUZZER_PIN,       OUTPUT);
  pinMode(DRONE_MOSFET,     OUTPUT);
  pinMode(TOUCH_SENSOR_PIN, INPUT);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  servo.write(90);  // CENTER for radar sweep

  connectToWiFi();

  webSocket.begin(WEBSOCKET_SERVER, WEBSOCKET_PORT, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  randomSeed(analogRead(0));  // Seed RNG from floating ADC

  Serial.println("[SETUP] Control Node Ready!");
}

// ============= MAIN LOOP =============
void loop() {
  webSocket.loop();
  checkCommandTimeout();
  checkUltrasonic();
  checkTouchSensor();
  receiveFromLocalizationNode();   // <-- was missing before
  runDecoyBehavior();
  runBuzzerPattern();
  runRadarSweep();
  delay(20);
}

// ============= WIFI =============
void connectToWiFi() {
  Serial.printf("[WIFI] Connecting to %s\n", SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500); Serial.print("."); attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WIFI] Failed – running offline");
  }
}

// ============= WEBSOCKET EVENTS =============
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected!");
      stopRobot();
      break;
    case WStype_CONNECTED:
      Serial.println("[WS] Connected to laptop!");
      sendStatus();
      break;
    case WStype_TEXT:
      handleCommand((char*)payload, length);
      break;
    default: break;
  }
}

// ============= COMMAND HANDLER =============
void handleCommand(char* payload, size_t length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload)) return;

  state.lastCmdTime = millis();

  String move      = doc["move"]   | "STOP";
  int    speed     = doc["speed"]  | NORMAL_SPEED;
  String mode      = doc["mode"]   | state.mode;
  String drone_s   = doc["drone"]  | (state.drone_on ? "ON" : "OFF");
  String servo_s   = doc["servo"]  | "NONE";
  String buzzer_s  = doc["buzzer"] | "NONE";

  state.mode  = mode;

  // ---- GHOST: enforce speed cap ----
  if (state.mode == "GHOST") {
    speed = min(speed, GHOST_SPEED);
    state.buzzer_on = false;           // no buzzer in ghost
    digitalWrite(BUZZER_PIN, LOW);
  }

  state.speed = constrain(speed, 0, MAX_SPEED);

  // ---- DECOY: ignore movement commands (handled internally) ----
  if (state.mode != "DECOY") {
    // Obstacle override: block forward
    if (state.obstacle && move == "FORWARD") {
      Serial.println("[SAFETY] Obstacle – blocking FORWARD");
      move = "STOP";
    }
    executeMovement(move);
  }

  // Actuators (always obeyed unless GHOST silences buzzer)
  if (drone_s != "NONE")  executeDrone(drone_s);
  if (servo_s != "NONE")  executeServo(servo_s);
  if (buzzer_s != "NONE" && state.mode != "GHOST") executeBuzzer(buzzer_s);

  Serial.printf("[CMD] move=%s speed=%d mode=%s\n",
                move.c_str(), state.speed, state.mode.c_str());
}

// ============= MOVEMENT =============
void executeMovement(const String& move) {
  if      (move == "FORWARD")  motorsForward(state.speed);
  else if (move == "BACKWARD") motorsBackward(state.speed);
  else if (move == "LEFT")     motorsTurnLeft(state.speed);
  else if (move == "RIGHT")    motorsTurnRight(state.speed);
  else                         stopMotors();
}

void motorsForward(int spd) {
  digitalWrite(MOTOR_LEFT_IN1,  HIGH); digitalWrite(MOTOR_LEFT_IN2,  LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH); digitalWrite(MOTOR_RIGHT_IN2, LOW);
  analogWrite(MOTOR_LEFT_PWM,  spd);
  analogWrite(MOTOR_RIGHT_PWM, spd);
}

void motorsBackward(int spd) {
  digitalWrite(MOTOR_LEFT_IN1,  LOW); digitalWrite(MOTOR_LEFT_IN2,  HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, LOW); digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_PWM,  spd);
  analogWrite(MOTOR_RIGHT_PWM, spd);
}

// Smooth differential turn (one wheel slower, not full pivot)
void motorsTurnLeft(int spd) {
  int inner = constrain(TURN_SPEED_INNER, 0, spd);
  digitalWrite(MOTOR_LEFT_IN1,  HIGH); digitalWrite(MOTOR_LEFT_IN2,  LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH); digitalWrite(MOTOR_RIGHT_IN2, LOW);
  analogWrite(MOTOR_LEFT_PWM,  inner);
  analogWrite(MOTOR_RIGHT_PWM, spd);
}

void motorsTurnRight(int spd) {
  int inner = constrain(TURN_SPEED_INNER, 0, spd);
  digitalWrite(MOTOR_LEFT_IN1,  HIGH); digitalWrite(MOTOR_LEFT_IN2,  LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH); digitalWrite(MOTOR_RIGHT_IN2, LOW);
  analogWrite(MOTOR_LEFT_PWM,  spd);
  analogWrite(MOTOR_RIGHT_PWM, inner);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM,  0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

void stopRobot() {
  stopMotors();
  digitalWrite(BUZZER_PIN,   LOW);
  digitalWrite(DRONE_MOSFET, LOW);
  Serial.println("[SAFETY] Emergency stop");
}

// ============= ACTUATORS =============
void executeServo(const String& cmd) {
  // Manual servo control overrides radar sweep temporarily
  if      (cmd == "OPEN")  { servo.write(180); state.servoAngle = 180; }
  else if (cmd == "CLOSE") { servo.write(0);   state.servoAngle = 0; }
  // Radar sweep will resume on next loop
}

void executeDrone(const String& cmd) {
  state.drone_on = (cmd == "ON");
  digitalWrite(DRONE_MOSFET, state.drone_on ? HIGH : LOW);
}

void executeBuzzer(const String& cmd) {
  if (cmd == "ON") {
    state.buzzer_on = true;
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    state.buzzer_on = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ============= DECOY MODE: random movement + buzzer pattern =============
void runDecoyBehavior() {
  if (state.mode != "DECOY") return;

  unsigned long now = millis();
  if (now >= state.decoyNextChange) {
    state.decoyMoveIdx  = random(0, DECOY_MOVE_COUNT);
    state.decoyNextChange = now + random(DECOY_MIN_INTERVAL_MS,
                                         DECOY_MAX_INTERVAL_MS);
    String randomMove = DECOY_MOVES[state.decoyMoveIdx];

    // Don't go forward if obstacle
    if (state.obstacle && randomMove == "FORWARD") randomMove = "BACKWARD";

    executeMovement(randomMove);
    Serial.printf("[DECOY] Random move: %s\n", randomMove.c_str());
  }
}

// ============= NON-BLOCKING BUZZER PATTERN (DECOY) =============
void runBuzzerPattern() {
  if (state.mode != "DECOY") return;

  unsigned long now = millis();
  if (now >= state.buzzerToggleAt) {
    state.buzzerState = !state.buzzerState;
    digitalWrite(BUZZER_PIN, state.buzzerState ? HIGH : LOW);
    state.buzzerToggleAt = now + (state.buzzerState ? BUZZER_ON_MS : BUZZER_OFF_MS);
  }
}

// ============= RADAR SWEEP =============
void runRadarSweep() {
  unsigned long now = millis();
  if (now - state.lastServoMove < 30) return;  // 30ms between steps = ~1 second per sweep
  state.lastServoMove = now;

  // Move servo
  state.servoAngle += state.servoDirection * 2;  // 2° steps

  // Reverse at limits (0° to 180°)
  if (state.servoAngle >= 180) {
    state.servoAngle = 180;
    state.servoDirection = -1;
  } else if (state.servoAngle <= 0) {
    state.servoAngle = 0;
    state.servoDirection = 1;
  }

  servo.write(state.servoAngle);

  // Send ultrasonic reading with servo angle
  sendRadarData();
}

void sendRadarData() {
  long dist = getUltrasonicDistance();
  
  StaticJsonDocument<256> doc;
  doc["node"]        = "CONTROL";
  doc["ultrasonic"]  = dist;
  doc["servoAngle"]  = state.servoAngle;
  doc["obstacle"]    = (dist > 0 && dist < ULTRASONIC_THRESHOLD);
  doc["timestamp"]   = millis();
  
  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(json);
}

// ============= ULTRASONIC =============
long getUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  long dur = pulseIn(ULTRASONIC_ECHO, HIGH, 23200);
  return dur * 0.034 / 2;
}

void checkUltrasonic() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 100) return;
  lastCheck = millis();

  long dist = getUltrasonicDistance();
  bool prev = state.obstacle;
  state.obstacle = (dist > 0 && dist < ULTRASONIC_THRESHOLD);

  if (state.obstacle && !prev) {
    Serial.printf("[ULTRASONIC] Obstacle at %ld cm\n", dist);
    sendAlert("OBSTACLE", dist);
  }
}

// ============= TOUCH SENSOR =============
void checkTouchSensor() {
  static unsigned long lastCheck = 0;
  static bool lastState = false;
  if (millis() - lastCheck < 200) return;
  lastCheck = millis();

  bool cur = digitalRead(TOUCH_SENSOR_PIN) == HIGH;
  if (cur && !lastState) {
    Serial.println("[TOUCH] Arsenal trigger!");
    executeServo("OPEN");
    sendEventJSON("event", "TOUCH_TRIGGER");
  }
  lastState = cur;
}

// ============= COMMAND TIMEOUT =============
void checkCommandTimeout() {
  if (state.lastCmdTime == 0) return;
  if (millis() - state.lastCmdTime > COMMAND_TIMEOUT_MS) {
    // Only stop if NOT in DECOY (decoy is self-driving)
    if (state.mode != "DECOY") {
      state.lastCmdTime = 0;
      Serial.println("[SAFETY] Command timeout – stop");
      stopRobot();
    }
  }
}

// ============= UART: receive from Localization Node =============
void receiveFromLocalizationNode() {
  if (!SerialNode.available()) return;
  String data = SerialNode.readStringUntil('\n');
  data.trim();
  if (data.length() == 0) return;

  // Relay localization data straight to laptop via WebSocket
  webSocket.sendTXT(data);

  // Optionally parse confidence for local logging
  StaticJsonDocument<256> doc;
  if (!deserializeJson(doc, data)) {
    float conf = doc["confidence"] | 1.0f;
    if (conf < 0.5f) {
      Serial.printf("[LOC] Low confidence: %.2f\n", conf);
    }
  }
}

// ============= STATUS / TELEMETRY =============
void sendStatus() {
  long dist = getUltrasonicDistance();
  StaticJsonDocument<256> doc;
  doc["node"]       = "CONTROL";
  doc["mode"]       = state.mode;
  doc["obstacle"]   = state.obstacle;
  doc["ultrasonic"] = dist;
  doc["speed"]      = state.speed;
  doc["timestamp"]  = millis();
  String json; serializeJson(doc, json);
  webSocket.sendTXT(json);
}

void sendAlert(const String& type, long value) {
  StaticJsonDocument<128> doc;
  doc["alert"]     = type;
  doc["value"]     = value;
  doc["timestamp"] = millis();
  String json; serializeJson(doc, json);
  webSocket.sendTXT(json);
}

void sendEventJSON(const String& key, const String& value) {
  StaticJsonDocument<128> doc;
  doc[key]         = value;
  doc["timestamp"] = millis();
  String json; serializeJson(doc, json);
  webSocket.sendTXT(json);
}
