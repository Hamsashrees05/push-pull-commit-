/**
 * ESP32 LOCALIZATION NODE - COMPLETE VERSION
 * Fixes:
 *   - ToF wall-based drift correction now actually implemented
 *   - receiveFromControlNode() parses commands (not just prints)
 *   - Velocity integration improved (double-integrate accel for x,y)
 *   - Confidence resets correctly on ArUco marker correction
 *   - Sends ultrasonic distance from Control Node to laptop
 */

#include <Wire.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <HardwareSerial.h>

// ============= PINOUT =============
#define SDA_PIN 21
#define SCL_PIN 22
#define RX_PIN  16
#define TX_PIN  17
#define LED_PIN  5

// ============= CONFIGURATION =============
const char*    SSID             = "YourSSID";
const char*    PASSWORD         = "YourPassword";
const char*    WEBSOCKET_SERVER = "192.168.1.100";
const uint16_t WEBSOCKET_PORT   = 82;  // Localization on port 82

#define UPDATE_RATE_MS        50      // main loop period
#define CONFIDENCE_DECAY      0.9997f // per 50 ms tick (~0.6% / sec)
#define INITIAL_CONFIDENCE    0.95f
#define MIN_CONFIDENCE        0.20f
#define ACCEL_NOISE_THRESHOLD 0.4f    // m/s² – ignore smaller accel
#define GYRO_LSB_PER_DPS      16.4f   // 2000 DPS range
#define ACCEL_LSB_PER_G       2048.0f // 16 G range

// ToF correction parameters
#define TOF_WALL_KNOWN_DIST_MM 1000   // known wall distance at origin (set per room)
#define TOF_CORRECTION_THRESH  80     // mm – max acceptable drift before snap
#define TOF_CHECK_INTERVAL_MS  300    // how often we attempt ToF correction

// ============= OBJECTS =============
MPU6050          mpu;
VL53L0X          tof;
WebSocketsClient webSocket;
HardwareSerial   SerialControl(1);   // UART1 to Control Node

// ============= LOCALIZATION STATE =============
struct Position {
  float x          = 0.0f;
  float y          = 0.0f;
  float angle      = 0.0f;   // yaw, degrees
  float vx         = 0.0f;   // velocity x m/s
  float vy         = 0.0f;   // velocity y m/s
  float confidence = INITIAL_CONFIDENCE;
};

struct IMURaw {
  float ax, ay, az;     // raw accel (LSB)
  float gx, gy, gz;     // raw gyro  (LSB)
};

Position pos;
IMURaw   imu;

struct Calibration {
  float gyro_z  = 0.0f;
  float accel_x = 0.0f;
  float accel_y = 0.0f;
};
Calibration calib;

// ToF reference tracking
float tof_reference_mm   = -1.0f;   // -1 = not yet set
unsigned long lastTofCorrection = 0;
unsigned long lastUpdate        = 0;

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  SerialControl.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(LED_PIN, OUTPUT);

  delay(500);
  Serial.println("[SETUP] Localization Node starting...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  initMPU();
  initToF();
  connectToWiFi();

  webSocket.begin(WEBSOCKET_SERVER, WEBSOCKET_PORT, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  calibrateSensors();
  lastUpdate = millis();

  Serial.println("[SETUP] Localization Node Ready!");
}

// ============= MAIN LOOP =============
void loop() {
  webSocket.loop();

  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_RATE_MS) {
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    readIMU();
    updatePositionDR(dt);
    applyToFCorrection(now);
    receiveFromControlNode();
    sendTelemetry();

    // Heartbeat LED
    digitalWrite(LED_PIN, (now / 500) % 2);
  }
}

// ============= MPU INIT =============
void initMPU() {
  Serial.println("[MPU] Initializing...");
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G)) {
    Serial.println("[MPU] ERROR: not found on I2C!");
    return;
  }
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);
  Serial.println("[MPU] OK");
}

// ============= TOF INIT =============
void initToF() {
  Serial.println("[ToF] Initializing...");
  if (!tof.begin()) {
    Serial.println("[ToF] ERROR: not found!");
    return;
  }
  tof.setMeasurementTimingBudget(33000);   // 33 ms – balanced speed/accuracy
  tof.startContinuous();
  Serial.println("[ToF] OK");
}

// ============= CALIBRATION =============
void calibrateSensors() {
  Serial.println("[CALIB] Keep robot still for 3 seconds...");
  delay(1000);

  float sum_gz = 0, sum_ax = 0, sum_ay = 0;
  for (int i = 0; i < 150; i++) {
    Vector g = mpu.readRawGyro();
    Vector a = mpu.readRawAccel();
    sum_gz += g.ZAxis;
    sum_ax += a.XAxis;
    sum_ay += a.YAxis;
    delay(10);
  }
  calib.gyro_z  = sum_gz / 150.0f;
  calib.accel_x = sum_ax / 150.0f;
  calib.accel_y = sum_ay / 150.0f;

  Serial.printf("[CALIB] gyro_z=%.2f  ax=%.2f  ay=%.2f\n",
                calib.gyro_z, calib.accel_x, calib.accel_y);
}

// ============= IMU READ =============
void readIMU() {
  Vector ra = mpu.readRawAccel();
  Vector rg = mpu.readRawGyro();

  imu.ax = ra.XAxis - calib.accel_x;
  imu.ay = ra.YAxis - calib.accel_y;
  imu.az = ra.ZAxis;
  imu.gx = rg.XAxis;
  imu.gy = rg.YAxis;
  imu.gz = rg.ZAxis - calib.gyro_z;
}

// ============= DEAD RECKONING =============
void updatePositionDR(float dt) {
  // 1. Update yaw from gyro
  float angular_vel = imu.gz / GYRO_LSB_PER_DPS;  // °/s
  pos.angle += angular_vel * dt;
  if (pos.angle >  360.0f) pos.angle -= 360.0f;
  if (pos.angle <    0.0f) pos.angle += 360.0f;

  // 2. Convert accel to m/s² (remove gravity component is simplified here)
  float ax_ms2 = (imu.ax / ACCEL_LSB_PER_G) * 9.81f;
  float ay_ms2 = (imu.ay / ACCEL_LSB_PER_G) * 9.81f;

  // 3. Threshold – ignore sensor noise at rest
  float magnitude = sqrtf(ax_ms2 * ax_ms2 + ay_ms2 * ay_ms2);
  if (magnitude < ACCEL_NOISE_THRESHOLD) {
    // Decay velocity when no meaningful acceleration (friction model)
    pos.vx *= 0.90f;
    pos.vy *= 0.90f;
  } else {
    // Rotate acceleration from body frame to world frame
    float rad    = pos.angle * 3.14159f / 180.0f;
    float world_ax =  ax_ms2 * cosf(rad) - ay_ms2 * sinf(rad);
    float world_ay =  ax_ms2 * sinf(rad) + ay_ms2 * cosf(rad);

    // Integrate acceleration → velocity
    pos.vx += world_ax * dt;
    pos.vy += world_ay * dt;
  }

  // 4. Integrate velocity → position
  pos.x += pos.vx * dt;
  pos.y += pos.vy * dt;

  // 5. Decay confidence over time
  pos.confidence = fmaxf(MIN_CONFIDENCE,
                          pos.confidence * CONFIDENCE_DECAY);
}

// ============= TOF WALL CORRECTION =============
/**
 * Strategy:
 *   On first valid reading, store it as the reference distance.
 *   On subsequent readings, compute drift = |current - reference|.
 *   If drift exceeds TOF_CORRECTION_THRESH, nudge the robot's
 *   position along its current heading to compensate, and
 *   bump confidence back up slightly.
 *
 *   This is a 1-D correction (perpendicular to a known wall).
 *   It won't fix absolute X,Y but significantly reduces longitudinal drift.
 */
void applyToFCorrection(unsigned long now) {
  if (now - lastTofCorrection < TOF_CHECK_INTERVAL_MS) return;
  lastTofCorrection = now;

  uint16_t dist = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred() || dist > 2000) return;   // bad read

  if (tof_reference_mm < 0) {
    // First valid reading – store as reference
    tof_reference_mm = (float)dist;
    Serial.printf("[ToF] Reference set: %.0f mm\n", tof_reference_mm);
    return;
  }

  float drift = fabsf((float)dist - tof_reference_mm);

  if (drift > TOF_CORRECTION_THRESH) {
    // Compute correction vector along robot heading
    float correction_m = (tof_reference_mm - (float)dist) / 1000.0f;
    float rad = pos.angle * 3.14159f / 180.0f;

    // Apply correction perpendicular to the wall (along heading)
    pos.x += correction_m * cosf(rad);
    pos.y += correction_m * sinf(rad);

    // Damp velocity to prevent oscillation
    pos.vx *= 0.5f;
    pos.vy *= 0.5f;

    // Restore some confidence
    pos.confidence = fminf(INITIAL_CONFIDENCE, pos.confidence + 0.08f);

    Serial.printf("[ToF] Drift correction: %.0f mm → pos nudge %.3f m  conf=%.2f\n",
                  drift, correction_m, pos.confidence);
  }
}

// ============= ARUCO / MARKER CORRECTION (from laptop via WS) =============
void correctFromMarker(float mx, float my, float m_angle) {
  pos.x          = mx;
  pos.y          = my;
  pos.angle      = m_angle;
  pos.vx         = 0.0f;
  pos.vy         = 0.0f;
  pos.confidence = INITIAL_CONFIDENCE;

  // Update ToF reference at this known position
  uint16_t dist = tof.readRangeContinuousMillimeters();
  if (!tof.timeoutOccurred() && dist < 2000) {
    tof_reference_mm = (float)dist;
  }

  Serial.printf("[MARKER] Position corrected → x=%.2f y=%.2f ang=%.1f\n",
                mx, my, m_angle);
  sendAlert("POSITION_CORRECTED");
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
    Serial.println("\n[WIFI] Failed – continuing without network");
  }
}

// ============= WEBSOCKET =============
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("[WS] Connected to laptop");
      break;
    case WStype_TEXT:
      handleIncoming((char*)payload, length);
      break;
    default: break;
  }
}

void handleIncoming(char* payload, size_t length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload)) return;

  // Marker correction message from laptop
  if (doc.containsKey("marker")) {
    float mx      = doc["marker"]["x"]     | 0.0f;
    float my      = doc["marker"]["y"]     | 0.0f;
    float mangle  = doc["marker"]["angle"] | 0.0f;
    correctFromMarker(mx, my, mangle);
  }

  // Mode change (for info only – control node handles movement)
  if (doc.containsKey("mode")) {
    Serial.printf("[MODE] Now: %s\n", doc["mode"].as<const char*>());
  }
}

// ============= UART: receive from Control Node =============
void receiveFromControlNode() {
  if (!SerialControl.available()) return;
  String data = SerialControl.readStringUntil('\n');
  data.trim();
  if (data.length() == 0) return;

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, data)) return;

  // If control node relays ultrasonic distance, include it in next telemetry
  // (stored as a module-level var so sendTelemetry() can pick it up)
  if (doc.containsKey("ultra")) {
    // Store for next telemetry packet
    extern int g_ultrasonic_cm;
    g_ultrasonic_cm = doc["ultra"] | 0;
  }
}

// ============= TELEMETRY =============
int g_ultrasonic_cm = 0;   // updated by receiveFromControlNode

void sendTelemetry() {
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry < 100) return;
  lastTelemetry = millis();

  uint16_t tof_dist = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) tof_dist = 0;

  StaticJsonDocument<256> doc;
  doc["node"]          = "LOCALIZATION";
  doc["pos"][0]        = roundf(pos.x * 1000.0f) / 1000.0f;   // 3 dp
  doc["pos"][1]        = roundf(pos.y * 1000.0f) / 1000.0f;
  doc["angle"]         = roundf(pos.angle * 10.0f) / 10.0f;
  doc["tof"]           = tof_dist;
  doc["ultrasonic"]    = g_ultrasonic_cm;
  doc["confidence"]    = roundf(pos.confidence * 100.0f) / 100.0f;
  doc["timestamp"]     = millis();

  String json;
  serializeJson(doc, json);

  webSocket.sendTXT(json);           // → laptop
  SerialControl.println(json);       // → control node (for status relay)
}

void sendAlert(const String& type) {
  StaticJsonDocument<128> doc;
  doc["alert"]     = type;
  doc["x"]         = pos.x;
  doc["y"]         = pos.y;
  doc["angle"]     = pos.angle;
  doc["timestamp"] = millis();
  String json; serializeJson(doc, json);
  webSocket.sendTXT(json);
}
