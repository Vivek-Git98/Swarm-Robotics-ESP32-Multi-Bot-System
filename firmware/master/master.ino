/*
 * ============================================================
 *  SWARM ROBOTICS — MASTER BOT FIRMWARE
 *  ESP32 | Bot ID: 1
 *
 *  Role  : Receives manual commands from PC GUI over Serial,
 *          moves accordingly, records path waypoints, and
 *          broadcasts position + waypoints to slave bots
 *          via ESP-NOW so they follow the same trail.
 *
 *  Author: (Your Name)
 *  Date  : 2025
 * ============================================================
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <QMC5883LCompass.h>
#include <Wire.h>

// ─────────────────────────────────────────
//  BOT CONFIG
// ─────────────────────────────────────────
#define BOT_ID            1
#define NUM_BOTS          3
#define MAX_WAYPOINTS     30     // circular path buffer
#define WAYPOINT_SPACING  15.0f  // cm — record new waypoint every N cm

// ─────────────────────────────────────────
//  PINS
// ─────────────────────────────────────────
#define ENCODER_LEFT_A   34
#define ENCODER_RIGHT_A  36

#define TRIG_FRONT  5
#define ECHO_FRONT  18
#define SERVO_FRONT 33
#define TRIG_BACK   25
#define ECHO_BACK   26
#define SERVO_BACK  12

#define MOTOR_EN_A  27
#define MOTOR_IN1   14
#define MOTOR_IN2   32
#define MOTOR_EN_B  13
#define MOTOR_IN3    4
#define MOTOR_IN4   15

// ─────────────────────────────────────────
//  WHEEL / ENCODER CONSTANTS
// ─────────────────────────────────────────
#define WHEEL_RADIUS_CM  3.3f
#define WHEEL_BASE_CM    14.0f
#define PULSES_PER_REV   330.0f
#define PI               3.14159265f

// ─────────────────────────────────────────
//  SENSOR / SCAN CONFIG
// ─────────────────────────────────────────
#define SCAN_INTERVAL            300   // ms between servo steps
#define SERVO_SETTLE             100   // ms after servo move
#define DISTANCE_THRESHOLD       50    // cm — obstacle detection
#define EMERGENCY_STOP_DIST      20    // cm
#define OBSTACLE_CONFIRM_DELAY   1500  // ms

// ─────────────────────────────────────────
//  COMMS
// ─────────────────────────────────────────
#define BEACON_INTERVAL   150   // ms
#define BOT_TIMEOUT       1500  // ms — peer considered lost

// ─────────────────────────────────────────
//  MOTOR CONTROL
// ─────────────────────────────────────────
#define BASE_SPEED         160
#define TURN_SPEED         120
#define SERIAL_BAUD        115200

// ─────────────────────────────────────────
//  PEER MAC ADDRESSES  (update to match your bots)
// ─────────────────────────────────────────
uint8_t slaveMac1[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};  // Slave Bot 2
uint8_t slaveMac2[] = {0x24, 0x6F, 0x28, 0x12, 0x34, 0x56};  // Slave Bot 3

// ─────────────────────────────────────────
//  DATA STRUCTURES
// ─────────────────────────────────────────

// Shared over ESP-NOW
typedef struct {
  int   botID;
  float x, y;
  float orientation;
  int   state;          // master uses: 0=MANUAL, 1=AUTO_FOLLOW, 2=STOP
  float velocity;
  // Waypoint the slaves should head to
  float wpX, wpY;
  int   wpIndex;        // slaves use this to stay in order
  unsigned long timestamp;
} BotMessage;

BotMessage myData;
BotMessage peerData[NUM_BOTS + 1];

// Waypoint circular buffer
typedef struct {
  float x, y;
} Waypoint;

Waypoint waypoints[MAX_WAYPOINTS];
int wpHead    = 0;   // next write position
int wpCount   = 0;   // total valid waypoints
float lastWpX = 0, lastWpY = 0;

// Sensor scanning state
typedef struct {
  Servo        servo;
  int          trigPin, echoPin;
  const char*  label;
  int          angles[5]     = {0, 45, 90, 135, 180};
  int          currentIdx    = 0;
  int          direction     = 1;
  unsigned long lastScanTime = 0;
  bool         waitSettle    = false;
  long         lastDist[5]   = {999,999,999,999,999};
  bool         obstacle[5]   = {false};
  unsigned long firstDetect[5] = {0};
} SensorUnit;

SensorUnit frontSensor = {.trigPin = TRIG_FRONT, .echoPin = ECHO_FRONT, .label = "F"};
SensorUnit backSensor  = {.trigPin = TRIG_BACK,  .echoPin = ECHO_BACK,  .label = "B"};

// ─────────────────────────────────────────
//  POSITION / ORIENTATION
// ─────────────────────────────────────────
volatile long leftEnc  = 0;
volatile long rightEnc = 0;
QMC5883LCompass compass;
float posX = 0, posY = 0;
float heading = 0;    // degrees, 0 = North

// ─────────────────────────────────────────
//  CONTROL STATE
// ─────────────────────────────────────────
enum MasterState { MANUAL, AUTO_FOLLOW, STOPPED };
MasterState masterState = STOPPED;

// Manual drive commands from GUI
int cmdLeft  = 0;
int cmdRight = 0;

unsigned long lastBeacon = 0;

// ─────────────────────────────────────────
//  ISR — ENCODERS
// ─────────────────────────────────────────
void IRAM_ATTR isrLeft()  { leftEnc++;  }
void IRAM_ATTR isrRight() { rightEnc++; }

// ─────────────────────────────────────────
//  MOTOR HELPER
// ─────────────────────────────────────────
void setMotors(int L, int R) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);
  analogWrite(MOTOR_EN_A, abs(L));
  analogWrite(MOTOR_EN_B, abs(R));
  digitalWrite(MOTOR_IN1, L > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, L > 0 ? LOW  : HIGH);
  digitalWrite(MOTOR_IN3, R > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_IN4, R > 0 ? LOW  : HIGH);
}

// ─────────────────────────────────────────
//  SENSOR
// ─────────────────────────────────────────
long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 25000);
  return (d == 0) ? 999 : d * 0.034 / 2;
}

void updateSensor(SensorUnit &s) {
  unsigned long now = millis();
  if (s.waitSettle) {
    if (now - s.lastScanTime > SERVO_SETTLE) {
      int idx = s.currentIdx;
      long d  = getDistance(s.trigPin, s.echoPin);
      s.lastDist[idx] = d;
      // Simple debounce confirmation
      if (d < DISTANCE_THRESHOLD) {
        if (s.firstDetect[idx] == 0) s.firstDetect[idx] = now;
        if (now - s.firstDetect[idx] > OBSTACLE_CONFIRM_DELAY) s.obstacle[idx] = true;
      } else {
        s.firstDetect[idx] = 0;
        s.obstacle[idx]    = false;
      }
      s.waitSettle   = false;
      s.lastScanTime = now;
    }
  } else if (now - s.lastScanTime > SCAN_INTERVAL) {
    s.currentIdx += s.direction;
    if (s.currentIdx >= 4) { s.direction = -1; s.currentIdx = 3; }
    else if (s.currentIdx <= 0) { s.direction =  1; s.currentIdx = 1; }
    s.servo.write(s.angles[s.currentIdx]);
    s.waitSettle   = true;
    s.lastScanTime = now;
  }
}

bool emergencyObstacle() {
  for (int i = 0; i < 5; i++) {
    if (frontSensor.obstacle[i] && frontSensor.lastDist[i] < EMERGENCY_STOP_DIST) return true;
    if (backSensor.obstacle[i]  && backSensor.lastDist[i]  < EMERGENCY_STOP_DIST) return true;
  }
  return false;
}

// ─────────────────────────────────────────
//  POSITION TRACKING
// ─────────────────────────────────────────
void updateCompass() {
  compass.read();
  heading = compass.getAzimuth();
}

void updateOdometry() {
  static long prevL = 0, prevR = 0;
  static unsigned long lastT = 0;
  unsigned long now = millis();
  if (now - lastT < 80) return;

  noInterrupts();
  long cL = leftEnc; long cR = rightEnc;
  interrupts();

  long dL = cL - prevL; long dR = cR - prevR;
  prevL = cL; prevR = cR; lastT = now;

  float distL = 2 * PI * WHEEL_RADIUS_CM * dL / PULSES_PER_REV;
  float distR = 2 * PI * WHEEL_RADIUS_CM * dR / PULSES_PER_REV;
  float dist  = (distL + distR) / 2.0f;
  float rad   = heading * PI / 180.0f;
  posX += dist * cos(rad);
  posY += dist * sin(rad);

  float vel = dist / ((now - lastT) / 1000.0f + 0.001f);
  myData.velocity = vel;

  // Record waypoint if moved far enough
  float dx = posX - lastWpX, dy = posY - lastWpY;
  if (sqrt(dx*dx + dy*dy) >= WAYPOINT_SPACING) {
    waypoints[wpHead] = {posX, posY};
    wpHead  = (wpHead + 1) % MAX_WAYPOINTS;
    wpCount = min(wpCount + 1, MAX_WAYPOINTS);
    lastWpX = posX; lastWpY = posY;
  }
}

// ─────────────────────────────────────────
//  ESP-NOW
// ─────────────────────────────────────────
void OnSent(const uint8_t*, esp_now_send_status_t s) {
  if (s != ESP_NOW_SEND_SUCCESS) Serial.println("[ESPNOW] Send fail");
}

void OnRecv(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  BotMessage msg;
  memcpy(&msg, data, sizeof(msg));
  if (msg.botID >= 1 && msg.botID <= NUM_BOTS)
    peerData[msg.botID] = msg;
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) { Serial.println("[ESPNOW] Init failed"); ESP.restart(); }
  esp_now_register_send_cb(OnSent);
  esp_now_register_recv_cb(OnRecv);
  esp_now_peer_info_t p = {};
  p.channel = 0; p.encrypt = false;
  memcpy(p.peer_addr, slaveMac1, 6); esp_now_add_peer(&p);
  memcpy(p.peer_addr, slaveMac2, 6); esp_now_add_peer(&p);
}

void sendBeacon() {
  myData.botID       = BOT_ID;
  myData.x           = posX;
  myData.y           = posY;
  myData.orientation = heading;
  myData.state       = masterState;
  myData.timestamp   = millis();

  // Send latest waypoint for slaves to follow
  if (wpCount > 0) {
    int latestWp    = (wpHead - 1 + MAX_WAYPOINTS) % MAX_WAYPOINTS;
    myData.wpX      = waypoints[latestWp].x;
    myData.wpY      = waypoints[latestWp].y;
    myData.wpIndex  = wpHead;
  }

  esp_now_send(slaveMac1, (uint8_t*)&myData, sizeof(myData));
  esp_now_send(slaveMac2, (uint8_t*)&myData, sizeof(myData));
}

// ─────────────────────────────────────────
//  SERIAL GUI COMMAND PARSER
//  Protocol (newline-terminated):
//    F<speed>   — forward   e.g. F160
//    B<speed>   — backward  e.g. B120
//    L<speed>   — turn left
//    R<speed>   — turn right
//    S          — stop
//    A          — auto-follow mode ON
//    M          — manual mode ON
//    ?          — telemetry request
// ─────────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  char ch  = cmd.charAt(0);
  int  val = cmd.length() > 1 ? cmd.substring(1).toInt() : BASE_SPEED;
  val = constrain(val, 0, 255);

  switch (ch) {
    case 'F': case 'f':
      masterState = MANUAL; cmdLeft =  val; cmdRight =  val; break;
    case 'B': case 'b':
      masterState = MANUAL; cmdLeft = -val; cmdRight = -val; break;
    case 'L': case 'l':
      masterState = MANUAL; cmdLeft = -TURN_SPEED; cmdRight = TURN_SPEED; break;
    case 'R': case 'r':
      masterState = MANUAL; cmdLeft =  TURN_SPEED; cmdRight = -TURN_SPEED; break;
    case 'S': case 's':
      masterState = STOPPED; cmdLeft = 0; cmdRight = 0; break;
    case 'A': case 'a':
      masterState = AUTO_FOLLOW; Serial.println("AUTO_FOLLOW"); break;
    case 'M': case 'm':
      masterState = MANUAL; Serial.println("MANUAL"); break;
    case '?':
      // Send JSON telemetry back to GUI
      Serial.printf("{\"id\":%d,\"x\":%.2f,\"y\":%.2f,\"hdg\":%.1f,"
                    "\"state\":%d,\"wp\":%d,\"vel\":%.2f}\n",
                    BOT_ID, posX, posY, heading, masterState, wpCount, myData.velocity);
      // Also send peer data
      for (int i = 1; i <= NUM_BOTS; i++) {
        if (i == BOT_ID) continue;
        if (millis() - peerData[i].timestamp < BOT_TIMEOUT) {
          Serial.printf("{\"id\":%d,\"x\":%.2f,\"y\":%.2f,\"hdg\":%.1f,\"state\":%d}\n",
                        i, peerData[i].x, peerData[i].y,
                        peerData[i].orientation, peerData[i].state);
        }
      }
      break;
  }
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Sensor pins
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK,  OUTPUT); pinMode(ECHO_BACK,  INPUT);

  // Motor pins
  pinMode(MOTOR_EN_A, OUTPUT); pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN_B, OUTPUT); pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
  setMotors(0, 0);

  // Encoders
  pinMode(ENCODER_LEFT_A,  INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A),  isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), isrRight, RISING);

  // Compass
  Wire.begin();
  compass.init();
  compass.setCalibration(-200, 110, -500, 300, -200, 200);

  // ESP-NOW
  initESPNow();

  // Servos
  frontSensor.servo.setPeriodHertz(50);
  backSensor.servo.setPeriodHertz(50);
  frontSensor.servo.attach(SERVO_FRONT);
  backSensor.servo.attach(SERVO_BACK);
  frontSensor.servo.write(90);
  backSensor.servo.write(90);

  memset(peerData, 0, sizeof(peerData));
  Serial.println("[MASTER] Ready. Connect GUI and send commands.");
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  updateCompass();
  updateOdometry();
  handleSerial();
  updateSensor(frontSensor);
  updateSensor(backSensor);

  // Emergency obstacle override
  if (emergencyObstacle()) {
    setMotors(0, 0);
    delay(100);
    setMotors(-100, -100);  // brief reverse
    delay(200);
    setMotors(0, 0);
    Serial.println("[MASTER] EMERGENCY STOP");
  } else {
    // Normal drive
    if (masterState == MANUAL)  setMotors(cmdLeft, cmdRight);
    if (masterState == STOPPED) setMotors(0, 0);
    // AUTO_FOLLOW: master just continues last manual command; slaves handle path following
  }

  // Periodic beacon to slaves
  if (millis() - lastBeacon >= BEACON_INTERVAL) {
    sendBeacon();
    lastBeacon = millis();
  }

  delay(5);
}
