/*
 * ============================================================
 *  SWARM ROBOTICS — SLAVE BOT FIRMWARE
 *  ESP32 | Bot ID: 2  (change BOT_ID to 3 for the third bot)
 *
 *  Role  : Follows the master bot's recorded path waypoints
 *          using PID navigation. Maintains a set distance
 *          behind the previous bot in the chain.
 *          Also shares its own position with all peers.
 *
 *  Change BOT_ID and update PEER_MAC_MASTER / PEER_MAC_OTHER
 *  for each physical unit.
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
//  BOT CONFIG  — change per unit
// ─────────────────────────────────────────
#define BOT_ID        2      // ← change to 3 for slave 2
#define NUM_BOTS      3
#define FOLLOW_DELAY  2      // how many waypoints behind master to target
                             // Bot2 = 2, Bot3 = 4 keeps them spaced

// ─────────────────────────────────────────
//  PINS  (same PCB layout on all bots)
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
//  PID TUNING
// ─────────────────────────────────────────
#define KP_DIST   0.9f
#define KI_DIST   0.03f
#define KD_DIST   0.25f
#define KP_ANGLE  1.3f
#define KI_ANGLE  0.08f
#define KD_ANGLE  0.35f

#define BASE_SPEED         150
#define MAX_SPEED          220
#define POSITION_THRESH    4.0f   // cm — "arrived at waypoint"
#define ANGLE_THRESH       5.0f   // degrees

// ─────────────────────────────────────────
//  SENSOR CONFIG
// ─────────────────────────────────────────
#define SCAN_INTERVAL         300
#define SERVO_SETTLE          100
#define DISTANCE_THRESHOLD    50
#define EMERGENCY_STOP_DIST   20
#define OBSTACLE_CONFIRM_MS   1500

// ─────────────────────────────────────────
//  COMMS
// ─────────────────────────────────────────
#define BEACON_INTERVAL  150
#define BOT_TIMEOUT      1500

// ─────────────────────────────────────────
//  PEER MAC ADDRESSES  (update to match)
// ─────────────────────────────────────────
uint8_t masterMac[] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};  // Master Bot 1
uint8_t peerMac[]   = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};  // Other slave

// ─────────────────────────────────────────
//  DATA STRUCTURES
// ─────────────────────────────────────────
typedef struct {
  int   botID;
  float x, y;
  float orientation;
  int   state;
  float velocity;
  float wpX, wpY;
  int   wpIndex;
  unsigned long timestamp;
} BotMessage;

BotMessage myData;
BotMessage peerData[NUM_BOTS + 1];

// Local waypoint buffer  (slave fills from master messages)
#define MAX_WAYPOINTS 30
typedef struct { float x, y; } Waypoint;
Waypoint wpBuffer[MAX_WAYPOINTS];
int wpCount  = 0;
int wpCursor = 0;  // current target index in buffer

// ─────────────────────────────────────────
//  SENSORS
// ─────────────────────────────────────────
typedef struct {
  Servo        servo;
  int          trigPin, echoPin;
  const char*  label;
  int          angles[5]      = {0, 45, 90, 135, 180};
  int          currentIdx     = 0;
  int          direction      = 1;
  unsigned long lastScanTime  = 0;
  bool         waitSettle     = false;
  long         lastDist[5]    = {999,999,999,999,999};
  bool         obstacle[5]    = {false};
  unsigned long firstDetect[5]= {0};
} SensorUnit;

SensorUnit frontSensor = {.trigPin=TRIG_FRONT, .echoPin=ECHO_FRONT, .label="F"};
SensorUnit backSensor  = {.trigPin=TRIG_BACK,  .echoPin=ECHO_BACK,  .label="B"};

// ─────────────────────────────────────────
//  STATE & POSITION
// ─────────────────────────────────────────
volatile long leftEnc  = 0;
volatile long rightEnc = 0;
QMC5883LCompass compass;
float posX    = 0, posY = 0;
float heading = 0;

float pidDistPrevErr   = 0, pidDistIntegral   = 0;
float pidAnglePrevErr  = 0, pidAngleIntegral  = 0;

unsigned long lastBeacon = 0;
bool masterConnected     = false;
bool emergencyStopped    = false;

// ─────────────────────────────────────────
//  ISR
// ─────────────────────────────────────────
void IRAM_ATTR isrLeft()  { leftEnc++;  }
void IRAM_ATTR isrRight() { rightEnc++; }

// ─────────────────────────────────────────
//  MOTOR
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
      int  idx = s.currentIdx;
      long d   = getDistance(s.trigPin, s.echoPin);
      s.lastDist[idx] = d;
      if (d < DISTANCE_THRESHOLD) {
        if (s.firstDetect[idx] == 0) s.firstDetect[idx] = now;
        if (now - s.firstDetect[idx] > OBSTACLE_CONFIRM_MS) s.obstacle[idx] = true;
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
    else if (s.currentIdx <= 0) { s.direction = 1; s.currentIdx = 1; }
    s.servo.write(s.angles[s.currentIdx]);
    s.waitSettle   = true;
    s.lastScanTime = now;
  }
}

bool checkEmergency() {
  for (int i = 0; i < 5; i++) {
    if (frontSensor.obstacle[i] && frontSensor.lastDist[i] < EMERGENCY_STOP_DIST) return true;
    if (backSensor.obstacle[i]  && backSensor.lastDist[i]  < EMERGENCY_STOP_DIST) return true;
  }
  return false;
}

// ─────────────────────────────────────────
//  POSITION
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
  myData.velocity = dist / max((float)(now - lastT) / 1000.0f, 0.001f);
}

// ─────────────────────────────────────────
//  PID NAVIGATION
// ─────────────────────────────────────────
/*  Returns true when waypoint is reached  */
bool navigateTo(float tx, float ty) {
  float ex = tx - posX;
  float ey = ty - posY;
  float dist = sqrt(ex*ex + ey*ey);
  if (dist < POSITION_THRESH) {
    setMotors(0, 0);
    pidDistIntegral  = 0; pidDistPrevErr  = 0;
    pidAngleIntegral = 0; pidAnglePrevErr = 0;
    return true;
  }

  // --- Angle PID ---
  float targetHdg = atan2(ey, ex) * 180.0f / PI;
  if (targetHdg < 0) targetHdg += 360.0f;
  float angleErr = targetHdg - heading;
  if (angleErr >  180) angleErr -= 360;
  if (angleErr < -180) angleErr += 360;

  pidAngleIntegral += angleErr;
  pidAngleIntegral  = constrain(pidAngleIntegral, -200, 200);
  float dAngle      = angleErr - pidAnglePrevErr;
  float angleOut    = KP_ANGLE * angleErr + KI_ANGLE * pidAngleIntegral + KD_ANGLE * dAngle;
  pidAnglePrevErr   = angleErr;
  angleOut = constrain(angleOut, -100, 100);

  // --- Distance PID ---
  pidDistIntegral += dist;
  pidDistIntegral  = constrain(pidDistIntegral, 0, 500);
  float dDist      = dist - pidDistPrevErr;
  float distOut    = KP_DIST * dist + KI_DIST * pidDistIntegral + KD_DIST * dDist;
  pidDistPrevErr   = dist;
  distOut = constrain(distOut, 0, BASE_SPEED);

  // Motor mixing
  int L = (int)(distOut - angleOut);
  int R = (int)(distOut + angleOut);
  L = constrain(L, -MAX_SPEED, MAX_SPEED);
  R = constrain(R, -MAX_SPEED, MAX_SPEED);
  setMotors(L, R);
  return false;
}

// ─────────────────────────────────────────
//  WAYPOINT MANAGEMENT
// ─────────────────────────────────────────
void ingestMasterWaypoint(const BotMessage &msg) {
  // Append new waypoint from master if it's new
  if (wpCount == 0 || wpBuffer[(wpCount - 1) % MAX_WAYPOINTS].x != msg.wpX
                   || wpBuffer[(wpCount - 1) % MAX_WAYPOINTS].y != msg.wpY) {
    wpBuffer[wpCount % MAX_WAYPOINTS] = {msg.wpX, msg.wpY};
    wpCount++;
  }
}

bool followPath() {
  // Calculate which waypoint this slave should aim for
  // (FOLLOW_DELAY waypoints behind master's latest)
  int masterLatest = wpCount - 1;
  int targetIdx    = masterLatest - FOLLOW_DELAY;
  if (targetIdx < 0 || wpCount == 0) return false;   // nothing yet

  int bufIdx = targetIdx % MAX_WAYPOINTS;
  if (navigateTo(wpBuffer[bufIdx].x, wpBuffer[bufIdx].y)) {
    // Advance cursor toward master
    if (wpCursor < masterLatest - FOLLOW_DELAY)
      wpCursor++;
  }
  return true;
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
  if (msg.botID < 1 || msg.botID > NUM_BOTS) return;
  peerData[msg.botID] = msg;

  if (msg.botID == 1) {  // from master
    masterConnected = true;
    ingestMasterWaypoint(msg);
  }
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) { Serial.println("[ESPNOW] Init failed"); ESP.restart(); }
  esp_now_register_send_cb(OnSent);
  esp_now_register_recv_cb(OnRecv);
  esp_now_peer_info_t p = {};
  p.channel = 0; p.encrypt = false;
  memcpy(p.peer_addr, masterMac, 6); esp_now_add_peer(&p);
  memcpy(p.peer_addr, peerMac,   6); esp_now_add_peer(&p);
}

void sendBeacon() {
  myData.botID       = BOT_ID;
  myData.x           = posX;
  myData.y           = posY;
  myData.orientation = heading;
  myData.timestamp   = millis();
  myData.wpIndex     = wpCursor;
  esp_now_send(masterMac, (uint8_t*)&myData, sizeof(myData));
  esp_now_send(peerMac,   (uint8_t*)&myData, sizeof(myData));
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK,  OUTPUT); pinMode(ECHO_BACK,  INPUT);

  pinMode(MOTOR_EN_A, OUTPUT); pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN_B, OUTPUT); pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
  setMotors(0, 0);

  pinMode(ENCODER_LEFT_A,  INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A),  isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), isrRight, RISING);

  Wire.begin();
  compass.init();
  compass.setCalibration(-200, 110, -500, 300, -200, 200);

  initESPNow();

  frontSensor.servo.setPeriodHertz(50); backSensor.servo.setPeriodHertz(50);
  frontSensor.servo.attach(SERVO_FRONT); backSensor.servo.attach(SERVO_BACK);
  frontSensor.servo.write(90); backSensor.servo.write(90);

  memset(peerData, 0, sizeof(peerData));
  Serial.printf("[SLAVE %d] Ready. Waiting for master...\n", BOT_ID);
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  updateCompass();
  updateOdometry();
  updateSensor(frontSensor);
  updateSensor(backSensor);

  // Check master connection
  masterConnected = (millis() - peerData[1].timestamp < BOT_TIMEOUT);
  if (!masterConnected) {
    setMotors(0, 0);
    Serial.println("[SLAVE] Master lost — waiting...");
    delay(200);
    return;
  }

  // Emergency obstacle stop (non-blocking version)
  if (checkEmergency()) {
    if (!emergencyStopped) {
      setMotors(0, 0);
      Serial.println("[SLAVE] EMERGENCY STOP");
      emergencyStopped = true;
    }
  } else {
    emergencyStopped = false;
    // Follow master path
    if (!followPath()) {
      setMotors(0, 0);  // no waypoints yet
    }
  }

  // Beacon
  if (millis() - lastBeacon >= BEACON_INTERVAL) {
    sendBeacon();
    lastBeacon = millis();
  }

  delay(5);
}
