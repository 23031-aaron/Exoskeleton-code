#include <WiFi.h>
#include <esp_now.h>

const int MAX_TILT  = 17000;
const int MIN_SPEED = 200;
const int MAX_SPEED = 1023;
const unsigned long MAX_ASSIST_MS = 5000;
const unsigned long LINK_TIMEOUT_MS = 500;

typedef struct {
  int tiltA;
  int tiltB;
  bool risingA;
  bool risingB;
} LegData;

volatile LegData incoming;
volatile unsigned long lastPacket = 0;

struct MotorLeg {
  int pwmPin, dirPin, stopPin;
  unsigned long startTime;
  bool on;
};

MotorLeg legA = { 18, 19, 21, 0, false };
MotorLeg legB = { 23, 25, 26, 0, false };

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(LegData)) {
    memcpy((void*)&incoming, data, sizeof(LegData));
    lastPacket = millis();
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);

  setupMotor(legA);
  setupMotor(legB);
}

void loop() {
  bool linkOK = (millis() - lastPacket) < LINK_TIMEOUT_MS;

  LegData data;
  noInterrupts();
  data = *(LegData*)&incoming;
  interrupts();

  if (linkOK) {
    drive(legA, data.tiltA, data.risingA);
    drive(legB, data.tiltB, data.risingB);
  } else {
    stop(legA);
    stop(legB);
  }
  delay(50);
}

void setupMotor(MotorLeg &leg) {
  pinMode(leg.dirPin, OUTPUT);
  pinMode(leg.stopPin, OUTPUT);
  digitalWrite(leg.stopPin, LOW);
  digitalWrite(leg.dirPin, LOW);
  ledcAttach(leg.pwmPin, 1000, 10);
}

void drive(MotorLeg &leg, int tilt, bool rising) {
  if (rising) {
    if (!leg.on) {
      leg.on = true;
      leg.startTime = millis();
    }
    if (millis() - leg.startTime >= MAX_ASSIST_MS) leg.on = false;
  } else {
    leg.on = false;
  }

  int speed = leg.on ? constrain(map(tilt, 0, MAX_TILT, MIN_SPEED, MAX_SPEED), MIN_SPEED, MAX_SPEED) : 0;
  ledcWrite(leg.pwmPin, speed);
}

void stop(MotorLeg &leg) {
  leg.on = false;
  ledcWrite(leg.pwmPin, 0);
}
