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
