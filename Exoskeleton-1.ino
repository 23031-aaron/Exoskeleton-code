#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

const int MPU_ADDR_A = 0x68; // AD0 -> GND
const int MPU_ADDR_B = 0x69; // AD0 -> 3.3V

const int DEAD_ZONE      = 1000;
const int RISE_THRESHOLD = 50;
const int SPIKE_LIMIT    = 4000;

const int MEAN_SAMPLES   = 5;

const int DEBOUNCE_COUNT = 3;

uint8_t receiverMAC[] = {0x68, 0x09, 0x47, 0x28, 0x93, 0x74};

typedef struct {
  int tiltA;
  int tiltB;
  bool risingA;
  bool risingB;
} LegData;

LegData outgoing;

struct SensorLeg {
  int mpuAddr;
  int baseline;
  int prevTilt;
  int riseCount;
  const char* name;
};

SensorLeg legA = { MPU_ADDR_A, 0, 0, 0, "A" };
SensorLeg legB = { MPU_ADDR_B, 0, 0, 0, "B" };

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5);

  wakeMPU(legA.mpuAddr);
  wakeMPU(legB.mpuAddr);

  calibrate(legA);
  calibrate(legB);

  WiFi.mode(WIFI_STA);

  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  esp_now_init();

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  esp_now_add_peer(&peer);
}

void loop() {

  readLeg(legA, outgoing.tiltA, outgoing.risingA);
  readLeg(legB, outgoing.tiltB, outgoing.risingB);

  esp_now_send(
    receiverMAC,
    (uint8_t *)&outgoing,
    sizeof(outgoing)
  );

  Serial.print("A:");
  Serial.print(outgoing.tiltA);

  Serial.print(" B:");
  Serial.println(outgoing.tiltB);

  delay(50);
}

void wakeMPU(int addr) {

  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void calibrate(SensorLeg &leg) {

  long sum = 0;

  for (int i = 0; i < 100; i++) {

    sum += readAccelX(leg.mpuAddr);

    delay(10);
  }

  leg.baseline = sum / 100;

  Serial.print("Leg ");
  Serial.print(leg.name);
  Serial.print(" baseline: ");
  Serial.println(leg.baseline);
}

void readLeg(SensorLeg &leg, int &tiltOut, bool &risingOut) {

  long sum = 0;

  for (int i = 0; i < MEAN_SAMPLES; i++) {

    int raw = readAccelX(leg.mpuAddr) - leg.baseline;

    sum += raw;

    delay(5);
  }

  int tilt = sum / MEAN_SAMPLES;

  if (abs(tilt - leg.prevTilt) > SPIKE_LIMIT) {

    tiltOut = leg.prevTilt;
    risingOut = false;

    leg.riseCount = 0;

    return;
  }

  int change = tilt - leg.prevTilt;

  bool candidate =
    (tilt > DEAD_ZONE) &&
    (change > RISE_THRESHOLD);

  leg.riseCount =
    candidate ? leg.riseCount + 1 : 0;


  tiltOut = tilt;

  risingOut =
    (leg.riseCount >= DEBOUNCE_COUNT);

  leg.prevTilt = tilt;
}

int readAccelX(int addr) {

  Wire.beginTransmission(addr);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(addr, 6, true);

  int16_t x =
    Wire.read() << 8 |
    Wire.read();

  Wire.read();
  Wire.read();
  Wire.read();
  Wire.read();

  return x;
}
