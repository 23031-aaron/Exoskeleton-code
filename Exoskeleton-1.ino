#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

const int MPU_ADDR_A = 0x68; // AD0 -> GND
const int MPU_ADDR_B = 0x69; // AD0 -> 3.3V

const int RISE_THRESHOLD = 50;
const int SPIKE_LIMIT    = 4000;

const int MEAN_SAMPLES   = 5;
const int DEBOUNCE_COUNT = 3;

const int MAX_TILT  = 16000;
const int MIN_SPEED = 300;
const int MAX_SPEED = 1023;

uint8_t receiverMAC[] = {
  0x68, 0x09, 0x47, 0x28, 0x93, 0x74
};

typedef struct {
  int speedA;
  int speedB;
} LegData;

LegData outgoing;

struct SensorLeg {
  int mpuAddr;
  int baseline;

  int samples[MEAN_SAMPLES];
  int sampleIndex;
  bool bufferFull;

  int prevTilt;
  int riseCount;

  const char* name;
};

SensorLeg legA = {
  MPU_ADDR_A, 0, {0}, 0, false, 0, 0, "A"
};

SensorLeg legB = {
  MPU_ADDR_B, 0, {0}, 0, false, 0, 0, "B"
};


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

  outgoing.speedA = readSpeed(legA);
  outgoing.speedB = readSpeed(legB);

  esp_now_send(
    receiverMAC,
    (uint8_t *)&outgoing,
    sizeof(outgoing)
  );

  Serial.print("A:");
  Serial.print(outgoing.speedA);

  Serial.print(" B:");
  Serial.println(outgoing.speedB);

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


int readSpeed(SensorLeg &leg) {

  int raw = readAccelX(leg.mpuAddr) - leg.baseline;

  leg.samples[leg.sampleIndex] = raw;

  leg.sampleIndex++;

  if (leg.sampleIndex >= MEAN_SAMPLES) {

    leg.sampleIndex = 0;
    leg.bufferFull = true;
  }

  long sum = 0;

  int numberOfSamples =
    leg.bufferFull ? MEAN_SAMPLES : leg.sampleIndex;

  if (numberOfSamples == 0)
    numberOfSamples = 1;

  for (int i = 0; i < numberOfSamples; i++) {

    sum += leg.samples[i];
  }

  int tilt = sum / numberOfSamples;

  if (abs(tilt - leg.prevTilt) > SPIKE_LIMIT) {

    leg.riseCount = 0;

    return 0;
  }

  int change = tilt - leg.prevTilt;

  bool candidate =
    (change > RISE_THRESHOLD);

  if (candidate) {

    leg.riseCount++;

  } else {

    leg.riseCount = 0;
  }


  bool rising =
    (leg.riseCount >= DEBOUNCE_COUNT);

  if (!rising) {

    return 0;
  }

  int speed = map(
    tilt,
    0,
    MAX_TILT,
    MIN_SPEED,
    MAX_SPEED
  );


  return constrain(
    speed,
    MIN_SPEED,
    MAX_SPEED
  );
}


int readAccelX(int addr) {

  Wire.beginTransmission(addr);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(addr, 6, true);

  int16_t x =
    Wire.read() << 8 |
    Wire.read();

  // Ignore Y and Z
  Wire.read();
  Wire.read();
  Wire.read();
  Wire.read();

  return x;
}
