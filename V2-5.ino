/* version 2.5
*/
// ===== compat =====
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define USE_CORE_3
#else
  #define USE_CORE_2
#endif

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#include <driver/gpio.h>
#include "soc/gpio_struct.h"
#include "esp_system.h"

// ===== wifi =====
char ssid[] = "name";
char pass[] = "password";
uint16_t localPort = 5000;
uint16_t matlabPort = 5001;
IPAddress matlabIP(192,168,0,21);
// IPAddress matlabIP(10,151,143,237);
// IPAddress matlabIP(10,100,177,237);
WiFiUDP Udp;

#define rgbPin 48
#define ledCount 1
Adafruit_NeoPixel pixel(ledCount, rgbPin, NEO_GRB + NEO_KHZ800);

// ===== Bot Geometry =====
const float L1 = 6.0;
const float L2 = 6.0;
const float L3 = 15.0;
const float L4 = 15.0;
const float L5 = 17.0;

// ===== Control Parameters =====
const float countsPerRev = 2000.0;
const float countsPerDeg = countsPerRev / 360.0;

const unsigned long dT_ms = 5;
const float dT = dT_ms / 1000.0;

float Kp = 1;
float Kd = 0.03;

// ===== udp timing =====
const uint32_t T_receive = 10;
const uint32_t T_send = 25;

// ===== struct for a joint =====
struct Joint {
  int in1;
  int in2;
  int pwmPin;

  int pinA;
  int pinB;
  int pinZ;

#ifdef USE_CORE_2
  int pwmChannel;
#endif
  int minPWM;

  // State
  volatile long encoderCount;
  float prevDeg;
  float currentDeg;
  float targetDeg;
  float currentError;
  int lastPWM;

  bool isHomed;

  Joint(int _in1, int _in2, int _pwmPin, int _pinA, int _pinB, int _pinZ
#ifdef USE_CORE_2
        , int _pwmChannel
#endif
       )
    : in1(_in1), in2(_in2), pwmPin(_pwmPin),
      pinA(_pinA), pinB(_pinB), pinZ(_pinZ),
#ifdef USE_CORE_2
      pwmChannel(_pwmChannel),
#endif
      minPWM(150), encoderCount(1), prevDeg(0), currentDeg(0), targetDeg(90), currentError(0), lastPWM(0), isHomed(false)
  {}
};

Joint joint1(7, 15, 16,   6, 5, 4
#ifdef USE_CORE_2
             , 0
#endif
            );

Joint joint2(40, 39, 38,   41, 42, 1
#ifdef USE_CORE_2
             , 1
#endif
            );


void IRAM_ATTR encoderISR1() {
  static uint8_t lastState = 0;
  uint32_t reg = GPIO.in;
  uint8_t state = (((reg >> joint1.pinB) & 1) << 1) | ((reg >> joint1.pinA) & 1);
  static const int8_t lookup[] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  joint1.encoderCount += lookup[(lastState << 2) | state];
  lastState = state;
}
void IRAM_ATTR encoderISR2() {
  static uint8_t lastState = 0;
  uint32_t reg = GPIO.in1.val;
  uint8_t pinA_bit = joint2.pinA - 32;  // 41-32=9
  uint8_t pinB_bit = joint2.pinB - 32;  // 42-32=10
  uint8_t state = (((reg >> pinB_bit) & 1) << 1) | ((reg >> pinA_bit) & 1);
  static const int8_t lookup[] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  joint2.encoderCount += lookup[(lastState << 2) | state];
  lastState = state;
}



// ===== Use z to set 0 =====
void IRAM_ATTR indexISR1() { 
    joint1.encoderCount = 0; 
    joint1.isHomed = true;
}
void IRAM_ATTR indexISR2() { 
    joint2.encoderCount = 0; 
    joint2.isHomed = true;
}

void homeJoint(Joint &j, void (*indexISR)()) {
    attachInterrupt(digitalPinToInterrupt(j.pinZ), indexISR, RISING);

    setMotorPWM(j, 20);
    unsigned long start = millis();
    while (!j.isHomed) {
        if (millis() - start > 3000) {
            setMotorPWM(j, 0);
            sendMessage("Timeout finding home");
            detachInterrupt(digitalPinToInterrupt(j.pinZ));
            return;
        }
    }
    setMotorPWM(j, 0);
    detachInterrupt(digitalPinToInterrupt(j.pinZ));
    sendMessage("Home found");
}


void sendMessage(const char* msg) {
  Udp.beginPacket(matlabIP, matlabPort);
  Udp.print(msg);
  Udp.endPacket();
}

// ===== pwm config =====
const int pwmFreq = 20000;
const int pwmResolution = 8;

void setupPWM(Joint &j) {
#ifdef USE_CORE_3
  ledcAttach(j.pwmPin, pwmFreq, pwmResolution);
#else
  ledcSetup(j.pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(j.pwmPin, j.pwmChannel);
#endif
}

void writePWM(Joint &j, int value) {
#ifdef USE_CORE_3
  ledcWrite(j.pwmPin, value);
#else
  ledcWrite(j.pwmChannel, value);
#endif
}


// ===== deadzone compensation =====
int applyDeadzone(Joint &j, int pwm) {
  if (abs(pwm) < 3) return 0;
  int mapped = map(abs(pwm), 3, 255, j.minPWM, 255);
  return (pwm > 0) ? mapped : -mapped;
}

void setMotorPWM(Joint &j, int pwm) {
  pwm = constrain(pwm, -255, 255);
  pwm = applyDeadzone(j, pwm);

  if (pwm > 0) {
    digitalWrite(j.in1, HIGH);
    digitalWrite(j.in2, LOW);
    writePWM(j, pwm);
  } else if (pwm < 0) {
    digitalWrite(j.in1, LOW);
    digitalWrite(j.in2, HIGH);
    writePWM(j, -pwm);
  } else {
    digitalWrite(j.in1, LOW);
    digitalWrite(j.in2, LOW);
    writePWM(j, 0);
  }
}

void setTarget(Joint &j, float deg) {
  j.targetDeg = deg;
}


void receiveUDP() {
  int packetSize = Udp.parsePacket();
  if (!packetSize) return;

  char buffer[32];
  int len = Udp.read(buffer, sizeof(buffer) - 1);
  if (len <= 0) return;

  buffer[len] = '\0';

  // ===== Pack command =====
  if (strcmp(buffer, "END") == 0) {
    pixel.setPixelColor(0, pixel.Color(0, 200, 200));
    pixel.show();
    setTarget(joint1, -110.0);
    setTarget(joint2, -70.0);
    sendMessage("Packing...");

    unsigned long packStart = millis();
    while (millis() - packStart < 2000) {
      updateJoint(joint1);
      updateJoint(joint2);
      delay(dT_ms);
    }
  esp_restart();
  }

  float x, y;
  if (sscanf(buffer, "%f,%f", &x, &y) == 2) {
    solveIK(x, y);
  }
}


// ===== Inverse Kinematics =====
void solveIK(float x, float y) {
  float r1_sq = x*x + y*y;

  if (r1_sq > (L1+L3)*(L1+L3) || r1_sq < (L1-L3)*(L1-L3)) {
    sendMessage("Left side unreachable");
    return;
  }
  float cos_q3 = constrain((r1_sq - L1*L1 - L3*L3) / (2.0*L1*L3), -1.0, 1.0);
  float sin_q3 = -sqrt(1.0 - cos_q3*cos_q3);
  float q1 = atan2(y, x) - atan2(L3*sin_q3, L1 + L3*cos_q3);

  float xr   = x - L5;
  float r2_sq = xr*xr + y*y;

  if (r2_sq > (L2+L4)*(L2+L4) || r2_sq < (L2-L4)*(L2-L4)) {
    sendMessage("Right side unreachable");
    return;
  }
  float cos_q4 = constrain((r2_sq - L2*L2 - L4*L4) / (2.0*L2*L4), -1.0, 1.0);
  float sin_q4 = sqrt(1.0 - cos_q4*cos_q4);
  float q2 = atan2(y, xr) - atan2(L4*sin_q4, L2 + L4*cos_q4);

  setTarget(joint1, q1 * 180.0 / PI);
  setTarget(joint2, q2 * 180.0 / PI);
}


// ===== Control iteration =====
void updateJoint(Joint &j) {
  long currentCount;
  noInterrupts();
  currentCount = j.encoderCount;
  interrupts();

  j.currentDeg = currentCount / countsPerDeg;

  float error      = j.targetDeg - j.currentDeg;
  float derivative = -(j.currentDeg - j.prevDeg) / dT;
  j.prevDeg        = j.currentDeg;

  float control = constrain(Kp*error + Kd*derivative, -255, 255);
  j.lastPWM     = (int)control;
  setMotorPWM(j, (int)control);

  j.currentError = error;
}


// =============== Setup ===============
void setup() {
  joint1.minPWM = 203;
  joint2.minPWM = 203;
  Serial.begin(115200);
  delay(100);

  pinMode(joint1.in1, OUTPUT);
  pinMode(joint1.in2, OUTPUT);
  pinMode(joint1.pinA, INPUT);
  pinMode(joint1.pinB, INPUT);
  pinMode(joint1.pinZ, INPUT);

  pinMode(joint2.in1, OUTPUT);
  pinMode(joint2.in2, OUTPUT);
  pinMode(joint2.pinA, INPUT);
  pinMode(joint2.pinB, INPUT);
  pinMode(joint2.pinZ, INPUT);

  setupPWM(joint1);
  setupPWM(joint2);

  attachInterrupt(digitalPinToInterrupt(joint1.pinA), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(joint1.pinB), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(joint2.pinA), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(joint2.pinB), encoderISR2, CHANGE);

  pixel.begin();
  pixel.setBrightness(15);


  // ===== wifi =====
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {ledSearching(); delay(50);}
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());
  Udp.begin(localPort);

  // ===== Handshake =====
  Serial.println("Waiting for ACK...");
  bool confirmed = false;
  while (!confirmed) {
    ledWaiting();
    Udp.beginPacket(matlabIP, matlabPort);
    Udp.print("Ready");
    Udp.endPacket();

    int packetSize = Udp.parsePacket();
    if (packetSize > 0) {
      char buffer[16];
      int len = Udp.read(buffer, sizeof(buffer) - 1);
      buffer[len] = '\0';
      if (strcmp(buffer, "ACK") == 0) confirmed = true;
    }
    delay(200);
  }
  Serial.println("Connection successful");

  pixel.setPixelColor(0, pixel.Color(150, 0, 200));
  pixel.show();
  sendMessage("Homing joint 1...");
  homeJoint(joint1, indexISR1);
  joint1.encoderCount = 500;
  delay(500);
  sendMessage("Homing joint 2...");
  homeJoint(joint2, indexISR2);
  joint2.encoderCount = 500;
  delay(500);

  if (!joint1.isHomed || !joint2.isHomed) {
    sendMessage("Homing failed - reset required");
    while (true) { ledError(); delay(50); }
  }

  sendMessage("Setup done. Control loop starting.");
  Serial.println("Setup done. Control loop starting.");
  pixel.setPixelColor(0, pixel.Color(0, 200, 0));
  pixel.show();
}


// =============== Main ===============
void loop() {
  static unsigned long lastTime    = 0;
  static unsigned long lastReceive = 0;
  static unsigned long lastPrint   = 0;

  if (millis() - lastReceive >= T_receive) {
    lastReceive += T_receive;
    receiveUDP();
  }

  if (millis() - lastTime >= dT_ms) {
    lastTime += dT_ms;
    updateJoint(joint1);
    updateJoint(joint2);
  }

  if (millis() - lastPrint >= T_send) {
    lastPrint += T_send;
    Udp.beginPacket(matlabIP, matlabPort);
    Udp.print(joint1.currentDeg, 2); Udp.print(",");
    Udp.print(joint2.currentDeg, 2);
    Udp.endPacket();
  }
}

// ==================== led functions
void ledSearching() {
  static unsigned long t = 0;
  static bool state = false;

  if (millis() - t > 100) {
    t = millis();
    state = !state;

    if (state)
      pixel.setPixelColor(0, pixel.Color(0, 0, 200));
    else
      pixel.setPixelColor(0, 0);
    pixel.show();
  }
}

void ledWaiting() {
  static unsigned long t = 0;
  static bool state = false;

  if (millis() - t > 400) {
    t = millis();
    state = !state;

    if (state)
      pixel.setPixelColor(0, pixel.Color(250, 200, 0));
    else
      pixel.setPixelColor(0, 0);
    pixel.show();
  }
}

void ledError() {
  static unsigned long t = 0;
  static bool state = false;

  if (millis() - t > 200) {
    t = millis();
    state = !state;

    if (state)
      pixel.setPixelColor(0, pixel.Color(200,0,0));
    else
      pixel.setPixelColor(0, 0);
    pixel.show();
  }
}