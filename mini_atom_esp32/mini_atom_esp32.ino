#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

// —— WiFi & UDP Networking Configurations ——
#include ".env"
const char* ssid = ENV_SSID;
const char* password = ENV_PASSWORD;
const char* laptop_ip = ENV_LAPTOP_IP; // Match your laptop's current active IP
const int local_port = 8888;              // ESP32 listening port
const int remote_port = 9999;             // Laptop Bridge listening port

WiFiUDP udp;

// —— Binary Packet Communication Layouts ——
#pragma pack(push, 1)
struct RobotCommand {
  float linear_velocity_x;
  float angular_velocity_z;
};

struct RobotFeedback {
  int32_t left_encoder_ticks;
  int32_t right_encoder_ticks;
};
#pragma pack(pop)

RobotCommand incomingCmd = {0.0, 0.0};
RobotFeedback outgoingFeedback = {0, 0};

// —— Hardware Pin Configurations ——
#define M1_IN1 35
#define M1_IN2 36
#define M2_IN1 37
#define M2_IN2 38

const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

const int M1_ENCODER_A = 15;
const int M1_ENCODER_B = 16;
const int M2_ENCODER_A = 17;
const int M2_ENCODER_B = 18;

const float TICKS_PER_REV = 1060.0;

// High-speed volatile tracking variables
volatile long tickCountM1 = 0;  
volatile long tickCountM2 = 0;  

void IRAM_ATTR readEncoderM1() { tickCountM1++; }
void IRAM_ATTR readEncoderM2() { tickCountM2++; }

// —— Physical Geometry Calibration ——
const float wheelDiameter = 0.044;    
const float wheelSeparation = 0.095;  
const float ticksPerMeter = TICKS_PER_REV / (PI * wheelDiameter);
const float MAX_VELOCITY = 0.22;       

// Target Control States
float expectedTicksM1 = 0.0;
float expectedTicksM2 = 0.0;
unsigned long prevSnapM1 = 0;
unsigned long prevSnapM2 = 0;

// PD Loop Gains
float Kp = 0.32;
float Kd = 0.08;
long lastErrorM1 = 0;
long lastErrorM2 = 0;

unsigned long lastControlTime = 0;
unsigned long lastFeedbackTime = 0;

void activeBrake() {
  ledcWrite(M1_IN1, 255);
  ledcWrite(M1_IN2, 255);
  ledcWrite(M2_IN1, 255);
  ledcWrite(M2_IN2, 255);
}

void driveHardware(int leftPWM, int rightPWM, bool leftForward, bool rightForward) {
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  if (leftForward) { ledcWrite(M1_IN1, leftPWM); ledcWrite(M1_IN2, 0); }
  else             { ledcWrite(M1_IN1, 0); ledcWrite(M1_IN2, leftPWM); }

  if (rightForward) { ledcWrite(M2_IN1, rightPWM); ledcWrite(M2_IN2, 0); }
  else              { ledcWrite(M2_IN1, 0); ledcWrite(M2_IN2, rightPWM); }
}

void setup() {
  Serial.begin(115200);

  pinMode(40, OUTPUT);
  digitalWrite(40, LOW);

  pinMode(M1_ENCODER_A, INPUT_PULLUP);
  pinMode(M2_ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), readEncoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), readEncoderM2, RISING);

  ledcAttach(M1_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(M1_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(M2_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(M2_IN2, PWM_FREQ, PWM_RESOLUTION);

  activeBrake();

  // Connect to Local Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Start listening for UDP packets
  udp.begin(local_port);
  
  digitalWrite(40, HIGH); // Steady status LED means network is ready
  lastControlTime = millis();
  lastFeedbackTime = millis();
}

void loop() {
  unsigned long now = millis();

  // 1. INCOMING NETWORK CHECK (Asynchronous UDP Consumption)
  int packetSize = udp.parsePacket();
  if (packetSize == sizeof(RobotCommand)) {
    udp.read((char*)&incomingCmd, sizeof(RobotCommand));
  }

  // 2. HARDWARE KINEMATICS & MOTOR PD LOOP (Runs at 100Hz / Every 10ms)
  if (now - lastControlTime >= 10) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    noInterrupts();
    long snapM1 = tickCountM1;
    long snapM2 = tickCountM2;
    interrupts();

    // Track targets based on laptop's parsed velocities
    float targetVelM1 = incomingCmd.linear_velocity_x - (incomingCmd.angular_velocity_z * (wheelSeparation / 2.0));
    float targetVelM2 = incomingCmd.linear_velocity_x + (incomingCmd.angular_velocity_z * (wheelSeparation / 2.0));

    bool forwardM1 = targetVelM1 >= 0;
    bool forwardM2 = targetVelM2 >= 0;

    float absTargetVelM1 = fabs(targetVelM1);
    float absTargetVelM2 = fabs(targetVelM2);

    expectedTicksM1 += (absTargetVelM1 * ticksPerMeter) * dt;
    expectedTicksM2 += (absTargetVelM2 * ticksPerMeter) * dt;

    long errorM1 = (long)expectedTicksM1 - snapM1;
    long errorM2 = (long)expectedTicksM2 - snapM2;

    float derivM1 = (errorM1 - lastErrorM1) / dt;
    float derivM2 = (errorM2 - lastErrorM2) / dt;

    lastErrorM1 = errorM1;
    lastErrorM2 = errorM2;

    int feedforwardM1 = (int)((absTargetVelM1 / MAX_VELOCITY) * 190.0);
    int feedforwardM2 = (int)((absTargetVelM2 / MAX_VELOCITY) * 190.0);

    int pwmOutM1 = feedforwardM1 + (int)((Kp * errorM1) + (Kd * derivM1));
    int pwmOutM2 = feedforwardM2 + (int)((Kp * errorM2) + (Kd * derivM2));

    if (incomingCmd.linear_velocity_x == 0.0 && incomingCmd.angular_velocity_z == 0.0) {
      activeBrake();
      expectedTicksM1 = (float)snapM1;
      expectedTicksM2 = (float)snapM2;
    } else {
      driveHardware(pwmOutM1, pwmOutM2, forwardM1, forwardM2);
    }
  }

  // 3. TELEMETRY OUTBOUND SHIPMENT (Runs at 50Hz / Every 20ms)
  if (now - lastFeedbackTime >= 20) {
    lastFeedbackTime = now;

    noInterrupts();
    outgoingFeedback.left_encoder_ticks = tickCountM1;
    outgoingFeedback.right_encoder_ticks = tickCountM2;
    interrupts();

    // Transmit packed structure over network
    udp.beginPacket(laptop_ip, remote_port);
    udp.write((uint8_t*)&outgoingFeedback, sizeof(RobotFeedback));
    udp.endPacket();
  }
}