#include <AccelStepper.h>
#include <Servo.h>

/*
  ⚡ Fixed SCARA Firmware - Siam's Final Servo Synchronization 🎯 
*/

#define MOTOR_INTERFACE_TYPE 1

const int STEP_PIN_1 = 8;  const int DIR_PIN_1  = 9;
const int STEP_PIN_2 = 10; const int DIR_PIN_2  = 11;
const int STEP_PIN_3 = 12; const int DIR_PIN_3  = 13;
const int STEP_PIN_4 = 6;  const int DIR_PIN_4  = 7;
const int JAW_SERVO_PIN = 5; 

const float STEPS_PER_DEGREE = 3200.0 / 360.0;
const float MOTOR_1_MULTIPLIER = 3.0;
const float MOTOR_2_MULTIPLIER = 3.0;
const float MOTOR_3_MULTIPLIER = 1.0;
const float MOTOR_4_MULTIPLIER = 3.0;

const float MAX_SPEED_1 = 1000.0; const float MAX_SPEED_2 = 1000.0;
const float MAX_SPEED_3 = 1000.0;  const float MAX_SPEED_4 = 900.0;
const float ACCEL_1 = 500.0;       const float ACCEL_2 = 500.0;
const float ACCEL_3 = 500.0;       const float ACCEL_4 = 450.0;

const float JAW_OPEN_DEG = 0.0;
const float JAW_CLOSED_DEG = 50.0;

AccelStepper stepper1(MOTOR_INTERFACE_TYPE, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper3(MOTOR_INTERFACE_TYPE, STEP_PIN_3, DIR_PIN_3);
AccelStepper stepper4(MOTOR_INTERFACE_TYPE, STEP_PIN_4, DIR_PIN_4);

Servo jawServo;

const byte SERIAL_BUFFER_SIZE = 96;
char serialBuffer[SERIAL_BUFFER_SIZE];

long degreeToSteps(float degrees, float multiplier) {
  return lround(degrees * STEPS_PER_DEGREE * multiplier);
}

float clampJawAngle(float jaw) {
  if (jaw < JAW_OPEN_DEG) return JAW_OPEN_DEG;
  if (jaw > JAW_CLOSED_DEG) return JAW_CLOSED_DEG;
  return jaw;
}

void configureStepper(AccelStepper &stepper, float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
}

bool parseMoveCommand(char *line, float &j1, float &j2, float &z, float &jaw) {
  char *token = strtok(line, ",");
  if (token == NULL || strcmp(token, "MOVE") != 0) return false;

  token = strtok(NULL, ","); if (token == NULL) return false; j1 = atof(token);
  token = strtok(NULL, ","); if (token == NULL) return false; j2 = atof(token);
  token = strtok(NULL, ","); if (token == NULL) return false; z = atof(token);
  token = strtok(NULL, ","); if (token == NULL) return false; jaw = atof(token);
  return true;
}

/* 🎯 ফিক্সড এক্সিকিউশন ফাংশন */
void executeMove(float j1, float j2, float z, float jaw) {
 // float j4Deg = (j1 + j2);
 float j4Deg = j2 - j1;;

  long target1 = degreeToSteps(j1, MOTOR_1_MULTIPLIER);
  long target2 = degreeToSteps(j2, MOTOR_2_MULTIPLIER);
  long target3 = degreeToSteps(z, MOTOR_3_MULTIPLIER);
  long target4 = degreeToSteps(j4Deg, MOTOR_4_MULTIPLIER);

  stepper1.moveTo(target1);
  stepper2.moveTo(target2);
  stepper3.moveTo(target3);
  stepper4.moveTo(target4);

  // ১. স্টেপার মোটরগুলো চলা শেষ করবে
  while (
    stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 ||
    stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0
  ) {
    stepper1.run(); stepper2.run(); stepper3.run(); stepper4.run();
  }

  // ২. স্টেপার থামার পর সার্ভোকে পজিশন দেওয়া হবে
  float safeJaw = clampJawAngle(jaw);
  jawServo.write((int)safeJaw);
  
  // 🧠 টাইমিং ফিক্স ডিলে: সার্ভোকে ফিজিক্যালি হা বা বন্ধ হওয়ার জন্য ৫০০ মিলি-সেকেন্ড সময় দেওয়া হলো
  delay(500); 

  // ৩. সার্ভো ঘোরার কাজ শেষ হলে তবেই পাইথনকে সিগন্যাল পাঠানো হবে
  Serial.println("DONE");
}

void setup() {
  Serial.begin(115200); 

  configureStepper(stepper1, MAX_SPEED_1, ACCEL_1);
  configureStepper(stepper2, MAX_SPEED_2, ACCEL_2);
  configureStepper(stepper3, MAX_SPEED_3, ACCEL_3);
  configureStepper(stepper4, MAX_SPEED_4, ACCEL_4);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);

  jawServo.attach(JAW_SERVO_PIN);
  jawServo.write((int)JAW_OPEN_DEG); // শুরুতে ০ ডিগ্রিতে হা করে থাকবে

  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) return;

  size_t length = Serial.readBytesUntil('\n', serialBuffer, SERIAL_BUFFER_SIZE - 1);
  serialBuffer[length] = '\0';

  for (byte i = 0; i < length; i++) {
    if (serialBuffer[i] == '\r') {
      serialBuffer[i] = '\0';
      break;
    }
  }

  float j1 = 0.0, j2 = 0.0, z = 0.0, jaw = 0.0;

  if (parseMoveCommand(serialBuffer, j1, j2, z, jaw)) {
    executeMove(j1, j2, z, jaw);
  } else {
    Serial.println("ERROR");
  }

  // ✅ ফিক্সড: সিরিয়াল বাফার পারফেক্টলি খালি করার লুপ
  while (Serial.available() > 0) {
    Serial.read(); 
  }
}