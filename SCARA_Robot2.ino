#include <AccelStepper.h>
#include <Servo.h>
#include <NewPing.h>
// Stepper drivers
AccelStepper stepper1(AccelStepper::DRIVER, 2, 5);
AccelStepper stepper2(AccelStepper::DRIVER, 3, 6);
AccelStepper stepper3(AccelStepper::DRIVER, 12,13);
AccelStepper stepper4(AccelStepper::DRIVER, 4, 7);
Servo gripperServo;
// Limit switches
#define LIMIT_SWITCH_PIN_1 9
#define LIMIT_SWITCH_PIN_2 10
#define LIMIT_SWITCH_PIN_3 11
// HC-SR04 ultrasonic sensor
#define SRF05_TRIG_PIN A1
#define SRF05_ECHO_PIN A2
#define SRF05_MAX_CM   50
NewPing sonar(SRF05_TRIG_PIN, SRF05_ECHO_PIN, SRF05_MAX_CM);
// Kinematics constants
const float TH1 = 92.12f;
const float TH2 = 71.111111f;
const float PHI = 0.0f;
const float ZSTEP = 200.0f;
// Command parsing and state machine
enum State { IDLE, MANUAL_STEP, TRAJECTORY } state = IDLE;
enum TrajPhase { PHASE1, PHASE2 } trajPhase;
int bufData[10];
int manualID, manualStep;
int trajMode;
long tarPos[4];
int gripVal, speedVal, accelVal;
// Obstacle detection timing
unsigned long lastPing = 0;
int lastCm = -1;
// Homing routine
void manualHoming() {
  stepper1.setSpeed(-1000);
  stepper2.setSpeed(-1000);
  stepper4.setSpeed(-2000);
  bool d1=false, d2=false, d4=false;
  while (!(d1&&d2&&d4)) {
    if (!d1) { if (digitalRead(LIMIT_SWITCH_PIN_1)==HIGH) stepper1.runSpeed(); else { stepper1.setCurrentPosition(0); d1=true; } }
    if (!d2) { if (digitalRead(LIMIT_SWITCH_PIN_2)==HIGH) stepper2.runSpeed(); else { stepper2.setCurrentPosition(0); d2=true; } }
    if (!d4) { if (digitalRead(LIMIT_SWITCH_PIN_3)==HIGH) stepper4.runSpeed(); else { stepper4.setCurrentPosition(0); d4=true; } }
  }
  gripperServo.write(0);
  Serial.println("Homing complete");
}
bool obstacleDetected() {
  unsigned long now = millis();
  if (now - lastPing >= 20) {
    lastPing = now;
    lastCm   = sonar.ping_cm();
  }
  return (lastCm > 0 && lastCm <= 3);
}
void setup() {
  Serial.begin(115200);

  // Configure steppers
  stepper1.setMaxSpeed(4000); stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000); stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000); stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(4000); stepper4.setAcceleration(2000);

  // Limit switches
  pinMode(LIMIT_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_2, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_3, INPUT_PULLUP);

  // Servo
  gripperServo.attach(A0, 600, 2500);
  gripperServo.write(0);

  // Initial homing
  manualHoming();
}
void loop() {
  // 1) Global obstacle check
  if (obstacleDetected()) {
    Serial.println("OBJECT_TOO_CLOSE");
    // stop all
    stepper1.stop(); stepper2.stop(); stepper3.stop(); stepper4.stop();
    manualHoming();
    Serial.println("DONE");
    state = IDLE;
    delay(300);
    return;
  }
  // 2) Manual stepping state
  if (state == MANUAL_STEP) {
    if (manualID == 1) stepper1.run();
    else if (manualID == 2) stepper2.run();
    else if (manualID == 4) stepper4.run();
    // when motor done
    bool done = (manualID==1 && !stepper1.isRunning()) ||
                (manualID==2 && !stepper2.isRunning()) ||
                (manualID==4 && !stepper4.isRunning());
    if (done) {
      Serial.print("CURRENT,");
      Serial.print(stepper1.currentPosition()); Serial.print(",");
      Serial.print(stepper2.currentPosition()); Serial.print(",");
      Serial.println(stepper4.currentPosition());
      state = IDLE;
    }
    return;
  }
  // 3) Trajectory state with sequencing
  if (state == TRAJECTORY) {
    if (trajMode == 1) {
      // simultaneous XY+Z
      stepper1.run(); stepper2.run(); stepper3.run(); stepper4.run();
      if (!stepper1.isRunning() && !stepper2.isRunning() &&
          !stepper3.isRunning() && !stepper4.isRunning()) {
        // now actuate gripper
        gripperServo.write(gripVal);
        delay(300);
        Serial.println("DONE");
        state = IDLE;
      }
      return;
    }
    if (trajMode == 2) {
      // Z -> then XY
      if (trajPhase == PHASE1) {
        stepper4.run();
        if (!stepper4.isRunning()) {
          // start XY
          stepper1.moveTo(tarPos[0]);
          stepper2.moveTo(tarPos[1]);
          stepper3.moveTo(tarPos[2]);
          trajPhase = PHASE2;
        }
      }
      else {
        stepper1.run(); stepper2.run(); stepper3.run();
        if (!stepper1.isRunning() && !stepper2.isRunning() && !stepper3.isRunning()) {
          gripperServo.write(gripVal);
          delay(300);
          Serial.println("DONE");
          state = IDLE;
        }
      }
      return;
    }

    if (trajMode == 3) {
      // XY -> then Z
      if (trajPhase == PHASE1) {
        stepper1.run(); stepper2.run(); stepper3.run();
        if (!stepper1.isRunning() && !stepper2.isRunning() && !stepper3.isRunning()) {
          stepper4.moveTo(tarPos[3]);
          trajPhase = PHASE2;
        }
      }
      else {
        stepper4.run();
        if (!stepper4.isRunning()) {
          gripperServo.write(gripVal);
          delay(300);
          Serial.println("DONE");
          state = IDLE;
        }
      }
      return;
    }
  }

  // 4) IDLE: read next command
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();

  // Homing command
  if (cmd == "HOMING") {
    manualHoming();
    Serial.println("DONE");
    return;
  }

  // Manual step
  if (cmd.startsWith("MANUAL")) {
    manualID = cmd.substring(7, cmd.indexOf(',',7)).toInt();
    manualStep = cmd.substring(cmd.indexOf(',',7)+1).toInt();
    if (manualID==1) stepper1.move(manualStep);
    if (manualID==2) stepper2.move(manualStep);
    if (manualID==4) stepper4.move(manualStep);
    state = MANUAL_STEP;
    return;
  }

  // Trajectory command
  if (cmd.startsWith("1,")) {
    // parse into bufData
    String s = cmd;
    for (int i=0; i<10; i++) {
      int idx = s.indexOf(',');
      bufData[i] = s.substring(0, idx).toInt();
      s = s.substring(idx+1);
    }
    trajMode  = bufData[1];
    tarPos[0] = bufData[2] * TH1;
    tarPos[1] = bufData[3] * TH2;
    tarPos[2] = bufData[4] * PHI;
    tarPos[3] = bufData[5] * ZSTEP;
    gripVal   = bufData[6];
    speedVal  = bufData[7];
    accelVal  = bufData[8];

    // configure steppers
    stepper1.setSpeed(speedVal);    stepper1.setAcceleration(accelVal);
    stepper2.setSpeed(speedVal);    stepper2.setAcceleration(accelVal);
    stepper3.setSpeed(speedVal);    stepper3.setAcceleration(accelVal);
    stepper4.setSpeed(speedVal);    stepper4.setAcceleration(accelVal);

    // initialize phases
    trajPhase = PHASE1;

    // start first phase
    if (trajMode == 1) {
      stepper1.moveTo(tarPos[0]);
      stepper2.moveTo(tarPos[1]);
      stepper3.moveTo(tarPos[2]);
      stepper4.moveTo(tarPos[3]);
    }
    else if (trajMode == 2) {
      stepper4.moveTo(tarPos[3]);
    }
    else if (trajMode == 3) {
      stepper1.moveTo(tarPos[0]);
      stepper2.moveTo(tarPos[1]);
      stepper3.moveTo(tarPos[2]);
    }
    state = TRAJECTORY;
  }
}
