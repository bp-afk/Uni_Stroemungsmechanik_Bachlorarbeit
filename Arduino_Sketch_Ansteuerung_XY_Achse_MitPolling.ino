#include <AccelStepper.h>

// Pin-Zuordnung anpassen!
#define STEP1_PIN 2
#define DIR1_PIN  3
#define ENA1_PIN  6
#define STEP2_PIN 4
#define DIR2_PIN  5
#define ENA2_PIN  7

AccelStepper stepperX(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);

void setup() {
  Serial.begin(115200);
  pinMode(ENA1_PIN, OUTPUT); pinMode(ENA2_PIN, OUTPUT);
  digitalWrite(ENA1_PIN, HIGH); digitalWrite(ENA2_PIN, HIGH); // Motor aus
  stepperX.setEnablePin(ENA1_PIN); stepperY.setEnablePin(ENA2_PIN);
  stepperX.setMaxSpeed(600); stepperY.setMaxSpeed(600);
  stepperX.setAcceleration(200); stepperY.setAcceleration(200);
}

void loop() {
  stepperX.run();
  stepperY.run();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); input.toUpperCase();
    parseCmd(input);
  }
}

void parseCmd(String cmd) {
  if (cmd.startsWith("GOTO X ")) {
    stepperX.moveTo(cmd.substring(7).toInt());
  } else if (cmd.startsWith("GOTO Y ")) {
    stepperY.moveTo(cmd.substring(7).toInt());
  } else if (cmd.startsWith("MOVE X ")) {
    stepperX.move(cmd.substring(7).toInt());
  } else if (cmd.startsWith("MOVE Y ")) {
    stepperY.move(cmd.substring(7).toInt());
  } else if (cmd.startsWith("SPEED X ")) {
    stepperX.setMaxSpeed(cmd.substring(8).toInt());
  } else if (cmd.startsWith("SPEED Y ")) {
    stepperY.setMaxSpeed(cmd.substring(8).toInt());
  } else if (cmd.startsWith("ACCEL X ")) {
    stepperX.setAcceleration(cmd.substring(8).toInt());
  } else if (cmd.startsWith("ACCEL Y ")) {
    stepperY.setAcceleration(cmd.substring(8).toInt());
  } else if (cmd.startsWith("IDLE X")) {
    digitalWrite(ENA1_PIN, HIGH); stepperX.disableOutputs();
  } else if (cmd.startsWith("IDLE Y")) {
    digitalWrite(ENA2_PIN, HIGH); stepperY.disableOutputs();
  } else if (cmd.startsWith("ENABLE X")) {
    digitalWrite(ENA1_PIN, LOW); stepperX.enableOutputs();
  } else if (cmd.startsWith("ENABLE Y")) {
    digitalWrite(ENA2_PIN, LOW); stepperY.enableOutputs();
  } else if (cmd.startsWith("ORIGIN")) {
    stepperX.setCurrentPosition(0); stepperY.setCurrentPosition(0);
    Serial.println("POS 0 0");
  } else if (cmd.startsWith("POS?")) {
    Serial.print("POS "); Serial.print(stepperX.currentPosition());
    Serial.print(" "); Serial.println(stepperY.currentPosition());
  }
}