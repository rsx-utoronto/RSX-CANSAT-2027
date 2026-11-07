
#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

// ===== Servo configuration =====
const int SERVO_PIN = 18;  // Example GPIO; confirm your wiring
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

// Keep correction within this range for the first tests
const float MAX_CORRECTION = 20.0f;

// Two camera positions, 90 degrees apart.
// These are servo angles, not guaranteed physical camera angles.
const float CAMERA_POSITION_A = 45.0f;
const float CAMERA_POSITION_B = 135.0f;

Servo cameraServo;

// ===== PID gains =====
// Starter values for software testing only.
// These are NOT tuned for your real camera.
float Kp = 1.0f;
float Ki = 0.0f;
float Kd = 0.05f;

float integral = 0.0f;
float previousError = 0.0f;
unsigned long previousTime = 0;

// ===== Camera state =====
float simulatedRoll = 0.0f;
float cameraBaseAngle = CAMERA_POSITION_A;
float pidCorrection = 0.0f;

// Calculate PID correction from measured roll.
// This function currently uses simulated data.
float computePID(float roll) {
    unsigned long now = millis();

    float dt = 0.0f;
    if (previousTime != 0) {
        dt = (now - previousTime) / 1000.0f;
    }
    previousTime = now;

    // Desired roll is 0 degrees.
    float error = 0.0f - roll;

    float derivative = 0.0f;

    if (dt > 0.0f) {
        integral += error * dt;
        derivative = (error - previousError) / dt;
    }

    previousError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    // Limit the correction
    output = constrain(
        output, -MAX_CORRECTION, MAX_CORRECTION
    );

    return output;
}

// Command the servo to a requested angle.
void setCameraAngle(float angle) {
    angle = constrain(angle, SERVO_MIN, SERVO_MAX);
    cameraServo.write((int)roundf(angle));
}

// Select one of two camera positions.
void selectCameraPosition(bool positionB) {
    cameraBaseAngle =
        positionB ? CAMERA_POSITION_B : CAMERA_POSITION_A;

    Serial.print("Selected camera position: ");
    Serial.println(positionB ? "B (90-degree change)"
                             : "A (starting position)");
}

// Run one stabilization update.
void stabilizeCamera(float roll) {
    pidCorrection = computePID(roll);

    // Combine the selected camera position with the correction.
    float servoAngle = cameraBaseAngle + pidCorrection;

    setCameraAngle(servoAngle);

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" deg | PID correction: ");
    Serial.print(pidCorrection);
    Serial.print(" deg | Servo command: ");
    Serial.print(servoAngle);
    Serial.println(" deg");
}

void printHelp() {
    Serial.println();
    Serial.println("===== Camera Control Test =====");
    Serial.println("a      : Select camera position A");
    Serial.println("b      : Select camera position B");
    Serial.println("r<number> : Set simulated roll");
    Serial.println("          Example: r10 or r-5");
    Serial.println("h      : Show commands");
    Serial.println();
}

void setup() {
    Serial.begin(115200);

    cameraServo.setPeriodHertz(50);
    cameraServo.attach(SERVO_PIN, 500, 2400);

    setCameraAngle(cameraBaseAngle);

    Serial.println("Camera control test started.");
    Serial.println("IMU is not connected in this test.");
    printHelp();
}

void loop() {
    // Read test commands from the Serial Monitor.
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "a") {
            selectCameraPosition(false);
        }
        else if (command == "b") {
            selectCameraPosition(true);
        }
        else if (command == "h") {
            printHelp();
        }
        else if (command.startsWith("r")) {
            simulatedRoll = command.substring(1).toFloat();

            Serial.print("Simulated roll set to ");
            Serial.print(simulatedRoll);
            Serial.println(" degrees.");
        }
        else if (command.length() > 0) {
            Serial.println("Unknown command. Enter h for help.");
        }
    }

    // Test the control algorithm using simulated roll.
    stabilizeCamera(simulatedRoll);

    delay(20);
}
