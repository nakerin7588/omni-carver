/*
  Omni-carver

  Motor's driver board: Smile Robotics EVO24X50
*/

#include <ESP32Encoder.h>
#include "pid.h"
#include "joints_state.h"

// LED
#define LED_PIN 2  // Define the GPIO pin for the LED

//motorA
const int outputA_A = 16;
const int outputB_A = 17;

const int InA_motorA = 25;
const int InB_motorA = 33;
const int PWM_motorA = 26;
//motorB
const int outputA_B = 15;
const int outputB_B = 19;

const int InA_motorB = 14;
const int InB_motorB = 13;
const int PWM_motorB = 27;
//motorC
const int outputA_C = 5;
const int outputB_C = 18;

const int InA_motorC = 22;
const int InB_motorC = 23;
const int PWM_motorC = 21;

// Encoder
#define cnt_per_rev 7000
ESP32Encoder encoderA;
ESP32Encoder encoderB;
ESP32Encoder encoderC;

// PID velocity control
PID pid_motorA;
PID pid_motorB;
PID pid_motorC;
float motorA_setpoint = 0;
float motorB_setpoint = 0;
float motorC_setpoint = 0;

// Control loop
unsigned long last_time;
enum {
  OLD = 0,
  NEW = 1
};
#define rate 1000 // Hz
float uA; // Control law motor A
float uB; // Control law motor B
float uC; // Control law motor C

// Joints state
joints_state encoderA_;
joints_state encoderB_;
joints_state encoderC_;

void motor(float u, char motor){

  float pwmMotor = (u / 24.0) * 255.0;
  uint8_t mA;
  uint8_t mB;
  uint8_t pwm;

  if (motor == 'A'){
    mA = InA_motorA;
    mB = InB_motorA;
    pwm = PWM_motorA;
  }
  if (motor == 'B'){
    mA = InA_motorB;
    mB = InB_motorB;
    pwm = PWM_motorB;
  }
  if (motor == 'C'){
    mA = InA_motorC;
    mB = InB_motorC;
    pwm = PWM_motorC;
  }

  if(pwmMotor < 0){
    digitalWrite(mA, HIGH);
    digitalWrite(mB, LOW);
    analogWrite(pwm, abs(pwmMotor));
  }
  else if(pwmMotor > 0){
    digitalWrite(mA, LOW);
    digitalWrite(mB, HIGH);
    analogWrite(pwm, pwmMotor);
  }
  else{
    digitalWrite(mA, LOW);
    digitalWrite(mB, LOW);
    analogWrite(pwm, 0);
  }
}


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);

  delay(1000);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderA.attachHalfQuad(outputB_A, outputA_A);   // current use
  encoderB.attachHalfQuad(outputB_B, outputA_B);  // no input
  encoderC.attachHalfQuad(outputB_C, outputA_C);  // no input

  // PID velocity control
  PID_init(&pid_motorA, 0.015, 0.005, 0); // 0.015, 0.005, 0
  PID_init(&pid_motorB, 0.015, 0.005, 0); // 0.009, 0.006, 0
  PID_init(&pid_motorC, 0.015, 0.005, 0); // 0.015, 0.005, 0

  // Joints state
  js_init(&encoderA_, cnt_per_rev, rate);
  js_init(&encoderB_, cnt_per_rev, rate);
  js_init(&encoderC_, cnt_per_rev, rate);

  //motorA
  pinMode(outputA_A, INPUT);
  pinMode(outputB_A, INPUT);

  pinMode(InA_motorA, OUTPUT);
  pinMode(InB_motorA, OUTPUT);
  pinMode(PWM_motorA, OUTPUT);

  //motorB
  pinMode(outputA_B, INPUT);
  pinMode(outputB_B, INPUT);

  pinMode(InA_motorB, OUTPUT);
  pinMode(InB_motorB, OUTPUT);
  pinMode(PWM_motorB, OUTPUT);

  //motorC
  pinMode(outputA_C, INPUT);
  pinMode(outputB_C, INPUT);

  pinMode(InA_motorC, OUTPUT);
  pinMode(InB_motorC, OUTPUT);
  pinMode(PWM_motorC, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  delay(500);
}

void loop() {
  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.length() > 0) {
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);

      if (firstComma != -1 && secondComma != -1) {
        String s1 = data.substring(0, firstComma);
        String s2 = data.substring(firstComma + 1, secondComma);
        String s3 = data.substring(secondComma + 1);

        // Convert data from serial to motor setpoint
        motorA_setpoint = s1.toFloat();
        motorB_setpoint = s2.toFloat();
        motorC_setpoint = s3.toFloat();
     }
   }
  }
  if (millis() - last_time >= 10) {

    last_time = millis();

    encoderA_.diffcnt = encoderA.getCount();
    encoderA.clearCount();

    encoderB_.diffcnt = encoderB.getCount();
    encoderB.clearCount();

    encoderC_.diffcnt = encoderC.getCount();
    encoderC.clearCount();
    
    update_joints_state(&encoderA_);
    update_joints_state(&encoderB_);
    update_joints_state(&encoderC_);

    float values[] = {encoderA_.rad, encoderB_.rad, encoderC_.rad, encoderA_.radps, encoderB_.radps, encoderC_.radps};
    for (int i = 0; i < 6; i++) {
      Serial.print(values[i], 4);
      if (i < 5) {
        Serial.print(",");
      } else {
        Serial.println();  // End the line with a newline character
      }
    }

    float error_A = motorA_setpoint - encoderA_.radps;
    float error_B = motorB_setpoint - encoderB_.radps;
    float error_C = motorC_setpoint - encoderC_.radps;
    uA = Update_pid(&pid_motorA, error_A, 24.0, 24.0);
    uB = Update_pid(&pid_motorB, error_B, 24.0, 24.0);
    uC = Update_pid(&pid_motorC, error_C, 24.0, 24.0);

    // Serial.println(motorA_setpoint);
    // Serial.println(motorB_setpoint);
    // Serial.println(motorC_setpoint);

    motor(uA, 'A');
    motor(uB, 'B');
    motor(uC, 'C');
  }

}
