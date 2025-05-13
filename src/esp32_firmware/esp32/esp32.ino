/*
  Omni-carver

  Motor's driver board: Smile Robotics EVO24X50
*/  

#include <ESP32Encoder.h>
#include "pid.h"
#include "joints_state.h"

//motorA
const int outputA_A = 16;
const int outputB_A = 17;

const int InA_motorA = 25;
const int InB_motorA = 26;
const int PWM_motorA = 33;
//motorB
const int outputA_B = 15;
const int outputB_B = 19;

const int InA_motorB = 14;
const int InB_motorB = 12;
const int PWM_motorB = 27;
//motorC
const int outputA_C = 5;
const int outputB_C = 18;

const int InA_motorC = 22;
const int InB_motorC = 21;
const int PWM_motorC = 23;

// Encoder
#define cnt_per_rev 14000
ESP32Encoder encoderA;
ESP32Encoder encoderB;
ESP32Encoder encoderC;

// PID velocity control
PID pid_motorA;
PID pid_motorB;
PID pid_motorC;
int motorA_setpoint = 0;
int motorB_setpoint = 0;
int motorC_setpoint = 0;

// Control loop
uint64_t last_time = 0;
enum {
  OLD = 0,
  NEW = 1
};
#define rate 100  // Hz
float uA;         // Control law motor A
float uB;         // Control law motor B
float uC;         // Control law motor C

// Joints state
joints_state encoderA_;
joints_state encoderB_;
joints_state encoderC_;

void motor(float u, char motor) {

  float pwmMotor = (u / 24.0) * 255.0;
  uint8_t mA;
  uint8_t mB;
  uint8_t pwm;

  if (motor == 'A') {
    mA = InA_motorA;
    mB = InB_motorA;
    pwm = PWM_motorA;
  }
  if (motor == 'B') {
    mA = InA_motorB;
    mB = InB_motorB;
    pwm = PWM_motorB;
  }
  if (motor == 'C') {
    mA = InA_motorC;
    mB = InB_motorC;
    pwm = PWM_motorC;
  }

  if (pwmMotor < 0) {
    digitalWrite(mA, LOW);
    digitalWrite(mB, HIGH);
    analogWrite(pwm, abs(pwmMotor));
  } else if (pwmMotor > 0) {
    digitalWrite(mA, HIGH);
    digitalWrite(mB, LOW);
    analogWrite(pwm, pwmMotor);
  } else {
    digitalWrite(mA, LOW);
    digitalWrite(mB, LOW);
    analogWrite(pwm, 0);
  }
}

#define MAX_DATA_LEN 50  // Maximum length of the received data
#define ARRAY_SIZE 3     // Size of the array to store the data
char receivedData[50];   // Buffer to store received data
float setpointArray[3] = {0.0, 0.0, 0.0};  // Array to store parsed data

void setup() {
  Serial.begin(115200);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderA.attachFullQuad(outputB_A, outputA_A);  // current use
  encoderB.attachFullQuad(outputB_B, outputA_B);  // no input
  encoderC.attachFullQuad(outputB_C, outputA_C);  // no input
  delay(100);

  // PID velocity control
  PID_init(&pid_motorA, 0.055, 0.05, 0);  // 0.015, 0.005, 0
  PID_init(&pid_motorB, 0.055, 0.05, 0);  // 0.009, 0.006, 0
  PID_init(&pid_motorC, 0.055, 0.05, 0);  // 0.015, 0.005, 0
  // Joints state
  js_init(&encoderA_, cnt_per_rev, rate);
  js_init(&encoderB_, cnt_per_rev, rate);
  js_init(&encoderC_, cnt_per_rev, rate);

  //motorA
  pinMode(outputA_A, INPUT_PULLUP);
  pinMode(outputB_A, INPUT_PULLUP);

  pinMode(InA_motorA, OUTPUT);
  pinMode(InB_motorA, OUTPUT);
  pinMode(PWM_motorA, OUTPUT);

  //motorB
  pinMode(outputA_B, INPUT_PULLUP);
  pinMode(outputB_B, INPUT_PULLUP);

  pinMode(InA_motorB, OUTPUT);
  pinMode(InB_motorB, OUTPUT);
  pinMode(PWM_motorB, OUTPUT);

  //motorC
  pinMode(outputA_C, INPUT_PULLUP);
  pinMode(outputB_C, INPUT_PULLUP);

  pinMode(InA_motorC, OUTPUT);
  pinMode(InB_motorC, OUTPUT);
  pinMode(PWM_motorC, OUTPUT);
  delay(500);

  // setpointArray[0] = 3.0;
  // setpointArray[1] = 3.0;
  // setpointArray[2] = 3.0;
}
void loop() {
  // Serial.println(millis() - last_time);
  if (Serial.available() > 0) {
    int bytesRead = Serial.readBytesUntil('\n', receivedData, MAX_DATA_LEN);
    receivedData[bytesRead] = '\0';  // Null-terminate the string
    int count = 0;
    char *ptr = strtok(receivedData, ",");
    while (ptr != NULL && count < ARRAY_SIZE) {
      // Serial.println(ptr);
      setpointArray[count] = atoi(ptr) / 1000.0;  // Convert string to integer and store in the array
      // Serial.println(setpointArray[count]);
      ptr = strtok(NULL, ",");
      count++;
    }
    // for(int i = 0; i < 3; i++){
    //   Serial.printf("%.4f \n", setpointArray[i]);
    // }
    // Serial.println();
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

    int16_t values[3] = { int16_t(encoderA_.radps * 1000), int16_t(encoderB_.radps * 1000), int16_t(encoderC_.radps * 1000) };
    Serial.write((uint8_t *)values, sizeof(values));

    float error_A = setpointArray[0] - encoderA_.radps;
    float error_B = setpointArray[1] - encoderB_.radps;
    float error_C = setpointArray[2] - encoderC_.radps;
    uA = Update_pid(&pid_motorA, error_A, 24.0, 24.0);
    uB = Update_pid(&pid_motorB, error_B, 24.0, 24.0);
    uC = Update_pid(&pid_motorC, error_C, 24.0, 24.0);

    motor(uA, 'A');
    motor(uB, 'B');
    motor(uC, 'C');

    // Serial.print(encoderA_.radps);
    // Serial.print(", ");
    // Serial.print(encoderB_.radps);
    // Serial.print(", ");
    // Serial.println(encoderC_.radps);
    // Serial.print(", ");
    // Serial.print(uA);
    // Serial.print(", ");
    // Serial.print(uB);
    // Serial.print(", ");
    // Serial.println(uC);
    
    // Serial.println(millis() - last_time);
    // Serial.print(digitalRead(outputA_C));
    // Serial.print(", ");
    // Serial.println(digitalRead(outputB_C));
  }
}
