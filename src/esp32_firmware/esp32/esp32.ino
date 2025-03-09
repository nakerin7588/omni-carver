/*
  Omni-carver

  Motor's driver board: Smile Robotics EVO24X50
*/

#include <ESP32Encoder.h>
#include "pid.h"

// LED
#define LED_PIN 2  // Define the GPIO pin for the LED

//motorA
const int outputA_A = 16;
const int outputB_A = 17;

const int InA_motorA = 33;
const int InB_motorA = 25;
const int PWM_motorA = 26;

int pwmA;
//motorB
const int outputA_B = 15;
const int outputB_B = 19;

const int InA_motorB = 13;
const int InB_motorB = 14;
const int PWM_motorB = 27;

int pwmB;
//motorC
const int outputA_C = 5;
const int outputB_C = 18;

const int InA_motorC = 23;
const int InB_motorC = 22;
const int PWM_motorC = 21;

int pwmC;

// Encoder
ESP32Encoder encoderA;
ESP32Encoder encoderB;
ESP32Encoder encoderC;

// Pulse counter [OLD, NEW]
int encoderA_cnt[2];
int encoderB_cnt[2];
int encoderC_cnt[2];

typedef struct{
  int cnt[2];     // [OLD, NEW] counter values
  int total_cnt;  // Total counter values
  float m;        // Current position in meters
  float mps;      // Current velocity in meters / secs
} encoder_values;

encoder_values encoderA_;
encoder_values encoderB_;
encoder_values encoderC_;

// PID velocity control
PID pid_motorA;
PID pid_motorB;
PID pid_motorC;

// Control loop
unsigned long last_time;
enum {
  OLD = 0,
  NEW = 1
};

float calculate_joints_state(int cnt[2], float meters){

  // Calculate position
  float meters


  // Calculate velocity


  return ;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);

  delay(1000);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderA.attachHalfQuad(outputA_A, outputB_A);   // current use
  encoderB.attachHalfQuad(outputA_B, outputB_B);  // no input
  encoderC.attachHalfQuad(outputA_C, outputB_C);  // no input

  // PID velocity control
  PID_init(&pid_motorA, 1.2, 0.9, 0);
  PID_init(&pid_motorB, 1.2, 0.9, 0);
  PID_init(&pid_motorC, 1.2, 0.9, 0);

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
        float motorA_setpoint = s1.toFloat();
        float motorB_setpoint = s2.toFloat();
        float motorC_setpoint = s3.toFloat();
     }
   }
  }
  if (millis() - last_time >= 10) {

    last_time = millis();

    encoderA.getCount();
    encoderA.clearCount();

    encoderB.getCount();
    encoderB.clearCount();

    encoderC.getCount();
    encoderC.clearCount();
    
    encoderA_m, encoderA_mps = calculate_joints_state();
    encoderB_m, encoderB_mps = 
    encoderC_m, encoderC_mps = 
  }

}
