#include <Servo.h>
#include <TimerFour.h>
#include <TimerThree.h>

//Servo Pins
#define servo_sw 44
#define Flow_pin 9   // PWM pin for servo
Servo Flow_sw;
//Servo Pins End

//Motor Pins
#define stepPin 5
int mot_pin1 = 13;
int mot_pin2 = 12;
int enA = 4;
short speed_val = 255;
volatile bool stepState = false;
//Motor Pins End

//Bluetooth vars
String inputString = "";   
bool stringComplete = false;
int userVol;
int userSpeed;
int curVol;
//Bluetooth var ends

//Acclusion detection
const int FLOW_IR = 24;
int black_seen;
int white_seen;
int idle_time;
int max_idle_time = 50;
int black_persist_count = 0; 
int min_black_persist_count = 3;
int black_wrong_persistance_count = 0; 
int max_black_wrong = 3;
//Acclusion detection End

//Current volume detection
const int MOTOR_IR = 8; 
int mot_count_flag;
unsigned int curr_mot_state = 0; 
unsigned int rot_count = 0;     
unsigned int rot_count_copy = 0;
unsigned int IR_cut = 0;    
unsigned int IR_no_cut = 0; 
int motor_curr = 0;
int prev_motor_curr = 0;
//Current vol detection end

// Status and system flags
bool fill_active = false;
bool drain_active = false;
bool occlusion_flag = false;
int battery_percent = 80; // simulated for now

// -------------------------------------------------------
// Core Functional Functions
// -------------------------------------------------------
void startFill() {
  digitalWrite(servo_sw, HIGH);
  Flow_sw.attach(Flow_pin);
  Flow_sw.write(135);
  delay(200);
  Flow_sw.detach();
  delay(100);
  digitalWrite(servo_sw, LOW);
  digitalWrite(mot_pin1, LOW);
  digitalWrite(mot_pin2, HIGH);
  analogWrite(enA, userSpeed);
}

void stopFill() {
  digitalWrite(mot_pin1, HIGH);
  digitalWrite(mot_pin2, HIGH);
  analogWrite(enA, 0);
}

void startDrain() {
  digitalWrite(servo_sw, HIGH);
  Flow_sw.attach(Flow_pin);
  Flow_sw.write(45);
  delay(1000);
  Flow_sw.detach();
  delay(100);
  digitalWrite(servo_sw, LOW);
  digitalWrite(mot_pin1, HIGH);
  digitalWrite(mot_pin2, LOW);
  analogWrite(enA, userSpeed);
}

void stopDrain() {
  digitalWrite(mot_pin1, HIGH);
  digitalWrite(mot_pin2, HIGH);
  analogWrite(enA, 0);
}

void stericonLock() {
  digitalWrite(servo_sw, HIGH);
  Flow_sw.attach(Flow_pin);
  Flow_sw.write(90);
  delay(300);
  Flow_sw.detach();
  delay(200);
  digitalWrite(servo_sw, LOW);
}

void stericonUnlock() {
  digitalWrite(servo_sw, HIGH);
  Flow_sw.attach(Flow_pin);
  Flow_sw.write(0);
  delay(300);
  Flow_sw.detach();
  delay(200);
  digitalWrite(servo_sw, LOW);
}

void servoInit() {
  pinMode(servo_sw, OUTPUT);
  digitalWrite(servo_sw, HIGH);
  Flow_sw.attach(Flow_pin);
  Flow_sw.write(90);
  delay(300);
  Flow_sw.detach();
  delay(200);
  digitalWrite(servo_sw, LOW);
}

// -------------------------------------------------------
// Sensor Callbacks
// -------------------------------------------------------
void vaneCallback() {
  int curr = digitalRead(FLOW_IR);
  if (curr == 1) {
    //Serial1.println("BLACK");
  } else {
    //Serial1.println("WHITE");
  }
}

void motorCheck() {
  motor_curr = digitalRead(MOTOR_IR);
  if (motor_curr != prev_motor_curr) {
    if (motor_curr == HIGH) {
      Serial1.println("MOTOR_IR: HIGH");
    } else {
      Serial1.println("MOTOR_IR: LOW");
    }
    prev_motor_curr = motor_curr;
  }

  if (motor_curr == HIGH) {
    if (curr_mot_state == 1) {
      IR_cut++;
      IR_no_cut = 0;
    } else if (curr_mot_state == 0) {
      curr_mot_state = 1;
      curVol++;
      Serial1.print("Motor rotation detected, curr_wght = ");
      Serial1.println(curVol);
    }
  } else if (motor_curr == LOW) {
    if (curr_mot_state == 0) {
      IR_no_cut++;
      IR_cut = 0;
    } else if (curr_mot_state == 1) {
      curr_mot_state = 0;
    }
  }
}

void occlusion() {
  digitalWrite(mot_pin1, HIGH);
  digitalWrite(mot_pin2, HIGH);
  analogWrite(enA, 0);
  occlusion_flag = true;
  Serial1.println("Occlusion Detected");
}

// -------------------------------------------------------
// ACK Response Handling
// -------------------------------------------------------
void handleACK(String ackString) {
  ackString.trim();

  if (ackString == "$MB,ACK#") {
    Serial1.print("$DEVICE,NACK,");
    Serial1.print(battery_percent);
    Serial1.println("#");
  } else if (ackString == "$MB,F,ACK#") {
    Serial1.print("$F,RECD,");
    Serial1.print(curVol);
    Serial1.print(",");
    Serial1.print(occlusion_flag);
    Serial1.print(",");
    Serial1.print(battery_percent);
    Serial1.println("#");
  } else if (ackString == "$MB,D,ACK#") {
    Serial1.print("$D,RECD,");
    Serial1.print(curVol);
    Serial1.print(",");
    Serial1.print(occlusion_flag);
    Serial1.print(",");
    Serial1.print(battery_percent);
    Serial1.println("#");
  }
}

// -------------------------------------------------------
// Command Parsing and Bluetooth RX
// -------------------------------------------------------
void rxCommand() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();

    if (inChar == '$') {
      inputString = "";
    } else if (inChar == '#') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    inputString.trim();
    String cmd[4];
    int idx = 0;
    int startIdx = 0;

    for (int i = 0; i < inputString.length(); i++) {
      if (inputString[i] == ',') {
        if (idx < 4) {
          cmd[idx++] = inputString.substring(startIdx, i);
        }
        startIdx = i + 1;
      }
    }

    if (idx < 4) {
      cmd[idx++] = inputString.substring(startIdx);
    }

    for (int i = 0; i < idx; i++) cmd[i].trim();

    // -------- Command Parser --------
    if (cmd[0] == "F") {
      if (cmd[1] == "STRT") {
        if (idx > 2) userVol = cmd[2].toInt();
        if (idx > 3) userSpeed = cmd[3].toInt();
        startFill();
        fill_active = true;
        drain_active = false;
      } else if (cmd[1] == "STOP") {
        stopFill();
        fill_active = false;
      }
    }

    else if (cmd[0] == "D") {
      if (cmd[1] == "STRT") {
        if (idx > 2) userVol = cmd[2].toInt();
        if (idx > 3) userSpeed = cmd[3].toInt();
        startDrain();
        drain_active = true;
        fill_active = false;
      } else if (cmd[1] == "STOP") {
        stopDrain();
        drain_active = false;
      }
    }

    else if (cmd[0] == "STERICON") {
      if (cmd[1] == "LOCK") {
        stericonLock();
      } else if (cmd[1] == "UNLOCK") {
        stericonUnlock();
      }
    }

    else if (cmd[0] == "RST") {
      servoInit();
    }

    // Check for ACK Messages
    if (inputString.startsWith("MB")) {
      handleACK("$" + inputString + "#");
    }

    // Clear for next command
    inputString = "";
    stringComplete = false;
  }
}

// -------------------------------------------------------
// Setup
// -------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Omni PD working");
  Serial1.begin(9600);
  Serial1.println("Omni PD Working BT");

  //Servo Initialize to 90 degrees
  servoInit();

  //Motor Init Start
  pinMode(enA, OUTPUT);
  pinMode(mot_pin1, OUTPUT);
  pinMode(mot_pin2, OUTPUT);
  digitalWrite(mot_pin1, LOW);
  digitalWrite(mot_pin2, LOW);
  pinMode(stepPin, OUTPUT);

  //Acclusion detection
  pinMode(FLOW_IR, INPUT_PULLUP);
  Timer4.initialize(100000);
  Timer4.attachInterrupt(vaneCallback);

  //Motor Check Init Start
  pinMode(MOTOR_IR, INPUT);
  Timer3.initialize(10000);
  Timer3.attachInterrupt(motorCheck);
}

// -------------------------------------------------------
// Main Loop
// -------------------------------------------------------
void loop() {
  rxCommand();
}
