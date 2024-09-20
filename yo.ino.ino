#include <Servo.h>

float Kp = 3.0, Ki = 0.1, Kd = 1.5;
float error = 0, P = 0, I = 0, D = 0, previous_error = 0, PID_value = 0;

int left_motor_in1 = 5, left_motor_in2 = 6, right_motor_in3 = 9, right_motor_in4 = 10;
int left_motor_speed_pin = 3;
int right_motor_speed_pin = 11;

int trigPin = 7;
int echoPin = 8;
int servoPin = 12;

int switchPin = 9;

Servo servo;

int base_speed = 150;
int distance = 0;
int threshold_distance = 20;

void setup() {
  pinMode(left_motor_in1, OUTPUT);
  pinMode(left_motor_in2, OUTPUT);
  pinMode(right_motor_in3, OUTPUT);
  pinMode(right_motor_in4, OUTPUT);
  pinMode(left_motor_speed_pin, OUTPUT);
  pinMode(right_motor_speed_pin, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(servoPin);  
  servo.write(90);
  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  if (digitalRead(switchPin) == 1) {
   
    error = (left_sensor_value * -1) + (right_sensor_value * 1);
    P = error;
    I = constrain(I + error, -100, 100);
    D = error - previous_error;
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    previous_error = error;
    int left_motor_speed = base_speed - PID_value;
    int right_motor_speed = base_speed + PID_value;
    left_motor_speed = constrain(left_motor_speed, -255, 255);
    right_motor_speed = constrain(right_motor_speed, -255, 255);
    motorControl(left_motor_speed, right_motor_speed);
    distance = getUltrasonicReading();
    if (distance < threshold_distance) {
      avoidObstacle();
    } else {
      digitalWrite(ledPin, LOW);
    }
  } else {
    motorControl(0, 0);
  }
}

void motorControl(int left_speed, int right_speed) {
  if (left_speed > 0) {
    digitalWrite(left_motor_in1, HIGH);
    digitalWrite(left_motor_in2, LOW);
  } else if (left_speed < 0) {
    digitalWrite(left_motor_in1, LOW);
    digitalWrite(left_motor_in2, HIGH);
  } else {
    digitalWrite(left_motor_in1, LOW);
    digitalWrite(left_motor_in2, LOW);
  }
  analogWrite(left_motor_speed_pin, abs(left_speed));
  if (right_speed > 0) {
    digitalWrite(right_motor_in3, HIGH);
    digitalWrite(right_motor_in4, LOW);
  } else if (right_speed < 0) {
    digitalWrite(right_motor_in3, LOW);
    digitalWrite(right_motor_in4, HIGH);
  } else {
    digitalWrite(right_motor_in3, LOW);
    digitalWrite(right_motor_in4, LOW);
  }
  analogWrite(right_motor_speed_pin, abs(right_speed));
}

int getUltrasonicReading() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  return distance;
}

void avoidObstacle() {
  motorControl(0, 0);
  delay(300);
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
  servo.write(45);
  delay(300);
  int left_distance = getUltrasonicReading();
  servo.write(135);
  delay(300);
  int right_distance = getUltrasonicReading();
  servo.write(90);
  delay(300);
  if (left_distance > right_distance) {
    motorControl(base_speed / 2, base_speed);
    delay(1000);
  } else {
    motorControl(base_speed, base_speed / 2);
    delay(1000);
  }
}
