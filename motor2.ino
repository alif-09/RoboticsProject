#include <SoftwareSerial.h>
SoftwareSerial BT(0, 1);  // Pin 0 sebagai RX dan pin 1 sebagai TX untuk komunikasi HC-05

// Pin motor
int EN1 = 5;  // PWM motor 1
int EN2 = 6;  // PWM motor 2
int IN1 = 4;  // kontrol arah motor 1
int IN2 = 7;  // kontrol arah motor 2

int speed = 255;  // Kecepatan default maksimum

void setup() {
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(13, OUTPUT);

  BT.begin(9600);
  BT.println("Hello from Arduino - Motor Control");
}

char command;
void loop() {
  if (BT.available()) {
    command = BT.read();

    if (command == 'F') {
      // Maju dengan kecepatan penuh
      speed = 255;
      forward(speed);
      BT.println("Motor maju dengan kecepatan penuh");
    } else if (command == 'f') {
      // Maju dengan kecepatan pelan
      speed = 200;
      forward(speed);
      BT.println("Motor maju dengan kecepatan pelan");
    } else if (command == 'L') {
      // Belok kiri dengan smooth
      smoothLeft();
      BT.println("Motor belok kiri smooth");
    } else if (command == 'l') {
      // Belok kiri dengan pelan
      analogWrite(EN1, 0);     // Motor kiri berhenti
      analogWrite(EN2, 200);   // Motor kanan bergerak pelan
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      BT.println("Motor belok kiri pelan");
    } else if (command == 'R') {
      // Belok kanan dengan smooth
      smoothRight();
      BT.println("Motor belok kanan smooth");
    } else if (command == 'r') {
      // Belok kanan dengan pelan
      analogWrite(EN1, 200);   // Motor kiri bergerak pelan
      analogWrite(EN2, 0);     // Motor kanan berhenti
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      BT.println("Motor belok kanan pelan");
    } else if (command == 'S') {
      // Berhenti
      stopMotors();
      BT.println("Motor berhenti");
    } else if (command == 'B') {
      // Mundur
      backward(255);
      BT.println("Motor mundur");
    } else if (command == '?') {
      // Informasi perintah
      BT.println("Send 'F' for forward full speed");
      BT.println("Send 'f' for forward slow");
      BT.println("Send 'L' for smooth left");
      BT.println("Send 'R' for smooth right");
      BT.println("Send 'S' for stop");
      BT.println("Send 'B' for backward");
    }
  }
}

void forward(int spd) {
  analogWrite(EN1, spd);
  analogWrite(EN2, spd);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void backward(int spd) {
  analogWrite(EN1, spd);
  analogWrite(EN2, spd);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void stopMotors() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void smoothLeft() {
  analogWrite(EN1, 100);   // Motor kiri bergerak pelan
  analogWrite(EN2, 255);   // Motor kanan bergerak penuh
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void smoothRight() {
  analogWrite(EN1, 255);   // Motor kiri bergerak penuh
  analogWrite(EN2, 100);   // Motor kanan bergerak pelan
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}
