int EN1 = 5;  //  PWM motor 1
int EN2 = 6;  //  PWM motor 2
int IN1 = 4;  //  kontrol arah motor 1
int IN2 = 7;  //  kontrol arah motor 2
char command;

void setup() {
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.begin(9600); 
}

// Fungsi untuk bergerak maju
void moveForward() {
  analogWrite(EN1, 255); 
  analogWrite(EN2, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// Fungsi untuk bergerak mundur
void moveBackward() {
  analogWrite(EN1, 255);
  analogWrite(EN2, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

// Fungsi untuk berbelok ke kiri
void turnLeft() {
   analogWrite(EN1, 0);    
  analogWrite(EN2, 255);  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// Fungsi untuk berbelok ke kanan
void turnRight() {
   analogWrite(EN1, 255);  
  analogWrite(EN2, 0);   
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// Fungsi untuk menendang
void kick() {
  Serial.println("Kicking!");
}

// Fungsi untuk menghentikan motor
void stopMotors() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
}

void loop() {
  // Cek apakah ada data yang masuk dari serial
  if (Serial.available() > 0) {
    command = Serial.read();  

    // Memilih tindakan berdasarkan perintah yang diterima
    switch (command) {
      case 'F':
        moveForward();
        break;
      case 'L':
        turnLeft();
        break;
      case 'R':
        turnRight();
        break;
      case 'T':
        kick();
        break;
      case 'S':  
        stopMotors();
        break;
      default:
        // Jika perintah tidak dikenali, hentikan motor
        stopMotors();
        break;
    }
  }
}
