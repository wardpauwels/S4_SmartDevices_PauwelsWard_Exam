#include <Servo.h>
#include <IRremote.h>
#include <SoftwareSerial.h>

SoftwareSerial BT (0, 1); //RX | TX
int data;

Servo servo;

const int RECV_PIN = 4;

IRrecv irrecv(RECV_PIN);

decode_results results;

const int irLed = A5;

const int maxHealth = 5;
int healthLeds[maxHealth] = {A0, A1, A2, A3, A4};
int health;
int angle;

int enablePin1 = 8;
int inputPin1 = 7;
int inputPin2 = 6;

int enablePin2 = 11;
int inputPin3 = 12;
int inputPin4 = 13;

int buttonPin = 2;

boolean death = false;
boolean servoDown = false;
boolean hasWon = false;


void setup() {
  health = maxHealth;
  servo.attach(5);
  servo.write(0);

  irrecv.enableIRIn(); // Start the receiver

  //leds
  for (int i = 0; i < health; i++) {
    pinMode(healthLeds[i], OUTPUT);
  }
  //IR
  pinMode(irLed, OUTPUT); //InfraRed led

  //wheel left
  pinMode(inputPin1, OUTPUT);
  pinMode(inputPin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);

  //wheel right
  pinMode(inputPin3, OUTPUT);
  pinMode(inputPin4, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  digitalWrite(enablePin1, 80);
  digitalWrite(enablePin2, 80);

  //button
  pinMode(buttonPin, INPUT);

  Serial.begin(9600);
  BT.begin(9600);
}


void loop() {
  if (digitalRead(buttonPin)) {
    death = true; //Safety if robot does his own thing
  }

  if (!death && !hasWon) {
    long ir_code = getCode();
    if (ir_code > 0) {
      gotHit();
      delay(50);
      irrecv.resume();
    }
    setLives();

    //Bluetooth simulation with serial line
    if (Serial.available())
    {
      delay(10);
      BT.write(Serial.read());
    }

    //Bluetooth connection
    if (BT.available()) {
      data = BT.read();
      doAction(data);
    }
  }
  else if (!death && hasWon) {
    winMove();
  }
  else {
    if (!servoDown) {
      setServo(179); //Put flag down when lost
      servoDown = true;
    }
    showDeath();
  }
}

void setLives() {
  for (int i = 0; i <= health; i++) {
    analogWrite(healthLeds[i], 0);
  }
  for (int i = 0; i < health; i++) {
    analogWrite(healthLeds[i], 130);
  }

}

void gotHit() {
  Serial.println("Hit");
  health--;
  if (health == 0) {
    death = true;
  }
}

void goForward() {
  Serial.println("forward");
  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, HIGH);
  digitalWrite(inputPin3, HIGH);
  digitalWrite(inputPin4, LOW);
}

void goBack() {
  Serial.println("back");
  digitalWrite(inputPin1, HIGH);
  digitalWrite(inputPin2, LOW);
  digitalWrite(inputPin3, LOW);
  digitalWrite(inputPin4, HIGH);
}

void goLeft() {
  Serial.println("left");
  digitalWrite(inputPin1, HIGH);
  digitalWrite(inputPin2, LOW);
  digitalWrite(inputPin3, HIGH);
  digitalWrite(inputPin4, LOW);
}

void goRight() {
  Serial.println("right");
  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, HIGH);
  digitalWrite(inputPin3, LOW);
  digitalWrite(inputPin4, HIGH);
}

void goStop() {
  Serial.println("stop");
  digitalWrite(inputPin1, HIGH);
  digitalWrite(inputPin2, HIGH);
  digitalWrite(inputPin3, HIGH);
  digitalWrite(inputPin4, HIGH);
}

void shoot() {
  Serial.println("shoot");
  analogWrite(irLed, 130);
  analogWrite(irLed, 0);
}

void setServo(int amount) {
  Serial.println("servo moved");
  servo.write(179);
}

void changeWinState() {
  hasWon = true;
}

long getCode() {
  long irCode = 0;
  if (irrecv.decode(&results)) {
    irCode = results.value;
  }
}

void showDeath() {
  for (int i = 0; i < 5; i++) {
    analogWrite(healthLeds[i], 130);
  }
  delay(500);
  for (int i = 0; i < 5; i++) {
    analogWrite(healthLeds[i], 0);
  }
  delay(500);
}

void doAction(int data) {
  switch (data)
  {
    case '1' : goForward(); break;
    case '2' : goBack(); break;
    case '3' : goLeft(); break;
    case '4' : goRight(); break;
    case '5' : shoot(); break;
    case '6' : goStop(); break;
    case '7' : changeWinState(); break;
  }
}

void winMove() {
  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, HIGH);
  digitalWrite(inputPin3, LOW);
  digitalWrite(inputPin4, HIGH);
  delay(1000);
  digitalWrite(inputPin1, HIGH);
  digitalWrite(inputPin2, LOW);
  digitalWrite(inputPin3, HIGH);
  digitalWrite(inputPin4, LOW);
  delay(1000);
}

