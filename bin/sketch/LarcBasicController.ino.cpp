#line 1 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
#include <Arduino.h>
#include <math.h>

#define motors 4
bool CHECK_MOTORS = false;
bool CHECK_ENCODERS = false;
bool CHECK_LINES = false;
bool CHECK_BNO = false;

class PID
{

private:
  float kp, kd, ki, umax;
  float eprev, eintegral;

public:
  PID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn)
  {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
  }
  void evalu(int value, int target, float deltaT, int &pwr, int &dir)
  {
    int e = target - value;
    float dedt = (e - eprev) / deltaT;
    eintegral = eintegral + e * deltaT;
    float u = kp * e + kd * dedt + ki * eintegral;
    pwr = (int)fabs(u);
    if (pwr > umax)
    {
      pwr = umax;
    }
    dir = 1;
    if (u < 0)
      dir = -1;

    eprev = e;
  }
};

// START: BNO
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (100)

#line 53 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void logAngles();
#line 68 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void initBNO();
#line 124 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void initMotor();
#line 160 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void moveMotor(int id, int direction, int pwm);
#line 204 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void moveAll(int direction, int pwm);
#line 230 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void frontLeftEncoderCallback();
#line 235 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void frontRightEncoderCallback();
#line 240 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void backLeftEncoderCallback();
#line 245 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void backRightEncoderCallback();
#line 250 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void initEncoders();
#line 258 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void resetEncoders();
#line 275 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void logFrontLines();
#line 285 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void logBackLines();
#line 295 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void logLeftLines();
#line 305 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void logRightLines();
#line 322 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
template <int j>void readEncoder();
#line 332 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void setup();
#line 359 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void setTarget(int dir, float deltaT);
#line 389 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
#line 407 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void loop();
#line 550 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void forward();
#line 558 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void backward();
#line 566 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void right();
#line 574 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void left();
#line 53 "c:\\Users\\angel\\OneDrive\\Escritorio\\LarcBasicController\\LarcBasicController.ino"
void logAngles()
{
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
void initBNO()
{
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}
// END: BNO

// START: MOTORS
// ROBOT 1
// Commentar al cambiar de robot
// FRONT LEFT PINOUT
static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {49, 48};
static constexpr uint8_t kAnalogPinFrontLeftMotor = 7;
static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {2, 5};
// BACK LEFT PINOUT
static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {50, 51};
static constexpr uint8_t kAnalogPinBackLeftMotor = 6;
static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {3, 4};
// FRONT RIGHT PINOUT
static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {47, 46};
static constexpr uint8_t kAnalogPinFrontRightMotor = 8;
static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {18, 17};
// BACK RIGHT PINOUT
static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {44, 45};
static constexpr uint8_t kAnalogPinBackRightMotor = 9;
static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {19, 16};

const int encb[4] = {kEncoderPinsFrontLeftMotor[1], kEncoderPinsBackLeftMotor[1], kEncoderPinsFrontRightMotor[1], kEncoderPinsBackRightMotor[1]};

// ROBOT 2
// Comentar al cambiar de robot
//  FRONT LEFT PINOUT
/*
static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {49, 48};
static constexpr uint8_t kAnalogPinFrontLeftMotor = 7;
static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {2, 5};
// BACK LEFT PINOUT
static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {32, 33};
static constexpr uint8_t kAnalogPinBackLeftMotor = 6;
static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {3, 4};
// FRONT RIGHT PINOUT
static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {47, 46};
static constexpr uint8_t kAnalogPinFrontRightMotor = 8;
static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {18, 17};
// BACK RIGHT PINOUT
static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {43, 42};
static constexpr uint8_t kAnalogPinBackRightMotor = 9;
static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {19, 16};
*/

void initMotor()
{
  pinMode(kDigitalPinsFrontLeftMotor[0], OUTPUT);
  pinMode(kDigitalPinsFrontLeftMotor[1], OUTPUT);
  pinMode(kAnalogPinFrontLeftMotor, OUTPUT);
  pinMode(kEncoderPinsFrontLeftMotor[0], INPUT);
  pinMode(kEncoderPinsFrontLeftMotor[1], INPUT);
  pinMode(kDigitalPinsBackLeftMotor[0], OUTPUT);
  pinMode(kDigitalPinsBackLeftMotor[1], OUTPUT);
  pinMode(kAnalogPinBackLeftMotor, OUTPUT);
  pinMode(kEncoderPinsBackLeftMotor[0], INPUT);
  pinMode(kEncoderPinsBackLeftMotor[1], INPUT);
  pinMode(kDigitalPinsFrontRightMotor[0], OUTPUT);
  pinMode(kDigitalPinsFrontRightMotor[1], OUTPUT);
  pinMode(kAnalogPinFrontRightMotor, OUTPUT);
  pinMode(kEncoderPinsFrontRightMotor[0], INPUT);
  pinMode(kEncoderPinsFrontRightMotor[1], INPUT);
  pinMode(kDigitalPinsBackRightMotor[0], OUTPUT);
  pinMode(kDigitalPinsBackRightMotor[1], OUTPUT);
  pinMode(kAnalogPinBackRightMotor, OUTPUT);
  pinMode(kEncoderPinsBackRightMotor[0], INPUT);
  pinMode(kEncoderPinsBackRightMotor[1], INPUT);
}

// ID: 0 = Back Left, 1 = Front Left, 2 = Back Right, 3 = Front Right
// DIRECTION: -1 = Backward, 0 = Stop, 1 = Forward
// PWM: 0-255

#define BACK_LEFT 0
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define FRONT_RIGHT 3
#define STOP 0
#define FORWARD 1
#define BACKWARD -1

void moveMotor(int id, int direction, int pwm)
{
  uint8_t digital_pins[2];
  uint8_t analog_pin;
  switch (id)
  {
  case BACK_LEFT:
    digital_pins[0] = kDigitalPinsBackLeftMotor[0];
    digital_pins[1] = kDigitalPinsBackLeftMotor[1];
    analog_pin = kAnalogPinBackLeftMotor;
    break;
  case FRONT_LEFT:
    digital_pins[0] = kDigitalPinsFrontLeftMotor[0];
    digital_pins[1] = kDigitalPinsFrontLeftMotor[1];
    analog_pin = kAnalogPinFrontLeftMotor;
    break;
  case BACK_RIGHT:
    digital_pins[0] = kDigitalPinsBackRightMotor[0];
    digital_pins[1] = kDigitalPinsBackRightMotor[1];
    analog_pin = kAnalogPinBackRightMotor;
    break;
  case FRONT_RIGHT:
    digital_pins[0] = kDigitalPinsFrontRightMotor[0];
    digital_pins[1] = kDigitalPinsFrontRightMotor[1];
    analog_pin = kAnalogPinFrontRightMotor;
    break;
  }
  if (direction == FORWARD)
  {
    digitalWrite(digital_pins[0], HIGH);
    digitalWrite(digital_pins[1], LOW);
  }
  else if (direction == BACKWARD)
  {
    digitalWrite(digital_pins[0], LOW);
    digitalWrite(digital_pins[1], HIGH);
  }
  else
  {
    digitalWrite(digital_pins[0], LOW);
    digitalWrite(digital_pins[1], LOW);
  }
  analogWrite(analog_pin, pwm);
}
void moveAll(int direction, int pwm)
{
  moveMotor(FRONT_LEFT, direction, pwm);
  moveMotor(FRONT_RIGHT, direction, pwm);
  moveMotor(BACK_LEFT, direction, pwm);
  moveMotor(BACK_RIGHT, direction, pwm);
}
// END: MOTORS

// START: ENCODERS
static constexpr double kPulsesPerRevolution = 300.0;
const int front_left_dir_sign = 1;
int front_left_encoder = 0;
int front_left_dir = 0;

const int front_right_dir_sign = 1;
int front_right_encoder = 0;
int front_right_dir = 0;

const int back_left_dir_sign = 1;
int back_left_encoder = 0;
int back_left_dir = 0;

const int back_right_dir_sign = -1;
int back_right_encoder = 0;
int back_right_dir = 0;
void frontLeftEncoderCallback()
{
  front_left_dir = (int)(digitalRead(kEncoderPinsFrontLeftMotor[1]) == HIGH ? front_left_dir_sign : front_left_dir_sign * -1);
  front_left_encoder++;
}
void frontRightEncoderCallback()
{
  front_right_dir = (int)(digitalRead(kEncoderPinsFrontRightMotor[1]) == HIGH ? front_right_dir_sign : front_right_dir_sign * -1);
  front_right_encoder++;
}
void backLeftEncoderCallback()
{
  back_left_dir = (int)(digitalRead(kEncoderPinsBackLeftMotor[1]) == HIGH ? back_left_dir_sign : back_left_dir_sign * -1);
  back_left_encoder++;
}
void backRightEncoderCallback()
{
  back_right_dir = (int)(digitalRead(kEncoderPinsBackRightMotor[1]) == HIGH ? back_right_dir_sign : back_right_dir_sign * -1);
  back_right_encoder++;
}
void initEncoders()
{
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsFrontLeftMotor[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsFrontRightMotor[0]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsBackLeftMotor[0]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsBackRightMotor[0]), readEncoder<3>, RISING);
}

void resetEncoders()
{
  front_left_encoder = 0;
  front_right_encoder = 0;
  back_left_encoder = 0;
  back_right_encoder = 0;
}

// END: ENCODERS

// START: LINES
// Order: Middle, Front, Back
static constexpr uint8_t kLineFrontLines[3] = {A7, A8, A9};
static constexpr uint8_t kLineBackLines[3] = {A4, A5, A6};
static constexpr uint8_t kLineLeftLines[3] = {A13, A14, A15};
static constexpr uint8_t kLineRightLines[3] = {A10, A11, A12};
// Get values to string
void logFrontLines()
{
  Serial.print("Front: ");
  Serial.print(analogRead(kLineFrontLines[0]));
  Serial.print(" ");
  Serial.print(analogRead(kLineFrontLines[1]));
  Serial.print(" ");
  Serial.print(analogRead(kLineFrontLines[2]));
  Serial.println(" ");
}
void logBackLines()
{
  Serial.print("Back: ");
  Serial.print(analogRead(kLineBackLines[0]));
  Serial.print(" ");
  Serial.print(analogRead(kLineBackLines[1]));
  Serial.print(" ");
  Serial.print(analogRead(kLineBackLines[2]));
  Serial.println(" ");
}
void logLeftLines()
{
  Serial.print("Left: ");
  Serial.print(analogRead(kLineLeftLines[0]));
  Serial.print(" ");
  Serial.print(analogRead(kLineLeftLines[1]));
  Serial.print(" ");
  Serial.print(analogRead(kLineLeftLines[2]));
  Serial.println(" ");
}
void logRightLines()
{
  Serial.print("Right: ");
  Serial.print(analogRead(kLineRightLines[0]));
  Serial.print(" ");
  Serial.print(analogRead(kLineRightLines[1]));
  Serial.print(" ");
  Serial.print(analogRead(kLineRightLines[2]));
  Serial.println(" ");
}
// END: LINES
int posPrev[] = {0, 0};
long prevT = 0;

volatile int posi[] = {0, 0, 0, 0};

PID pid[motors];

template <int j>
void readEncoder()
{
  int b = digitalRead(encb[j]);
  if (b > 0)
    posi[j]++;
  else
    posi[j]--;
}
void setup()
{

  Serial.begin(115200);
  while (!Serial)
  delay(10); // wait for serial port to open!
  initBNO();
  initMotor();
  initEncoders();
  for (int i = 0; i < 4; i++)
  {
    pid[i].setParams(4, 0.02, .02, 255);
  }
}

//---------------PID VARIABLES----------------//

// Used if ROS disabled.


int pwm[4] = {kAnalogPinFrontLeftMotor, kAnalogPinBackLeftMotor, kAnalogPinFrontRightMotor, kAnalogPinBackRightMotor};
uint8_t in1[4] = {kDigitalPinsFrontLeftMotor[0], kDigitalPinsBackLeftMotor[0], kDigitalPinsFrontRightMotor[0], kDigitalPinsBackRightMotor[0]};
uint8_t in2[4] = {kDigitalPinsFrontLeftMotor[1], kDigitalPinsBackLeftMotor[1], kDigitalPinsFrontRightMotor[1], kDigitalPinsBackRightMotor[1]};

float targets[] = {0.0, 0.0, 0.0, 0.0};
long target[] = {0, 0, 0, 0};

void setTarget(int dir, float deltaT)
{
  float positionChange[4] = {0.0, 0.0, 0.0, 0.0};
  float pulsesPerTurn = 300 * 15;
  float pulsesPerMeter = pulsesPerTurn * 5.6;
  float velocity = 1;  
  if(dir == 1){
    for (int i = 0; i < 4; i++)
    {
      positionChange[i] = velocity * deltaT * pulsesPerMeter;
    }
  
}
  else if (dir == -1)
  {
    for (int i = 0; i < 4; i++)
    {
      positionChange[i] = -velocity * deltaT * pulsesPerMeter;
    }
  }
  
  for (int i = 0; i < 4; i++)
  {
    targets[i] = targets[i] + positionChange[i];
  }

}

// Globals

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm, pwmVal);
  if (dir == 1)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1)
  {
    digitalWrite(in1, LOW),
    digitalWrite(in2, HIGH);
  }
  else
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

void loop()

{
  // int target[motors];
  // time difference

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  setTarget(1, deltaT);

  long pos[motors];
  for (int i = 0; i < motors; i++)
  {
    pos[i] = posi[i];
  }

  // LOOP TO THE MOTORS

  for (int i = 0; i < motors; i++)
  {
    int pwr, dir;
    pid[i].evalu(pos[i], targets[i], deltaT, pwr, dir);
    setMotor(dir, pwr, pwm[i], in1[i], in2[i]);
  }

  for (int i = 0; i < motors; i++)
  {
    Serial.print(target[i]);
    Serial.print(" ");
  }
  for (int i = 0; i < 4; i++)
  {
    Serial.print(pos[i]);
    Serial.print(" ");
  }
    Serial.println();

  // TERMINA IMPLMENTACION PID//

  if (CHECK_ENCODERS)
  {
    bool direction = false;
    long long start_time = millis();
    while (1)
    {
      if (millis() - start_time > 3 * 1000)
      {
        start_time = millis();
        direction = !direction;
        Serial.println("Direction Changed");
      }
      if (direction)
      {
        moveMotor(FRONT_LEFT, FORWARD, 150);
        moveMotor(BACK_LEFT, FORWARD, 150);
        moveMotor(FRONT_RIGHT, FORWARD, 150);
        moveMotor(BACK_RIGHT, FORWARD, 150);
      }
      else
      {
        moveMotor(FRONT_LEFT, BACKWARD, 150);
        moveMotor(BACK_LEFT, BACKWARD, 150);
        moveMotor(FRONT_RIGHT, BACKWARD, 150);
        moveMotor(BACK_RIGHT, BACKWARD, 150);
      }

      Serial.print("BL ");
      Serial.print((back_left_encoder / kPulsesPerRevolution) * 60);
      Serial.print(" ");
      Serial.println(back_left_dir);
      Serial.print("FL ");
      Serial.print((front_left_encoder / kPulsesPerRevolution) * 60);
      Serial.print(" ");
      Serial.println(front_left_dir);
      Serial.print("BR ");
      Serial.print((back_right_encoder / kPulsesPerRevolution) * 60);
      Serial.print(" ");
      Serial.println(back_right_dir);
      Serial.print("FR ");
      Serial.print((front_right_encoder / kPulsesPerRevolution) * 60);
      Serial.print(" ");
      Serial.println(front_right_dir);
      Serial.println("\n");
    }
  }

  if (CHECK_MOTORS)
  {
    // Check Motors
    bool direction = false;
    long long start_time = millis();
    while (1)
    {
      if (direction)
      {
        moveMotor(FRONT_LEFT, FORWARD, 200);
        moveMotor(BACK_LEFT, FORWARD, 200);
        moveMotor(FRONT_RIGHT, FORWARD, 200);
        moveMotor(BACK_RIGHT, FORWARD, 200);
      }
      else
      {
        moveMotor(FRONT_LEFT, BACKWARD, 200);
        moveMotor(BACK_LEFT, BACKWARD, 200);
        moveMotor(FRONT_RIGHT, BACKWARD, 200);
        moveMotor(BACK_RIGHT, BACKWARD, 200);
      }
      delay(1000);
      moveMotor(FRONT_LEFT, STOP, 200);
      moveMotor(BACK_LEFT, STOP, 200);
      moveMotor(FRONT_RIGHT, STOP, 200);
      moveMotor(BACK_RIGHT, STOP, 200);
      delay(1000);
      direction = !direction;
    }
  }

  if (CHECK_BNO)
  {
    while (1)
    {
      logAngles();
    }
  }
  if (CHECK_LINES)
  {
    while (1)
    {
      logFrontLines();
      logBackLines();
      logLeftLines();
      logRightLines();
      Serial.println();
      delay(1000);
    }
  }

  // Apply PID output to motor speeds
}

// Movimiento de motores

void forward()
{
  moveMotor(FRONT_LEFT, FORWARD, 100);
  moveMotor(BACK_LEFT, FORWARD, 100);
  moveMotor(FRONT_RIGHT, FORWARD, 100);
  moveMotor(BACK_RIGHT, FORWARD, 100);
}

void backward()
{
  moveMotor(FRONT_LEFT, BACKWARD, 200);
  moveMotor(BACK_LEFT, BACKWARD, 200);
  moveMotor(FRONT_RIGHT, BACKWARD, 200);
  moveMotor(BACK_RIGHT, BACKWARD, 200);
}

void right()
{
  moveMotor(FRONT_LEFT, FORWARD, 200);
  moveMotor(BACK_LEFT, BACKWARD, 200);
  moveMotor(FRONT_RIGHT, BACKWARD, 200);
  moveMotor(BACK_RIGHT, FORWARD, 200);
}

void left()
{
  moveMotor(FRONT_LEFT, BACKWARD, 200);
  moveMotor(BACK_LEFT, FORWARD, 200);
  moveMotor(FRONT_RIGHT, FORWARD, 200);
  moveMotor(BACK_RIGHT, BACKWARD, 200);
}

