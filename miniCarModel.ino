#include "src/AccelStepper/AccelStepper.h"

#define SERIAL_RX_BUFFER_SIZE (128)
#define SERIAL_RX_TIMEOUT_MS (3)  //ms Timeout
#define DRIVER_L_PLS_PIN (3)      //Left Driver Pulse Pin
#define DRIVER_L_DIR_PIN (4)      //Left Driver Direction Pin
#define DRIVER_R_PLS_PIN (5)      //Right Driver Pulse Pin
#define DRIVER_R_DIR_PIN (6)      //Right Driver Direction Pin
#define DRIVER_PPR (500)          //Pulse Per Round (Resolution)
#define WHEEL_RADIUS (50)         //Wheel radius in mm
#define AXLE_LENGTH (210)         //Axle effective length in mm

//Car Model Variables
const float wheel_circumference = 2 * WHEEL_RADIUS * M_PI;  //Wheel's Circumference in mm
int16_t V  = 0; //Car Linear Velocity (mm/s)
int16_t W  = 0; //Car Angular Velocity (10000*rad/s)
float Vl = 0;   //Left Wheel Velocity (mm/s)
float Vr = 0;   //Right Wheel Velocity (mm/s)

//Motor Variables
AccelStepper motorL(AccelStepper::DRIVER, DRIVER_L_PLS_PIN, DRIVER_L_DIR_PIN);
AccelStepper motorR(AccelStepper::DRIVER, DRIVER_R_PLS_PIN, DRIVER_R_DIR_PIN);

//Serial3 Variables
byte rx_buffer[SERIAL_RX_BUFFER_SIZE] = {0};
uint16_t rx_index = 0;
uint32_t lastRxTime = 0;

IntervalTimer timer1;
IntervalTimer timer2;

float x = 0;
float y = 0;
float angle = 0;
int32_t prevPosL = 0;
int32_t prevPosR = 0;

void timer1_ISR() {
  int32_t currentPosL = motorL.currentPosition();
  int32_t currentPosR = motorR.currentPosition();

  float Dl = 2.0 * M_PI * WHEEL_RADIUS * (currentPosL - prevPosL) / DRIVER_PPR;
  float Dr = 2.0 * M_PI * WHEEL_RADIUS * (currentPosR - prevPosR) / DRIVER_PPR;
  float Dc = (Dl + Dr) / 2;
  x += Dc * cos(angle);
  y += Dc * sin(angle);
  angle += (Dl - Dr) / AXLE_LENGTH;
  prevPosL = currentPosL;
  prevPosR = currentPosR;
}

void timer2_ISR() { 
  Serial.print(motorL.currentPosition());
  Serial.print("\t");
  Serial.println(motorR.currentPosition());

  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" angle: ");
  Serial.println((angle/M_PI*180));

}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial3.begin(115200);
  Serial.begin(115200);
  motorL.setMaxSpeed(5000);
  motorR.setMaxSpeed(5000);
  timer1.begin(timer1_ISR, 1000);
  timer2.begin(timer2_ISR, 100000);
}

void loop() {
  // put your main code here, to run repeatedly:
  motorL.runSpeed();
  motorR.runSpeed();

  handleReceivedData();
}

void serialEvent3() {
  while (Serial3.available()) {
    byte inByte = (byte)Serial3.read();
    rx_buffer[rx_index++] = inByte;
    lastRxTime = millis();
  }
}

void serialEvent() {
  while (Serial.available()) {
    Serial.read();
    float speedL = motorL.speed();
    float speedR = motorR.speed();
    motorL.setCurrentPosition(0);
    motorR.setCurrentPosition(0);
    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);
    x = 0;
    y = 0;
    angle = 0;
    prevPosL = 0;
    prevPosR = 0;
  }
}


void handleReceivedData() {
  if (((millis() - lastRxTime) > SERIAL_RX_TIMEOUT_MS) && (rx_index > 0)) {
    if ((rx_index == 6) && (rx_buffer[0] == 'S') && (rx_buffer[5] == 'E')) {
      Serial3.println("Right!");
      V = (rx_buffer[1] << 8) | rx_buffer[2];
      W = (rx_buffer[3] << 8) | rx_buffer[4];
      computerWheelVelocity();
    }
    rx_index = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void computerWheelVelocity() {
  Vl = V - ( ( (W / 10000.0) * AXLE_LENGTH ) / 2 );
  Vr = V + ( ( (W / 10000.0) * AXLE_LENGTH ) / 2 );
  Serial3.print("Vl: ");
  Serial3.print(Vl);
  Serial3.print(" Vr: ");
  Serial3.println(Vr);
  motorL.setSpeed(convertToPPS(Vl));
  motorR.setSpeed(convertToPPS(Vr));
}

//Coverts
float convertToPPS(float v) {
  return (DRIVER_PPR / wheel_circumference * v);
}
