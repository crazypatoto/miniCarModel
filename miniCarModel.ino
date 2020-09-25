#include "src/AccelStepper/AccelStepper.h"

#define SERIAL_RX_BUFFER_SIZE (128)
#define SERIAL_RX_TIMEOUT_US (500) //microsecond Timeout
#define DRIVER_L_PLS_PIN (3)      //Left Driver Pulse Pin
#define DRIVER_L_DIR_PIN (4)      //Left Driver Direction Pin
#define DRIVER_R_PLS_PIN (5)      //Right Driver Pulse Pin
#define DRIVER_R_DIR_PIN (6)      //Right Driver Direction Pin
#define DRIVER_PPR (5000)          //Pulse Per Round (Resolution)
#define WHEEL_RADIUS (50)         //Wheel radius in mm
#define AXLE_LENGTH (210)         //Axle effective length in mm

//Car Model Variables
const float wheel_circumference = 2.0f * WHEEL_RADIUS * M_PI;  //Wheel's Circumference in mm
int16_t V  = 0; //Car Linear Velocity (mm/s)
int16_t W1000  = 0; //Car Angular Velocity (1000*rad/s)
float Vl = 0;   //Left Wheel Velocity (mm/s)
float Vr = 0;   //Right Wheel Velocity (mm/s)

//Motor Variables
AccelStepper motorL(AccelStepper::DRIVER, DRIVER_L_PLS_PIN, DRIVER_L_DIR_PIN);
AccelStepper motorR(AccelStepper::DRIVER, DRIVER_R_PLS_PIN, DRIVER_R_DIR_PIN);

//Serial2 Variables
byte rx_buffer[SERIAL_RX_BUFFER_SIZE] = {0};
uint16_t rx_index = 0;
uint32_t lastRxTime = 0;

IntervalTimer timer2;

//Odometry Variables
float x = 0;
float y = 0;
float angle = 0;
int32_t prevPosL = 0;
int32_t prevPosR = 0;

void calculateOdometry() {
  static uint32_t lastOdometryTime = 0;
  uint32_t now = millis();
  if ( (now - lastOdometryTime) < 10) {
    return;
  }
  lastOdometryTime = now;

  int32_t currentPosL = motorL.currentPosition();
  int32_t currentPosR = motorR.currentPosition();

  float Dl = 2.0f * M_PI * WHEEL_RADIUS * (currentPosL - prevPosL) / DRIVER_PPR;
  float Dr = 2.0f * M_PI * WHEEL_RADIUS * (currentPosR - prevPosR) / DRIVER_PPR;
  float Dc = (Dl + Dr) / 2;
  x += Dc * cosf(angle);
  y += Dc * sinf(angle);
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
  //Serial.println((atan2(sin(angle), cos(angle)) / M_PI * 180));
  Serial.println(angle);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial2.begin(115200);
  Serial.begin(115200);
  motorL.setMaxSpeed(convertToPPS(2000)); //max speed of 2m/s
  motorR.setMaxSpeed(convertToPPS(2000)); //max speed of 2m/s
  //timer2.begin(timer2_ISR, 100000);
}

void loop() {
  // put your main code here, to run repeatedly:
  noInterrupts();
  motorR.runSpeed();
  motorL.runSpeed();
  interrupts();
  handleReceivedData();
  calculateOdometry();
}

void serialEvent2() {
  while (Serial2.available()) {
    byte inByte = (byte)Serial2.read();
    rx_buffer[rx_index++] = inByte;
    lastRxTime = micros();
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

//Packet Format:
//  byte[0] 'S' Fixed character 'S'
//  byte[1]  High Byte of Linear Velocity (mm/s)
//  byte[2]  Low Byte of Linear Velocity (mm/s)
//  byte[3]  High Byte of Angular Velocity (1000*rad/s)
//  byte[4]  Low Byte of Angular Velocity (1000*rad/s)
//  byte[5] 'E' Fixed character 'E'
void handleReceivedData() {
  if (((micros() - lastRxTime) > SERIAL_RX_TIMEOUT_US) && (rx_index > 0)) {
    if ((rx_index == 6) && (rx_buffer[0] == 'S') && (rx_buffer[5] == 'E')) {
      V = (rx_buffer[1] << 8) | rx_buffer[2];
      W1000 = (rx_buffer[3] << 8) | rx_buffer[4];
      computeWheelVelocity();

    }
    else if ((rx_index == 3) && (rx_buffer[0] == 'O') && (rx_buffer[1] == 'D') && (rx_buffer[2] == 'M')) {
      byte txbuff[6] = {0};
      int16_t _x = (int16_t)x;
      int16_t _y = (int16_t)y;
      int16_t _angle = (int16_t)(atan2(sin(angle), cos(angle)) * 1000); //atan2(sin(angle),cos(angle)) keeps angle between -PI ~ PI
      //int16_t _angle = (int16_t)(angle * 1000); //atan2(sin(angle),cos(angle)) keeps angle between -PI ~ PI

      txbuff[0] = (_x >> 8) & 0xFF;
      txbuff[1] = _x & 0xFF;
      txbuff[2] = (_y >> 8) & 0xFF;
      txbuff[3] = _y & 0xFF;
      txbuff[4] = (_angle >> 8) & 0xFF;
      txbuff[5] = _angle & 0xFF;
      Serial2.write(txbuff, 6);
    } else if ((rx_index == 3) && (rx_buffer[0] == 'C') && (rx_buffer[1] == 'L') && (rx_buffer[2] == 'R')) {
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
    } else if ((rx_index == 8) && (rx_buffer[0] == 'O') && (rx_buffer[7] == 'E')) {
      int16_t _x = (int16_t)((rx_buffer[1] << 8) | rx_buffer[2]);
      int16_t _y = (int16_t)((rx_buffer[3] << 8) | rx_buffer[4]);      
      int16_t _angle = (int16_t)((rx_buffer[5] << 8) | rx_buffer[6]);

      x = (float)_x;
      y = (float)_y;
      angle = (float)_angle/1000.0;
    }
    //Serial.println(rx_index);
    rx_index = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void computeWheelVelocity() {
  Vl = V - ( ( (W1000 / 1000.0) * AXLE_LENGTH ) / 2 );
  Vr = V + ( ( (W1000 / 1000.0) * AXLE_LENGTH ) / 2 );

  motorL.setSpeed(convertToPPS(Vl));
  motorR.setSpeed(convertToPPS(Vr));
  //Serial.println(motorL.currentPosition());
  //Serial.println(motorR.currentPosition());
  //  Serial.print("Vl: ");
  //  Serial.print(Vl);
  //  Serial.print(" Vr: ");
  //  Serial.println(Vr);
}

//Coverts mm/s to pps(pulse per second)
float convertToPPS(float v) {
  return (DRIVER_PPR / wheel_circumference * v);
}
