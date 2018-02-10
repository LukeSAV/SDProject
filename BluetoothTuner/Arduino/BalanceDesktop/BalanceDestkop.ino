/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(13, 12);
#define OUTPUT_READABLE_YAWPITCHROLL
#define TSTEP .02
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;         // [w, x, y, z]
VectorInt16 aa;       // [x, y, z]
VectorInt16 aaReal;   // [x, y, z]
VectorInt16 aaWorld;  // [x, y, z]
VectorFloat gravity;  // [x, y, z]
float euler[3];       // [psi, theta, phi]
float ypr[3];         // [yaw, pitch, roll]

MPU6050 mpu;

Servo balanceServo;

// Interupt detection routine
volatile bool mpuInterrupt = false;
void dmpDataReady(){
    mpuInterrupt = true;
}




void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  balanceServo.attach(11);
  Serial.begin(115200);
  while(!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();


  // verify connection
  Serial.println(F("testing device connections..."));
  Serial.println(mpu.testConnection()
        ? F("MPU connection successful") : F("MPU connection failed"));

  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  bluetooth.begin(9600);
}

double degrees = 1;
int kp  = 0;
int ki = 0;
double sum = 0;
int sp = 2;
unsigned long lastUpdateTime = 0;
double last_err = 0;
void loop() {
    // if programming failed, don't try to do anyting
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while(!mpuInterrupt && fifoCount < packetSize){

    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO fifoCount
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            double pitch =  ypr[1] * 180/ M_PI;
            degrees = degrees - (degrees - pitch)*.25;

        if(micros() - lastUpdateTime > 20000){
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);



            kp =  8;
            ki = .1;
            int kd = 0;

            lastUpdateTime = micros();
            //Compute the error relative to 11
            double error = 2.4 - degrees;



            sum+=error*TSTEP;

            if( sum > 500) sum = 500;
            if( sum < -100) sum = -100;

            int u = kp*(error) + ki*sum + kd * (error - last_err)/TSTEP;




            int val = 110 -u;
            if (val > 160) val = 160;
            if (val < 0) val = 15;
            balanceServo.write(val);



            Serial.print("\tAngle: ");
            Serial.print(val);
            Serial.print("\tSum: ");
            Serial.print(sum);
            Serial.print("\tDeg: ");
            Serial.print(degrees);
            Serial.println();
            last_err = error;
        }

        // blink LED to indicate activity
        blinkState = !blinkState;
    }
}
