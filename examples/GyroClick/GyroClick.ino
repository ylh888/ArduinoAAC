// GyroClick_v1 : June 2014 : works with drawArduinoDatav2
// 0. teapot demo (standalone==0)
// 1. movement detector (standalone==1) - emits mouse click
// 2. gyro-based (standalone == 2) 


// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
 NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
 depends on the MPU-6050's INT pin being connected to the Arduino's
 external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
 digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
 NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
 when using Serial.write(buf, len). The Teapot output uses this method.
 The solution requires a modification to the Arduino USBAPI.h file, which
 is fortunately simple, but annoying. This will be fixed in the next IDE
 release. For more info, see these links:
 
 http://arduino.cc/forum/index.php/topic,109987.0.html
 http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
#define OUTPUT_TEAPOT

//ylh if under OUTPUT_TEAPOT and standAlone=1, then this version mimicks
// the "drawArduinoDatav2" processing code, which takes input from here
// with standAlone=0

#include <AACsensors.h>
#include <AACswitch.h>

const int debug = 0; // slow down
// standAlone = 0 -> teapot
//            = 1 click function by movement distance
//            = 2 click function by accel
const int standAlone = 0;  //must not be in debug if standAlone==1

Switch buzzer( 4, 40, 0 );

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

VectorInt16 gyro; //ylh

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { 
   '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//--- local Stand1 - click function ---
Quaternion oldq;
long previousGrabTime = 0;
long timeStep = 10; //low timestep for rapid processing
long refractory = 500;
long newPeriod = 0;
float cumS = 0;
long ignoreTime = 0;
long resetPeriod = 80;
float Avalue = 0, Xvalue = 0, Yvalue =0;

//--- local Stand2
long timeStep2 = 100;

void Stand1() {

   if(millis() - previousGrabTime >= timeStep)  {
      previousGrabTime = millis();
      Avalue = euclid( q, oldq );
      Xvalue += Avalue;

      if( (previousGrabTime - ignoreTime) > resetPeriod ) {
         Xvalue = 0;
         ignoreTime = previousGrabTime;
      }
      if( (millis() - newPeriod) > refractory ) {
         Yvalue = Xvalue > 0.015 ? 0.1 : 0;
         if( Yvalue == 0.1 ) {
            buzzer.On(50);
            Keyboard.press(KEY_UP_ARROW); 
            Keyboard.press(KEY_DOWN_ARROW); 
            Keyboard.releaseAll();
            //            Serial.print( newPeriod );
            //            Serial.println ( " +" );
            Xvalue = 0;
            ignoreTime = previousGrabTime;
            newPeriod = ignoreTime;
         } // else Serial.print (".");
      }

      oldq = q;
   }
}

float euclid( Quaternion o, Quaternion n ) {

   float sumsqs = 0;
   sumsqs += (o.w - n.w)*(o.w - n.w);
   sumsqs += (o.x - n.x)*(o.x - n.x);
   sumsqs += (o.y - n.y)*(o.y - n.y);
   sumsqs += (o.z - n.z)*(o.z - n.z);
   return sqrt( sumsqs);
}

VectorInt16 minima, maxima, atrest, myReal;
//Quaternion minima, maxima, atrest, myReal;
float min[3],max[3], myeuler[3];

void Stand2() {

   if(millis() - previousGrabTime >= timeStep2)  {
      previousGrabTime = millis();

      //setRealMinMax( );
      //if( myReal.x < -9 ) resetMinMax();
      //setEulerMinMax();

      setGyroMinMax();

      //setAccelMinMax();
      //setQMinMax();

      if( debug ) {
         /*
         for( int i=0; i<3; i++ ) {
          Serial.print("\t"); 
          Serial.print( euler[i] );
          }
          Serial.println();
          */
         //Serial.print(myReal.w);          Serial.print("\t");
         Serial.print(myReal.x);
         Serial.print("\t");
         Serial.print(myReal.y);
         Serial.print("\t");
         Serial.print(myReal.z);

         Serial.print("\t| ");
         //Serial.print(minima.w);Serial.print(", ");Serial.print(maxima.w);Serial.print("\t");
         Serial.print(minima.x);
         Serial.print(", ");
         Serial.print(maxima.x);
         Serial.print("\t");
         Serial.print(minima.y);
         Serial.print(", ");
         Serial.print(maxima.y);
         Serial.print("\t");
         Serial.print(minima.z);
         Serial.print(", ");
         Serial.print(maxima.z );
         Serial.println();
         
         /*
          for( int i=0; i<3; i++ ) {
          Serial.print(myeuler[i]);
          Serial.print("\t");
          }
          Serial.print("\t| ");
          for( int i=0; i<3; i++ ) {
          Serial.print(min[i]);
          Serial.print(", ");
          Serial.print(max[i]);
          Serial.print("\t");
          }
          Serial.println();
          */
      }
   }
}

void reCentre() {
   buzzer.On(50);
   delay(100);
   buzzer.Check();

}

void setEulerMinMax() {
   for( int i=0; i<3; i++ ) {
      if(  euler[i] < min[i] ) min[i] =  euler[i];
      if(  euler[i] > max[i] ) max[i] =  euler[i];
      myeuler[i] = constrain( map( (long) (100.0*euler[i]), (long) (100.0*min[i]), (long) (100.0*max[i]), -11, 11 ), -10,10);
   }
}

void setGyroMinMax() {

   if( gyro.x < minima.x ) minima.x = gyro.x;
   if( gyro.y < minima.y ) minima.y = gyro.y;
   if( gyro.z < minima.z ) minima.z = gyro.z;

   if( gyro.x > maxima.x ) maxima.x = gyro.x;
   if( gyro.y > maxima.y ) maxima.y = gyro.y;
   if( gyro.z > maxima.z ) maxima.z = gyro.z;
   /*
   myReal.x = constrain( map( gyro.x, minima.x, maxima.x, 11, -11 ), -10,10);
    myReal.y = constrain( map( gyro.y, minima.y, maxima.y, -11, 11 ), -10,10);
    myReal.z = constrain( map( gyro.z, minima.z, maxima.z,  11, -11), -10,10);
    */
   myReal.x = constrain( map( gyro.x, -80, 80, 11, -11 ), -10,10);
   myReal.y = constrain( map( gyro.y, -80, 80, -11, 11 ), -10,10);
   myReal.z = constrain( map( gyro.z, -80, 80,  11, -11), -10,10);

}
void setQMinMax() {
/*
   if( (int) 100*q.w < minima.w ) minima.w = (int) 100*q.w;
   if( (int) 100*q.x < minima.x ) minima.x = (int) 100*q.x;
   if( (int) 100*q.y < minima.y ) minima.y = (int) 100*q.y;
   if( (int) 100*q.z < minima.z ) minima.z = (int) 100*q.z;

   if( (int) 100*q.w > maxima.w ) maxima.w = (int) 100*q.w;
   if( (int) 100*q.x > maxima.x ) maxima.x = (int) 100*q.x;
   if( (int) 100*q.y > maxima.y ) maxima.y = (int) 100*q.y;
   if( (int) 100*q.z > maxima.z ) maxima.z = (int) 100*q.z;

   myReal.w = constrain( map( (int) 100*q.w, minima.w, maxima.w, -11, 11 ), -10,10);
   myReal.x = constrain( map( (int) 100*q.x, minima.x, maxima.x, -11, 11 ), -10,10);
   myReal.y = constrain( map( (int) 100*q.y, minima.y, maxima.y, -11, 11 ), -10,10);
   myReal.z = constrain( map( (int) 100*q.z, minima.z, maxima.z, -11, 11), -10,10);
*/
}

void setAccelMinMax() {

   if( aa.x < minima.x ) minima.x = aa.x;
   if( aa.y < minima.y ) minima.y = aa.y;
   if( aa.z < minima.z ) minima.z = aa.z;

   if( aa.x > maxima.x ) maxima.x = aa.x;
   if( aa.y > maxima.y ) maxima.y = aa.y;
   if( aa.z > maxima.z ) maxima.z = aa.z;

   myReal.x = constrain( map( aa.x, minima.x, maxima.x, -11, 11 ), -10,10);
   myReal.y = constrain( map( aa.y, minima.y, maxima.y, -11, 11 ), -10,10);
   myReal.z = constrain( map( aa.z, minima.z, maxima.z, -11, 11 ), -10,10);
}


void setRealMinMax() {

   if( aaReal.x < minima.x ) minima.x = aaReal.x;
   if( aaReal.y < minima.y ) minima.y = aaReal.y;
   if( aaReal.z < minima.z ) minima.z = aaReal.z;

   if( aaReal.x > maxima.x ) maxima.x = aaReal.x;
   if( aaReal.y > maxima.y ) maxima.y = aaReal.y;
   if( aaReal.z > maxima.z ) maxima.z = aaReal.z;

   myReal.x = constrain( map( aaReal.x, minima.x, maxima.x, 11, -11 ), -10,10);
   myReal.y = constrain( map( aaReal.y, minima.y, maxima.y, -11, 11 ), -10,10);
   myReal.z = constrain( map( aaReal.z, minima.z, maxima.z,  11, -11), -10,10);
}

void resetMinMax() {
   buzzer.On(50); 
   delay(100); 
   buzzer.Check();
   delay(500);

   //minima.w=
   minima.x=minima.y=minima.z=2000;
   //maxima.w=
   maxima.x=maxima.y=maxima.z=-2000;
   min[0]=min[1]=min[2]=2000;
   max[0]=max[1]=max[2]=-2000;
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
   mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

   //ylh
   buzzer.On(100);
   delay(150);
   buzzer.Check();
   delay(150);
   buzzer.On(50);
   delay(100);
   buzzer.Check();

   resetMinMax();

   // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
   Wire.begin();
   TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
   Fastwire::setup(400, true);
#endif

   // initialize serial communication
   // (115200 chosen because it is required for Teapot Demo output, but it's
   // really up to you depending on your project)
   Serial.begin(38400);
   
   if( standAlone==0 ) while (!Serial); // wait for Leonardo enumeration, others continue immediately

   // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
   // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
   // the baud timing being too misaligned with processor ticks. You must use
   // 38400 or slower in these cases, or use some kind of external separate
   // crystal solution for the UART timer.

   // initialize device
   Serial.println(F("Initializing I2C devices..."));
   mpu.initialize();

   // verify connection
   Serial.println(F("Testing device connections..."));
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

   // wait for ready
   if (standAlone==0 ) { //ylh: only when feeding data to Processing

      Serial.println(F("\nSend any character to begin DMP programming and demo: "));
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again
   } 

   // load and configure the DMP
   Serial.println(F("Initializing DMP..."));
   devStatus = mpu.dmpInitialize();

   // supply your own gyro offsets here, scaled for min sensitivity
   /*
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    */

   mpu.setXGyroOffset(110);
   mpu.setYGyroOffset(-3);
   mpu.setZGyroOffset(-20);
   mpu.setXAccelOffset(0);
   mpu.setYAccelOffset(0);
   mpu.setZAccelOffset(0); // 1688 factory default for my test chip

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
   } 
   else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
   }

   // configure LED for output
   pinMode(LED_PIN, OUTPUT);

   //ylh
   delay(2000);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
   //ylh
   if( debug ) delay (50);
   buzzer.Check();

   // if programming failed, don't try to do anything
   if (!dmpReady) return;

   // wait for MPU interrupt or extra packet(s) available
   while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
   }

   // reset interrupt flag and get INT_STATUS byte
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();

   // get current FIFO count
   fifoCount = mpu.getFIFOCount();

   // check for overflow (this should never happen unless our code is too inefficient)
   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();

      // ylh
      //Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
   } 
   else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
      // display quaternion values in InvenSense Teapot demo format:
      if( standAlone == 1 ) {
         mpu.dmpGetQuaternion(&q, fifoBuffer);
         Stand1();
      } 
      else if( standAlone == 2 ){
         // display real acceleration, adjusted to remove gravity
         mpu.dmpGetQuaternion(&q, fifoBuffer);
         mpu.dmpGetAccel(&aa, fifoBuffer);
         mpu.dmpGetGravity(&gravity, &q);
         mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

         mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

         mpu.dmpGetEuler(euler, &q);

         mpu.dmpGetGyro(&gyro, fifoBuffer);

         Stand2();
      }
      else if( standAlone == 0 ){
         teapotPacket[2] = fifoBuffer[0];
         teapotPacket[3] = fifoBuffer[1];
         teapotPacket[4] = fifoBuffer[4];
         teapotPacket[5] = fifoBuffer[5];
         teapotPacket[6] = fifoBuffer[8];
         teapotPacket[7] = fifoBuffer[9];
         teapotPacket[8] = fifoBuffer[12];
         teapotPacket[9] = fifoBuffer[13];

         Serial.write(teapotPacket, 14);
      }

      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
   }
}


















