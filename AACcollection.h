/* AACcollection.h
  merger of IRremoteserial_ylh and ylhjoy2 using Leonardo sensor shield, BT v.5
  
  Theory: Joystick pressed while shifting to the x-right turns joystick into mouse 
  what's received from IR gets sent to BT, and vice versa. In the format: NEC,77E1E041,32
  (protocol, hex code, no of bits)
  
  
  
  BlueTooth comm uses pins 10, 11 for Rx,Tx = software serial

  IR remote
  IR Tx on D9, Rx on D12 (selectable)
  
  Joystick controls tablet via OTG cable
  
  * 2-axis joystick connected to pins A0 and A1
  * z-axis pin A3 = left mouse press
  * activate/deactivate when pushdown while x is pushed to the right
      not using sel State
      
  * also has code for 4 pushbuttons here but not tested, uses selState
  
 The mouse movement is always relative. This sketch reads 
 two analog inputs that range from 0 to 1023 (or less on either end)
 and translates them into ranges of -6 to 6. 
 The sketch assumes that the joystick rsting values are around the 
 middle of the range, but that they vary within a threshold.
 
 WARNING:  When you use the Mouse.move() command, the Arduino takes
 over your mouse!  Make sure you have control before you use the command.
 This sketch includes a pushbutton to toggle the mouse control state, so
 you can turn on and off mouse control.
  
 IR code created 15 Sept 2011
 updated 28 Mar 2012
 by Tom Igoe
 
Merged code 6 Sept 2013 by ylh
 
 this code is in the public domain 
*/

#ifndef __AACcollection_h__
#define __AACcollection_h__

#include <Arduino.h>

#include <IRLib.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include "Esplora.h"
#include "L3G4200D.h"

/*
class IRremote
{
  public:
    IRremote( int send_pin, int recv_pin, int debug);
    IRsend My_Sender;
    IRrecv (rec_pin);
    IRdecode My_Decoder;

    void dot();
    void dash();
  private:
    void *My_Receiver();
    void *My_Decoder();
    int Debug;
    int _rec_pin;
    unsigned int Buffer[RAWBUF];
    int protocol;
    long code;
    int bits;
    String command;
};
*/

class Joystick
{
  public:
    Joystick(int,int,int,int);
    void DoMouseWork(void);

  private:
    int readAxis(int);
    int checkButtons(void);
    int _debug;
    int xAxis;         // joystick X axis  
    int yAxis;         // joystick Y axis
    int zAxis;         // joystick Z - down - ylh
    // parameters for reading the joystick:
    int range;        // output range of X or Y movement
    int threshold;      // resting threshold
    int center;         // resting position value
 
    boolean mouseIsActive;    // whether or not to control the mouse
    int lastSwitchState;      // previous switch state
    int rightState, upState, downState, leftState;
    int right, up, down, left;
    int selState;

    int switchPin;      // switch to turn on and off mouse control
    int mouseButton;    // input pin for the mouse pushButton
    int ledPin;         // Mouse control LED 

    int sel;                   // = 2mouse select pin
};

class Sensor
{
  public:
    Sensor(int, int, int);  // args: in_pin; scale; debug
    int Read();
    int RawValue();
    int HeldPeriod();

  private:
    int _sensorPin;     // analog input for Flex or Force
    int _scale;
    int sensorValue; 
    int sensorLevel;   // last level
    int sensorLasted;  // stable reading for how many iterations
    unsigned long _lastDetected;
    int _refractory;

    int _debug;
 };
 
class DigitalSensor
{
  public:
    DigitalSensor(int,int,int,int); // debug
    int Read(void);
  private:
    unsigned long _lastDetected;
    int _sensorPin;
    int _refractory;
    int _normalhigh;
    int _debug;
};

// recalibrates when held at the same level for 5 cycles
class AnalogSensor
{
  public:
    AnalogSensor(int, int, int, int);
    int Read();
    int Level(int); // number of levels differentiated: 2 or more
    void Reset();
    void Dump();
  private:
    int _Level3(), _Level4(); // old code for 3 or 4 level differentiation
    int minima, maxima;
    int sensorValue;
    int levelValue;
    int held, heldLevel;
    int _sensorPin;
    unsigned long _lastDetected;
    int _refractory;
    int _atRest;
    int _debug;
};


// Esplora device

class EsAccelerometer
{
  public:
    EsAccelerometer(int);  // args: debug
    void Read();
    void Reset();          // zero the readings at current position
    void Calibrate();      // determine max extension
    int x(), y(), z();

  private:
    int xAxis, yAxis, zAxis;
    int _xAdj, _yAdj, _zAdj;
    int minx, miny, minz, maxx, maxy,maxz;
    int _calibrated;
    int _debug;
 };
 
class EsJoy
{
  public:
    EsJoy(int);  // args: debug
    void DoMouseWork(void);
    void Read();
    void Reset();          // zero the readings at current position
    void Calibrate();      // determine max extension
    int x(), y();

  private:
    int xAxis, yAxis;
    int _xAdj, _yAdj;
    //int minx, miny, minz, maxx, maxy,maxz;
    int mousespeed;
    int mouseactivated;
    int _calibrated;
    int _debug;
 };
 
class EsAccAsMouse
{
  public:
    EsAccAsMouse(int);  // args: debug
    void DoMouseWork(void);
    //void Read();
    void Reset();          // zero the readings at current position
    void Calibrate();      // determine max extension
   int readAxis(int);
   int activated();

  private:
    int range;               // output range of X or Y movement
    int responseDelay;       // response delay of the mouse, in ms
    int threshold;      // resting threshold
    int center;         // resting position value
    int minima[2];
    int maxima[2];                 // actual analogRead maxima for {x, y}
    int axis[2];             // pin numbers for {x, y}
    int mouseReading[2];          // final mouse readings for {x, y}
    int previous[2];
    int mousespeed;

    int mouseactivated;
    int _calibrated;
    int _debug;
 };

class L3G4asMouse
{
  public:
    L3G4asMouse(int);  // args: debug
    void DoMouseWork(void);
    //void Read();
    void Reset();          // zero the readings at current position
    void Calibrate();      // determine max extension
   int readAxis(int);
   void activate(int);
   int activated();
   void dumpMinMax();

  private:
    L3G4200D gyro;
    int range;               // output range of X or Y movement
    int responseDelay;       // response delay of the mouse, in ms
    int threshold;      // resting threshold
    int center;         // resting position value
    int minima[2];
    int maxima[2];                 // actual analogRead maxima for {x, y}
    int axis[2];             // pin numbers for {x, y}
    int mouseReading[2];          // final mouse readings for {x, y}
    int prev[2];
    int msize[2];
    int mousespeed;

    int _activated;
    int _calibrated;
    int _debug;
 }; 

#endif
    
