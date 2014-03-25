/* IRremote_joy
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

/*
#include <IRLib.h>
#include <SoftwareSerial.h>
#include "Wire.h"

#include "Arduino.h"
#include "Esplora.h"
#include "L3G4200D.h"
*/

#include "AACcollection.h"

Joystick::Joystick (int x=A0, int y=A1, int z=A2, int debug=0) {
  _debug = debug;
  xAxis=x;
  yAxis=y;
  zAxis=z;
  right=3; up=4; down=5; left=6;

  range = 16;
  threshold = range/4;
  center = range/2;

  /* joystick with buttons
  switchPin=2;
  mouseButton=8;
  ledPin=5;

  pinMode(sel, INPUT);       // the switch pin - for joystick with push buttons  
  digitalWrite(sel, HIGH);
  pinMode(left, INPUT); 
  
  digitalWrite(left, HIGH);  //Enable the pull-up resistor on button 0
  pinMode(right, INPUT); 
  digitalWrite(right, HIGH);  
  pinMode(up, INPUT); 
  digitalWrite(up, HIGH);  
  pinMode(down, INPUT); 
  digitalWrite(down, HIGH);        

  rightState = false; upState=false; leftState=false; downState=false;
  
  //pinMode(ledPin, OUTPUT); // the LED pin  - for joystick with push buttons
 // take control of the mouse:
  */
  
  Mouse.begin(); 
}

int Joystick::readAxis(int thisAxis) { 
  // read the analog input:
  int reading = analogRead(thisAxis);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the
  // rest position threshold,  use it:

  int distance = reading - center;

  /* turn off thresholding
  if (abs(distance) < threshold) {
    distance = 0;
  } 
  */

  // return the distance for this axis:
  return distance;
}

// joystick with buttons
int Joystick::checkButtons(void) {
  selState  = 1-digitalRead(sel); //separate pin for mouse select
  leftState = 1-digitalRead(left);
  rightState= 1-digitalRead(right);
  upState=    1-digitalRead(up);
  downState=  1-digitalRead(down);
  
  Serial.print(leftState);  Serial.print(",");
  Serial.print(upState);  Serial.print(",");
  Serial.print(downState);  Serial.print(",");
  Serial.print(rightState);  Serial.print(",");
  Serial.print("\n");  
  
  return(leftState+rightState+upState+downState);
}

void Joystick::DoMouseWork(void) {
  int xReading;
  int yReading;
  int zReading;

  /* Joy BEGIN */
    // save switch state for next comparison:
  /*
  nButtons=checkButtons();
  
  xReading=0; yReading=0;  
  if(leftState==1) { xReading=-range/4; }
  if(rightState==1){ xReading=range/4; }
  if(upState==1)   { yReading=-range/4; }
  if(downState==1) { yReading=range/4; } 
  
  if ((leftState+rightState)==2)  {
     mouseIsActive = !mouseIsActive;
     // turn on LED to indicate mouse state:
     digitalWrite(ledPin, mouseIsActive); 
     lastSwitchState = switchState;
     delay(250);
  }
  // if the mouse control state is active, move the mouse:
  if (mouseIsActive) {
    Mouse.move(xReading, yReading, 0);
  }  
  */
  
  // read and scale the two axes:
  xReading = -readAxis(xAxis);
  yReading = readAxis(yAxis);
  zReading = readAxis(zAxis); // added for joystick pushdown - ylh
  
  if (_debug && ( xReading!=0 || yReading!=0 || zReading!=-center)) { // if there's activity
  Serial.print("X=");
  Serial.print(xReading);
  Serial.print("; Y=");
  Serial.print(yReading);
  Serial.print("; Z=");
  Serial.println(zReading);
  }
  if(zReading >0) {
    if( xReading>=3 )  {  // activate, de-activate here, when x pushed to the right
    mouseIsActive=!mouseIsActive;
    delay(1000); // make press sticky
    } else if (xReading==0 && yReading==0) {  // x, y no active while joystick pressed means left-moused pressed 
    if (!Mouse.isPressed(MOUSE_LEFT)) {
      Mouse.press(MOUSE_LEFT);   
      delay(250);    
    } else {
      Mouse.release(MOUSE_LEFT);
    }
    }
  }
  
  // if the mouse control state is active, move the mouse:
  if (mouseIsActive) {
    /* minimize fine movement
    if(xReading < 0 && xReading >= -3) xReading=-1; 
    if(xReading > 0 && xReading <=  3) xReading=1;
    if(yReading < 0 && yReading >= -3) yReading=-1; 
    if(yReading > 0 && yReading <=  3) yReading=1; 
    */
    Mouse.move(xReading, yReading, 0);
  } 

  // read the mouse button and click or not click:
  // if the mouse button is pressed:
  if (selState) {
    // if the mouse is not pressed, press it:
    if (!Mouse.isPressed(MOUSE_LEFT)) {
      Mouse.press(MOUSE_LEFT); 
    }
  } 
  // else the mouse button is not pressed:
  else {
    // if the mouse is pressed, release it:
    if (Mouse.isPressed(MOUSE_LEFT)) {
      Mouse.release(MOUSE_LEFT); 
    }
  }
  /* Joy END */
}

Sensor::Sensor( int in_pin, int scale, int debug) {
  _debug = debug;
  _sensorPin = in_pin;
  pinMode(_sensorPin,INPUT);
  _scale = scale;

}

int Sensor::Read() {

  /* Sensor BEGIN */
  sensorValue = analogRead(_sensorPin); 
  int newlevel = 0;
  int x=0;
  while( sensorValue>x ) {
    x=x+_scale;
    newlevel = newlevel + 1;
  }
  if(sensorLevel == newlevel) {
    sensorLasted +=1;
  } else {
    if( _debug ) {  // level changed, print how long previous level lasted & the new level
      int x=0;
      Serial.print(" "); Serial.println(sensorLasted);
      Serial.print(newlevel);
      while (newlevel>x) {
        Serial.print("*");
        x=x+1;
      }
     } 
    sensorLasted = 0;
    sensorLevel = newlevel;
  }
  return sensorLevel;
}

int Sensor::RawValue() { return sensorValue; }
int Sensor::HeldPeriod() { return sensorLasted; }

/////////////

DigitalSensor::DigitalSensor(int pin, int ref=500, int normalhigh=0, int debug=0) {
  _sensorPin=pin;
  _refractory = ref;
  _normalhigh = normalhigh;
  _debug=debug;
  _lastDetected= millis();
  pinMode(pin,INPUT);
}

int DigitalSensor::Read() {
  int x;
  x=digitalRead(_sensorPin);
  if(_debug) {Serial.print("Pin "); Serial.print(_sensorPin); Serial.print(" = "); Serial.println(x); }

  if( (millis() - _lastDetected) < _refractory ) {
    return _normalhigh;   // force read of 0 if within refractroy period
  }
  if( x==_normalhigh ) {
    return 0;
  } else {
    _lastDetected = millis();
    return 1;
  }
}


// Analog sensor: differentiates ternary and quarternary levels
// recalibrates when held at the same level for n=5 cycles (reads)
// Args: input pin,  refractory period,   debug
// Forced return of last value if read within refractory period 
// atRest is the rest level that is common. No recalibration at this level
AnalogSensor::AnalogSensor(int pin=A4, int ref=0, int atRest=0, int debug=0) {
  if( debug ) Serial.begin(9600);
  _sensorPin=pin;
  pinMode(pin,INPUT);
  _refractory = ref;
  _atRest= atRest;
  _debug=debug;
  Reset();
}

void AnalogSensor::Reset() {
  minima=10000;
  maxima=-10000;
  held=0; heldLevel=_atRest;
  if(_debug>2 ) Serial.println("                          * Reset");

  Level(2);
}

int AnalogSensor::Read() {

   sensorValue = analogRead(_sensorPin);
   if( sensorValue < minima) minima = sensorValue;
   if( sensorValue > maxima) maxima = sensorValue;
   if( abs(maxima-minima) <100) {
     // usually following recalibration; need to push min and max apart
     // recalibrate only when NOT at the _atRest state so adjust either min or max
     if(_atRest==0) {  
       minima=minima-50;     
     } else {
       maxima=maxima+50;
     }
   }
   return sensorValue;
}

// takes reading, how many levels?  3 means 0,1,2 as possible return values
int AnalogSensor::Level(int level) {
  int x;

  Read();
  if( (millis() - _lastDetected) < _refractory ) {
   //Serial.print((millis()-_lastDetected));
   //Serial.println("     ----- in refractory ----" );
   return heldLevel;   // return last value if within refractroy period
  }

   if( level<0 || level >10 ) level = 3;
   x=map( sensorValue, minima, maxima, 0, level);
   x=constrain( x, 0, level-1);  // throw away the high anchor

  // go into calibration mode
  if( x!=_atRest ) {
    if( x==heldLevel ) {
      held++; 
      if( held>6 ) {
        Reset();
        _lastDetected = millis();
      }
    } else {
       heldLevel=x;
       held=0;
       _lastDetected = millis();
    }
  }

  levelValue = x; 
  return levelValue; // also a class variable
}

void AnalogSensor::Dump() {
  if(_debug) {
    Serial.print("Sensor=");
    Serial.print(sensorValue);
    Serial.print("   "); Serial.print(minima);
    Serial.print("-"); Serial.print(maxima);  
    Serial.println();

    if( _debug>1 ) {
      Serial.print(" now=");
      Serial.print(millis());
      Serial.print(" last=");
      Serial.print(_lastDetected);
      Serial.print(" cnt=");
      Serial.print(held);
      Serial.print(" reported=");
      Serial.print(heldLevel);
      Serial.print(" actual=");
      Serial.println(levelValue);
    }
  }
}

// old code
int AnalogSensor::_Level3() {
  if( abs(sensorValue - minima) < (maxima-minima)/3 ) return 0;
  if( abs(maxima - sensorValue) < (maxima-minima)/3 ) return 2;
  return 1;
}

int AnalogSensor::_Level4() {
  if( abs(sensorValue - minima) < (maxima-minima)/4 ) return 0;
  if( abs(maxima - sensorValue) < (maxima-minima)/4 ) return 3;
  if( abs(sensorValue - minima) < (maxima-minima)/2 ) return 1;
  return 2;
}
////////////////////////////

EsAccelerometer::EsAccelerometer( int debug ) {
  _debug = debug;
  _xAdj=0;
  _yAdj=0;
  _zAdj=0;
  minx=255; miny=255; minz=255;
  maxx=-255;maxy=-255;maxz=-255;
  _calibrated = 0;
}

void EsAccelerometer::Read() {
  xAxis = Esplora.readAccelerometer(X_AXIS) + _xAdj;
  yAxis = Esplora.readAccelerometer(Y_AXIS) + _yAdj;
  zAxis = Esplora.readAccelerometer(Z_AXIS) + _zAdj;

/*  LATER!
  if( _calibrated ) {
    xAxis = map( xAxis, minx,maxx,-3,3); 
    xAxis = constrain(xAxis, minx, maxx);

    yAxis = map( yAxis, miny,maxy,-3,3); 
    yAxis = constrain(yAxis, miny, maxy);

    zAxis = map( zAxis, minz,maxz,-3,3);  
    zAxis = constrain(zAxis, minz, maxz);
  }
*/

}

void EsAccelerometer::Reset() {
  
}


void EsAccelerometer::Calibrate() {
  _xAdj = - Esplora.readAccelerometer(X_AXIS); 
  _yAdj = - Esplora.readAccelerometer(Y_AXIS); 
  _zAdj = - Esplora.readAccelerometer(Z_AXIS); 
  minx=255; miny=255; minz=255;
  maxx=-255;maxy=-255;maxz=-255;
  _calibrated=1;

/* LATER!
  if( (Esplora.readAccelerometer(X_AXIS)+_xAdj )> maxx ) {
     maxx=Esplora.readAccelerometer(X_AXIS)+_xAdj;
  }
    if( (Esplora.readAccelerometer(X_AXIS)+_xAdj )< minx ) {
     minx=Esplora.readAccelerometer(X_AXIS)+_xAdj;
  }
  if( (Esplora.readAccelerometer(Y_AXIS)+_yAdj )> maxy ) {
     maxy=Esplora.readAccelerometer(Y_AXIS)+_yAdj;
  }
    if( (Esplora.readAccelerometer(Y_AXIS)+_yAdj )< miny ) {
     miny=Esplora.readAccelerometer(Y_AXIS)+_yAdj;
  }
  if( (Esplora.readAccelerometer(Z_AXIS)+_zAdj )> maxz ) {
     maxz=Esplora.readAccelerometer(Z_AXIS)+_zAdj;
  }
  if( (Esplora.readAccelerometer(Z_AXIS)+_zAdj )< minz ) {
     minz=Esplora.readAccelerometer(Z_AXIS)+_zAdj;
  } 

*/

    _calibrated = 1;
}


int EsAccelerometer::x() {
  return xAxis;
}
int EsAccelerometer::y() {
  return yAxis;
}
int EsAccelerometer::z() {
  return zAxis;
}

EsJoy::EsJoy( int debug ) {
  _debug = debug;
  _xAdj=0;
  _yAdj=0;

  Mouse.begin();
  mouseactivated=0;
  mousespeed=10;
  _calibrated = 0;
}

void EsJoy::Reset() {

  mouseactivated = 1-mouseactivated;
}

void EsJoy::Read() {
  xAxis = Esplora.readJoystickX() + _xAdj;
  yAxis = Esplora.readJoystickY() + _yAdj;

}

int EsJoy::x() {
  return xAxis;
}
int EsJoy::y() {
  return yAxis;
}

void EsJoy::Calibrate() {
  _xAdj= - Esplora.readJoystickX();
  _yAdj= - Esplora.readJoystickY();
  mousespeed = map(Esplora.readSlider(), 0, 1023, 10, 0);
}

void EsJoy::DoMouseWork(void) {
  if( _debug ) {
     Serial.print("Joy: ");
     Serial.print(Esplora.readJoystickX());
     Serial.print(" +");
     Serial.print(_xAdj);
     Serial.print(", ");
     Serial.print(Esplora.readJoystickY());
     Serial.print(" +");
     Serial.print(_yAdj);
     Serial.print("; ");
     Serial.print(mousespeed);
     Serial.print(" | ");
     
  }
  if( mouseactivated ) {
  int mouseX = map( Esplora.readJoystickX()+_xAdj,-512, 512, mousespeed, -mousespeed); 
  int mouseY = map( Esplora.readJoystickY()+_yAdj,-512, 512, -mousespeed, mousespeed);
    Mouse.move(mouseX, mouseY, 0);
  }
 
}


EsAccAsMouse::EsAccAsMouse( int debug ) {
  _debug = debug;
  Mouse.begin();
  axis[0]=X_AXIS; axis[1]=Y_AXIS;

  Calibrate();

  mouseactivated = 0;
  _calibrated = 0;
}

void EsAccAsMouse::Reset() {
  //pAcc->Reset();
  mouseactivated = 1-mouseactivated;

  // read slider to decide how sensitive the mouse is
  //range = 16;
  range = map(Esplora.readSlider(), 0, 1023, 30, 0);
  responseDelay = 2;
  threshold = range/4;
  center = range/2;

  delay(300);
}

void EsAccAsMouse::Calibrate() {

  minima[0]=-20; minima[1]=-20;
  maxima[0]=20;    maxima[1]=20;
}


int EsAccAsMouse::readAxis(int axisNumber) {
  int distance = 0;    // distance from center of the output range

  // read the analog input:
  int reading = Esplora.readAccelerometer(axis[axisNumber]);
  mouseReading[axisNumber] = reading;

// of the current reading exceeds the max or min for this axis,
// reset the max or min:
  if (reading < minima[axisNumber]) {
    minima[axisNumber] = reading;
  }
  if (reading > maxima[axisNumber]) {
    maxima[axisNumber] = reading;
  }

  // map the reading from the analog input range to the output range:
  reading = map(reading, minima[axisNumber], maxima[axisNumber], 0, range);

  // shorten the range 
  if( (maxima[axisNumber] - minima[axisNumber]) > 75 ) {
   maxima[axisNumber] = maxima[axisNumber] - 5;
   minima[axisNumber] = minima[axisNumber] + 5;
  }

  // stablize the center - prevent drift 
  int diff = mouseReading[axisNumber] - previous[axisNumber];
  if( abs(diff) <5) {
      minima[axisNumber] = minima[axisNumber]+diff/2;
      maxima[axisNumber] = maxima[axisNumber]+diff/2;
  }
  previous[axisNumber] = mouseReading[axisNumber];

 // if the output reading is outside from the
 // rest position threshold,  use it:
  if (abs(reading - center) > threshold) {
    distance = (reading - center);
  } 

  // return the distance for this axis:
  return - distance;  // inverted for accelerometer
}


void EsAccAsMouse::DoMouseWork(void) {
    int mouseX = readAxis(0); 
    int mouseY = readAxis(1);

  if( mouseactivated ) {
    Mouse.move(readAxis(0), mouseY, 0);
  
 
  if( _debug  ) {
     Serial.print("Acc: ");
     Serial.print(mouseX);
     Serial.print(" ( ");
     Serial.print(mouseReading[0]);
     Serial.print(")  ");
     Serial.print(mouseY);
     Serial.print(" ( ");
     Serial.print(mouseReading[1]);
     Serial.print(") ");
     Serial.print(minima[0]);
     Serial.print("-");
     Serial.print(maxima[0]);
     Serial.print("  ");
     Serial.print(minima[1]);
     Serial.print("-");
     Serial.print(maxima[1]);
     Serial.print("  ");

     Serial.print("; r=");
     Serial.print(range);
     Serial.print(" | ");
  }

  } // mouseactivated
}

int EsAccAsMouse::activated(void) {
  return mouseactivated;
}

////////////////
L3G4asMouse::L3G4asMouse( int debug ) {
  _debug = debug;

  if(debug) Serial.begin(9600);
  Wire.begin();
  gyro.enableDefault();

  Mouse.begin();
  axis[0]=0; axis[1]=1;

  Reset();

  Calibrate();
  _activated = 1;
}
void L3G4asMouse::Reset() {

  _activated = 0;

  minima[0]=1000; minima[1]=1000;
  maxima[0]=-1000;    maxima[1]=-1000;

}

void L3G4asMouse::Calibrate() {
// not finished
  minima[0]=-20; minima[1]=-20;
  maxima[0]=20;    maxima[1]=20;
  _calibrated=1;
}


int L3G4asMouse::readAxis(int axisNumber) {
  int distance = 0;    // distance from center of the output range
  int reading, current;

  // read the analog input:
  gyro.read();

  if( axisNumber==0 )
    reading = -gyro.g.x;
  else
    reading = -gyro.g.z;
    
  if( reading < minima[axisNumber] ) minima[axisNumber]=reading;
  if( reading > maxima[axisNumber] ) maxima[axisNumber]=reading;
  

  if( abs(reading)<2000) { 
      current = 0;  
  }
  else {
     if( reading >0 ) {
       current= 1;
     } else {
       current= -1;
     } 
  }
 
  msize[axisNumber] =constrain(  map( abs(reading), 0, 6000, 0,6), 1, 5);
      
  return current;
}


void L3G4asMouse::DoMouseWork(void) {
    int mouseX,mouseY;
    mouseReading[0] = readAxis(0); 
    mouseReading[1] = readAxis(1);

  if( _activated ) {
   mouseX=0;
   mouseY=0;
   for( int i=0; i<2; i++) {
     if( mouseReading[i]==0 ) { // current = 0
       if( prev[i]!=0) {  // move as previous
          if(i==0) mouseX=prev[i]*msize[i];
          else mouseY=prev[i]*msize[i];
         mouseReading[i] = prev[i];
       } else {  // previous not moved as well: do nothing
      
       }
     }
      else {  // current = moved
      if(mouseReading[i]==prev[i]) {  // same direction
          if(i==0) mouseX=mouseReading[i]*msize[i];
          else mouseY=mouseReading[i]*msize[i];
      } else { 
        if( prev[i]==0 ){   // prev not moved
          if(i==0) mouseX=mouseReading[i]*msize[i];
          else mouseY=mouseReading[i]*msize[i];
          prev[i]=mouseReading[i];
        } else {  // reverse direction
                
          delay(300); 
         
              //mouseReading[i]=readAxis(i);
              prev[0]=prev[1]=0;
              msize[0]=msize[1]=2;
              mouseX=0;
              mouseY=0;                     
            
        }    
      }
    }
  }
    Mouse.move(mouseX,mouseY,0);
  } // mouseactivated
}

void L3G4asMouse::activate(int _a=1) {
  _activated=_a;
}

int L3G4asMouse::activated(void) {
  return _activated;
}
void L3G4asMouse::dumpMinMax() {
  if(_debug) {
    Serial.print("Gyro ");
    Serial.print(minima[0]);Serial.print(" "); Serial.println(maxima[0]);
    Serial.print(" | ");
    Serial.print(minima[1]);Serial.print(" "); Serial.println(maxima[1]);
  }
}
