/*
 * ASshort_longAnalog v3 Leornado
 
Leonardo for V using DIGITAL sensor (but using the analog algorithm)
 
 v3 uses buzzer requiring modulation
 
 short click ON DOWN sends iPad Select
 long click ON UP sends relay signal
 
 Added timer so that beyond longclickTime then
 sends relay signal, sound buzzer for 3 seconds
 
 * Copyright (c) 2014 Ability Spectrum.  For more information, see
 * 
 * http://abilityspectrum.com/
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <AACsensors.h>
#include <AACswitch.h>

const int testing = 0; // set this to 0 for final version

//digital param
const int triggerD = 0; // reading at this value will trigger
const int pinIn=6;  // digital sensor
const int pinLED=13;
Switch buzzer( 8, 50, 1 ); // use THIS
Switch relay( 2, 200, 0);
int pinBuzzer = 4;  // Serial buzzer - not used

//analog params
const int nLevels = 2; // number of levels
const int triggerA = 0; // reading at this value or less will trigger; must be less than nLevels
long lastTriggeredA = 0;
long lastTriggeredD = 0;
int lastReadA=1;
int lastReadD=1;
int triggeredD=0;
int soundLong=0;

long now;

const long ignoreTime = 400;
const long longclickTime = 1200;
const long discardTime = 2500;

int count=0;

//DigitalSensor sD(pinIn, 30, 1, 0);  // Pin, Refractory period, Normal High, Debug

AnalogSensor sA(A0, 200, nLevels-1, 0);

const int ylh[14] = { 
   3,1,3,3,0,1,3,1,1,0,1,1,1,1 }; //-.-- .-.. ....
const int morseTime=50;

void Signature() {
   //if(testing) return;

   for( int j=0; j<15; j++ ) {
      switch(ylh[j]) {
      case 0:
         delay(morseTime*3);
         break;
      default:
         for (int i=0; i<morseTime*ylh[j]; i++ ) {
            digitalWrite(pinBuzzer, 1);
            delay (1);
            digitalWrite(pinBuzzer, 0);
            delay (1);
         }
         delay(morseTime);
         break;
      }
   }
}

void SerialBuzzer(int times, int pitch) {
   int i;

   if(testing>1) {  // silent mode
      Serial.print(" +");
      Serial.print(times);
      Serial.print("+");
      Serial.println(pitch);
      return;
   }

   for(int j=0; j<times; j++ ) {
      digitalWrite(pinLED,0);  // pro micro led
      for (i=0; i<250; i++ ) {
         digitalWrite(pinBuzzer, 1);
         delay (pitch);
         digitalWrite(pinBuzzer, 0);
         delay (pitch);
      }
      digitalWrite(pinLED,1);
      delay(70);
   }
}

void tryTriggerLongA(int byWhat) {
   now = millis();
   if( ( now > (lastTriggeredA + longclickTime) ) &&
      ( now < (lastTriggeredA + discardTime) ) )
   {

      if( testing == 0 )
      {
         relay.On(300);
         buzzer.On(1000);
      } 
      else {
         Serial.println(count);

         Serial.print("\n*A--------A*\n");  // Triggered
         relay.On(300);
      }
      if(byWhat==1) soundLong = 0;  // long done
      Serial.print(" x"); 
      Serial.println(soundLong);

      if(++soundLong>5 && byWhat==2) {
         return; // dont buzz
      }
      if( byWhat== 2 ) {
         buzzer.On(1000);
         // SerialBuzzer(1,byWhat);  // DISABLED
      }

      //lastTriggeredA = millis();
   }
}

void setup() {    
   pinMode(pinIn, INPUT_PULLUP);
   pinMode(pinBuzzer, OUTPUT);
   SerialBuzzer(3,1);
   buzzer.On(300);
   //Signature();
}

void loop() {
   int xD,xA;

   delay(10);
   relay.Check();
   buzzer.Check();

   count++;
   xD = digitalRead(pinIn);

   //xD=sD.Read();  // digital read using library routine 
//   //************** DISABLE ANALOG?
   xA=sA.Level(nLevels);  //analog read   
   xD=1;
   
   if( testing>0 && xD!=lastReadD) {
      //Serial.print(count);
      Serial.print("d=");
      Serial.print(xD);
      Serial.print(" ");
   }
   if( testing>0 && xA!=lastReadA) {
      //Serial.print(count);
      Serial.print("a=");
      Serial.print(xA);
      Serial.print(" ");
      Serial.flush();
   }


   if( xA!=lastReadA ) {
      // state changed
      if( xA==triggerA ) {  // DOWN TICK
         now = millis();  
         if( now > (lastTriggeredA + longclickTime) ) {
            // soundLong++;
         }
         if( now < (lastTriggeredA + ignoreTime) ) {  //ignore
            return;
         }
         else  {   // trigger short
            if( testing == 0 ) {
               Keyboard.press(KEY_UP_ARROW); 
               Keyboard.press(KEY_DOWN_ARROW); 
               Keyboard.releaseAll();
               buzzer.On(50);
            }  
            else {
               Serial.print(count);
               Serial.print(" *A*\n");  // Triggered
               relay.On(30);
               buzzer.On(50);
            }       
            soundLong=0;
            // DIABLED
            //SerialBuzzer(1,1);
            lastTriggeredA = millis();
         }
      }
      else { // UP TICK
         tryTriggerLongA(1);
      }
   } 
   else {     //xA==last level
      if( xA==triggerA ) {

         tryTriggerLongA(2);
      }
   }

   lastReadD = xD; 
   lastReadA = xA; 
}











