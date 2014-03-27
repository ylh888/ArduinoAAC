
/*
 * ASOneClick_a
 *  analog input to trigger an iPad key sequence for SELECT (button press)
 * 
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

const int testing = 1; // set this to 0 for final version

//digital param
const int trigger = 1; // reading at this value will trigger
const int pinIn=4;

int lastRead=1, count=0;

DigitalSensor s(pinIn, 500, 1, 0);  // Pin, Refractory period, Normal High, Debug

//AnalogSensor s(A0, 500, 0, 0);

void setup() {    
  pinMode(pinIn, INPUT);
}

void loop() {
   int x,y;
  
   delay(10);
   count++;
   
   x=s.Read();  // digital read
   //x=s.Level(nLevels);  //analog read
   
   if(x ==lastRead) {
     return;
   }     
   
   if( testing==1 && x!=lastRead) {
       Serial.print(count);
       Serial.print(" ");
       Serial.print(x);
       Serial.print(" ");
   }
   
   lastRead = x;  
   
   if( x==trigger ) {
     if( testing == 0 ) {
       Keyboard.press(KEY_UP_ARROW); 
       Keyboard.press(KEY_DOWN_ARROW); 
       Keyboard.releaseAll();
     }  else {
 
       Serial.print(" *T*\n");  // Triggered
     }
   }
   
}
