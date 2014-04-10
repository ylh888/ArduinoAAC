/* AACswitch.cpp
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

#include "AACswitch.h"

Switch::Switch( int out_pin, int period=200, int debug=0) {
  _switchPin = out_pin;
  _period = period;
  _switched = 0; 
  _lastSwitched = 0; 
  _debug = debug;

  pinMode(_switchPin,OUTPUT);
}


void Switch::On( int period  = 0 ) {
  if( period == 0 )
    _period = 200;
  else
    _period = period;
  digitalWrite( _switchPin, HIGH);
  _switched=1;
  _lastSwitched = millis();

  if(_debug) digitalWrite(13, HIGH);
}

void Switch::Check() {
   if( _switched && (millis()- _lastSwitched ) > _period ) {
     _switched=0;
     digitalWrite( _switchPin, LOW);
     if(_debug) digitalWrite(13, LOW);
   }
}

SerialBuzzer::SerialBuzzer( int out_pin, int debug=0) {
  _buzzPin = out_pin;
  _debug = debug;

  pinMode(_buzzPin,OUTPUT);
}

void SerialBuzzer::Go(int times, int wavelength) {

   if(_debug>1) {  // silent mode
      Serial.print(" +");
      Serial.print(times);
      Serial.print("+");
      Serial.println(wavelength);
      return;
   }

   for(int j=0; j<times; j++ ) {
      if(_debug) digitalWrite(13, HIGH); // pro micro LED = 17
      for (int i=0; i<50; i++ ) {  // buzz length in ms
         digitalWrite(_buzzPin, 1);
         delay (wavelength);
         digitalWrite(_buzzPin, 0);
         delay (wavelength);
      }
      if(_debug) digitalWrite(13, LOW);
      delay(70);  // so that pause is audible
   }
}

