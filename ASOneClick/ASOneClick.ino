
/*
  ASOneClick
  
 this code is in the public domain

*/
#include "AACcollection.h"
#include "Wire.h"
#include "L3G4200D.h"
#include "Esplora.h"

int count=0;
DigitalSensor s(4, 500, 1, 0);  // Pin, Debounce, Normal High, Debug=0

//AnalogSensor s(A0, 500, 0, 2);

int lastRead=0;
void setup() {
       
}

void loop() {
  int x;
   delay(5);
   x=s.Read();
   //Serial.print(x);
   if(x !=lastRead) {
     lastRead = x;
     if( x== 1 ) {
      /* 
       Serial.print(count++);
       Serial.print(" ");
       Serial.print(x);
       Serial.print("\n");
      */
       
      Keyboard.press(KEY_UP_ARROW); 
      Keyboard.press(KEY_DOWN_ARROW); 
      Keyboard.releaseAll();
      
       
     }
   }
}
