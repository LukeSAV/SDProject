
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(13, 12);

void setup() {
  bluetooth.begin(9600);
  Serial.begin(9600);
  Serial.println("Setting up");
}


static float kp = 0;
static float ki = 0;
static float kd = 0;

void loop() {
  if (bluetooth.available()) {
    String s = bluetooth.readStringUntil(';');
    switch (s[0]) {
    case 't': //Tuning command
      float val =  s.substring(3, s.length()).toFloat();
      switch(s[1]){
      case 'P':
          kp =val;
          Serial.print("KP: ");
          Serial.println(kp);
          break;
       case 'I':
          ki = val;
          Serial.print("Ki: ");
          Serial.println(ki);
          break;
       case 'D':
          kd = val;
          Serial.print("KD: ");
          Serial.println(kd);
          break;
       case 'F':
          Serial.print("KF: ");
           break;
       }
       break;
    }
  }
}

