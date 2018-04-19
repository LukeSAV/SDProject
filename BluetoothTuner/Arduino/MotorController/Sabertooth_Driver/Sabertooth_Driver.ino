// Sweep Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <Servo.h>

Servo ST1, ST2; // We'll name the Sabertooth servo channel objects ST1 and ST2.
                // For how to configure the Sabertooth, see the DIP Switch Wizard for
                //   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                // Be sure to select RC Microcontroller Mode for use with this sample.
                //
                // Connections to make:
                //   Arduino Pin 9  ->  Sabertooth S1
                //   Arduino Pin 10 ->  Sabertooth S2
                //   Arduino GND    ->  Sabertooth 0V
                //   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
                //
                // Sabertooth accepts servo pulses from 1000 us to 2000 us.
                // We need to specify the pulse widths in attach(). 0 degrees will be full reverse, 180 degrees will be
                // full forward. Sending a servo command of 90 will stop the motor. Whether the servo pulses control
                // the motors individually or control throttle and turning depends on your mixed mode setting.
int l_power;
int r_power;
bool sync_received;
int sync_time_count;
// Notice these attach() calls. The second and third arguments are important.
// With a single argument, the range is 44 to 141 degrees, with 92 being stopped.
// With all three arguments, we can use 0 to 180 degrees, with 90 being stopped.

void syncCheck() {
  if(!sync_received) {
    for(int i = 0; i < 90; i++) {
      if(r_power > 90) {
        r_power--;
      } else if(r_power < 90) {
        r_power++;
      }
      if(l_power > 90) {
        l_power--;
      } else if(l_power < 90) {
        l_power++;
      }
      delay(10); //5 ms delay
    }  
  }
}

void setup()
{
  ST1.attach( 9, 1000, 2000);
  ST2.attach(10, 1000, 2000);
  Serial.begin(9600);
  l_power = 90;
  r_power = 90;
  sync_received = true;
  sync_time_count = 0;
}


void loop()
{
  if(Serial.available()) {
    String myStr = Serial.readStringUntil('\n');
    int lenStr = myStr.length();
    
    if(myStr[0] == 'L') {
      l_power = myStr.substring(1).toInt(); // Convert torque request to int
      Serial.print(l_power);
      Serial.print('\n');
    } else if(myStr[0] == 'R') {
      r_power = myStr.substring(1).toInt(); // Convert torque request to int
      Serial.print(r_power);
      Serial.print('\n');
    } else if(myStr[0] == 'S') { // Sync pulse
      sync_received = true;
    }
  }
  // Ramp both servo channels from 0 to 180 (full reverse to full forward),
  // waiting 20 ms (1/50th of a second) per value.

  /*for (power = 0; power <= 180; power ++)
  {
    ST1.write(power);
    ST2.write(power);
    delay(100);
  }*/
  
  // Now go back the way we came.
  /*for (power = 180; power >= 0; power --)
  {
    ST1.write(power);
    ST2.write(power);
    delay(100);
  }*/
  ST1.write(l_power);
  ST2.write(r_power);
  sync_time_count++;
  if(sync_time_count > 1000) {
    syncCheck();
    sync_received = false;
    sync_time_count = 0;
  }
    
  delay(1);
}
