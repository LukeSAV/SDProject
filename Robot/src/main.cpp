#include <Arduino.h>
#include <Sabertooth.h>

Sabertooth ST(128); 
void setup()
{
    pinMode(13, OUTPUT);
    SabertoothTXPinSerial.begin(9600); 
    ST.autobaud();                     
    digitalWrite(13, HIGH);
}
void loop()
{
    // Ramp motor 1 from -127 to 127 (full reverse to full forward),
    // waiting 20 ms (1/50th of a second) per value.
        ST.motor(1, 15);
        ST.motor(2, 15);

    digitalWrite(13, HIGH);
}