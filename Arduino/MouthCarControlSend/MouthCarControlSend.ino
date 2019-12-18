#include <CapacitiveSensor.h>
#include <SoftwareSerial.h>

/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10 megohm between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50 kilohm - 50 megohm. Larger resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this pin
 * Best results are obtained if sensor foil and wire is covered with an insulator such as paper or plastic sheet
 */
 
SoftwareSerial BTSerial = SoftwareSerial(15,14); // RX | TX

CapacitiveSensor   cs_3 = CapacitiveSensor(4,5);        // 10 megohm resistor between pins 4 & 2, pin 2 is sensor pin, add wire, foil
CapacitiveSensor   cs_4 = CapacitiveSensor(4,6);        // 10 megohm resistor between pins 4 & 6, pin 6 is sensor pin, add wire, foil
CapacitiveSensor   cs_5 = CapacitiveSensor(4,7);        // 10 megohm resistor between pins 4 & 8, pin 8 is sensor pin, add wire, foil
CapacitiveSensor   cs_6 = CapacitiveSensor(4,8); 

int CAP_THRESHOLD = 300;

char lastCommand = ' ';

void setup()                    
{
   BTSerial.begin(9600); // HC-05 default speed in AT command more
   BTSerial.println("AT\r\n");
   
   //cs_3.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
   Serial.begin(9600);

   delay(300);
}

void loop()                    
{
    long start = millis();
    long total1 =  cs_3.capacitiveSensor(30);
    long total2 =  cs_4.capacitiveSensor(30);
    long total3 =  cs_5.capacitiveSensor(30);
    long total4 =  cs_6.capacitiveSensor(30);

    if(total1 > CAP_THRESHOLD) {
      BTSerial.write('f');
      lastCommand = 'f';
      Serial.println("forward");
    }
    else if(total2 > CAP_THRESHOLD) {
      BTSerial.write('b');
      lastCommand = 'b';
      Serial.println("back");
    }
    else if(total3 > CAP_THRESHOLD) {
      BTSerial.write('r');
      lastCommand = 'r';
      Serial.println("right");
    }
    else if(total4 > CAP_THRESHOLD) {
      BTSerial.write('l');
      lastCommand = 'l';
      Serial.println("left");
    }
    else {
      BTSerial.write('s');
    }
    

    //Serial.print(millis() - start);        // check on performance in milliseconds

    Serial.print(total1);                  // print sensor output 1
    
    Serial.print("\t");
    Serial.print(total2);                  // print sensor output 2

    Serial.print("\t");
    Serial.print(total3);                // print sensor output 3
    
    Serial.print("\t");
    Serial.println(total4);       

    delay(100);                             // arbitrary delay to limit data to serial port 
}
