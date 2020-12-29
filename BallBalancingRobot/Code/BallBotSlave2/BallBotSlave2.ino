//StepperSlave 2

#include<Wire.h>

int y=0,z=0,d=0,s=0;
float w2=0,x=0;

void setup() 
{
 Wire.begin(3);
 Wire.onReceive(slave2);
 Serial.begin(9600);
 pinMode(2,OUTPUT);
 pinMode(3,OUTPUT);
 pinMode(4,OUTPUT);
 digitalWrite(2,LOW);
 digitalWrite(3,HIGH);
 digitalWrite(4,LOW);
}

void loop() 
{
 digitalWrite(2,HIGH);
 delayMicroseconds(x);
 digitalWrite(2,LOW);
 delayMicroseconds(x);
}

void slave2(int howMany)
{
 while(Wire.available()>3)
      {
       y=Wire.read();
      }
 while(Wire.available()>2)
      {
       z=Wire.read();     
      }
 while(Wire.available()>1)
      {
       d=Wire.read();     
      }
 s=Wire.read();
 if(s==0)
   {
    s=0;
    digitalWrite(3,HIGH);
    digitalWrite(4,LOW);
   }
 else
   {
    s=1;
    digitalWrite(3,LOW);
    digitalWrite(4,HIGH);  
   }
 w2=z+(d*0.01)+(255*y);
 x=map(w2,0.1,468,1000,120);
 if(w2==0)
   {
    x=0;  
   }
 Serial.print(w2);
 Serial.print('\t');Serial.print(x);
 Serial.println();  
} 

