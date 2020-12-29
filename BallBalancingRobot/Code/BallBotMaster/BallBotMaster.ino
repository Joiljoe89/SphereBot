//Three OmniWheel InverseKinematics
//Robot Velocity vr(m/s)
//Angular velocity omega(deg/s)
//Robot Angle phi(deg)
//Yaw Angle theta(deg)
//stepper motor velocities in rpm (w1,w2,w3)

#include<Wire.h>
#include"ps2.h"

#define r 200.08 //  where r=0.1192m and rpm to rad conversion factor 9.54(9.54/0.1192)
#define l 0.1  // l*0.4 where l=.1m
#define dt 27

float w1=0,w2=0,w3=0;//Stepper motor velocities
float vr=0,theta=0,omega=0,phi=0;
float cphi=0,sphi=0,ctheta=0,stheta=0;
int x=0,y=0,z=0,d=0,s=0;
float c1=0,c2=0,c3=0;

unsigned long t=0;

char stat,a,b;
float dx=0,dy=0,i=0;

PS2 mouse(6,5);

void setup() 
{
 mouse.mouse_init();
 Wire.begin(); 
 Serial.begin(9600);
}

void loop() 
{
  Serial.print("Enter Robot Velocity(m/s):");
  while(Serial.available()==0)
       {
       }
  vr=Serial.parseFloat();     
  Serial.print(vr);Serial.println("m/s");
  
  Serial.print("Enter Robot Angle(deg):");
  while(Serial.available()==0)
       {
       }
  phi=Serial.parseFloat();     
  Serial.print(phi);Serial.println("degs");
  
  Serial.print("Enter Angular Velocity(deg/s):");
  while(Serial.available()==0)
       {
       }
  omega=Serial.parseFloat();     
  Serial.print(omega);Serial.println("deg/s");

  Serial.print("Enter Yaw Angle(deg):");
  while(Serial.available()==0)
       {
       }
  theta=Serial.parseFloat();     
  Serial.print(theta);Serial.println("deg/s");
  
  phi=phi*(PI/180);
  omega=omega*(PI/180);
  theta=theta*(PI/180);
  
  cphi=cos(phi);sphi=sin(phi);
  ctheta=cos(theta);stheta=sin(theta);
  omega=l*omega;  

 w1=r*(omega-(vr*((cphi*((0.5*ctheta)-(0.866*stheta)))-(sphi*((0.5*stheta)+(0.866*ctheta))))));
 w2=r*(omega-(vr*((cphi*((0.5*ctheta)+(0.866*stheta)))+(sphi*((0.866*ctheta)-(0.5*stheta))))));
 w3=r*(omega-(vr*((sphi*stheta)-(cphi*ctheta)))); 
  //StepperSlave 1
  if(w1>0.2)
    {
     s=0; 
    }
  else if(w1<-0.2)
    {
     s=1;
     w1=w1*-1;   
    }
  else
    {
     w1=0;   
    }
  x=int(w1);
  d=100*(w1-x);
  y=x/255;z=x%255;

  Wire.beginTransmission(4);
  Wire.write(y);
  Wire.write(z);
  Wire.write(d);
  Wire.write(s);
  Wire.endTransmission();

  //StepperSlave 2
  if(w2>0.2)
    {
     s=0; 
    }
  else if(w2<-0.2)
    {
     s=1;
     w2=w2*-1;   
    }
  else
    {
     w2=0;   
    }
  x=int(w2);
  d=100*(w2-x);
  y=x/255;z=x%255;

  Wire.beginTransmission(2);
  Wire.write(y);
  Wire.write(z);
  Wire.write(d);
  Wire.write(s);
  Wire.endTransmission();

  //StepperSlave 3
  if(w3>0.2)
    {
     s=0; 
    }
  else if(w3<-0.2)
    {
     s=1;
     w3=w3*-1;   
    }
  else
    {
     w3=0;   
    }
  x=int(w3);
  d=100*(w3-x);
  y=x/255;z=x%255;

  Wire.beginTransmission(3);
  Wire.write(y);
  Wire.write(z);
  Wire.write(d);
  Wire.write(s);
  Wire.endTransmission();
  i=0;
  while(i<185)
       {
        t=millis();
        mouse.mouse_pos(stat,a,b);
        a=float(a)*0.254;b=float(b)*0.254;
        dx=dx+(a*dt*0.001);
        dy=dy+(b*dt*0.001);
        i++;   
        Serial.print("\nX:");Serial.print(dx);
        Serial.print("\tY:");Serial.print(dy);
        while((millis()-t)<dt){}
        Serial.print("\tT:");Serial.print(millis()-t);
       }
  
  //Stopping rotation
  w1=0;w2=0;w3=0;
  //Slave 1
  x=int(w1);
  d=100*(w1-x);
  y=x/255;z=x%255;

  Wire.beginTransmission(4);
  Wire.write(y);
  Wire.write(z);
  Wire.write(d);
  Wire.write(s);
  Wire.endTransmission();

  //Slave 2
  x=int(w2);
  d=100*(w2-x);
  y=x/255;z=x%255;

  Wire.beginTransmission(2);
  Wire.write(y);
  Wire.write(z);
  Wire.write(d);
  Wire.write(s);
  Wire.endTransmission();

  //Slave 3
  x=int(w3);
  d=100*(w3-x);
  y=x/255;z=x%255;

  Wire.beginTransmission(3);
  Wire.write(y);
  Wire.write(z);
  Wire.write(d);
  Wire.write(s);
  Wire.endTransmission();
  Serial.print("\n\n");  
}
