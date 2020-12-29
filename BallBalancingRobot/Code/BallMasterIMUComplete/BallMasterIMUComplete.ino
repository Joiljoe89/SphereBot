//Program for ballbot using GY-87 IMU, optical sensor and arduino I2C

#include<Wire.h>
#include<ps2.h>

//defining values and registers for GY-87 IMU
#define mpuaddr 0x68 //MPU6050 I2C address
#define accaddr 0x3B //Accelerometer register address and the first register
#define pwrmreg 0x6b //Power management register 1 for IMU
#define rtd 57.29 //Radian to degree conversion
#define fg 0.95 //Filter gain for complementary filter

//pins for optical sensor
#define dat 5 //data line
#define clk 6 //clock line

#define r 200.08 //r=0.1192m and rpm to rad/min conversion factor 9.54
#define l 0.139 //distance of from center of the robot

//Raw values from accelerometer, gyroscope and temperature sensor
int16_t axr,ayr,azr;//Raw values from accelerometer registers
int16_t gxr,gyr,gzr;//Raw values from gyroscope registers
int16_t tmp;//Raw value from onboard temperature sensor

//Processed values from accelerometer and gyroscope
float axp=0,ayp=0,azp=0;//Processed accelerometer values
float gxp=0,gyp=0,gzp=0;//Processed gyroscope values

//Variables for angle calculation
float ax=0,ay=0,az=0;//Angles from accelerometer
float gx=0,gy=0,gz=0;//Angles from gyroscope
float x=0,y=0,z=0;//Final angles using complementary filter

//Angles for robot
float theta=0; //Vertical angle
float phi=0; //Horizontal angle
float ctheta=0,stheta=0; //cosine and sine of theta
float cphi=0,sphi=0; //cosine and sine of phi

//Optical mouse variable
PS2 mouse(clk,dat);

//Integration time
#define dt 34

//Timer variable
unsigned long t=0;

//Variables for optical sensor
char stat,u,v;
float p=0,q=0; // velocity
float dx=0,dy=0; //displacement

//Velocities
float w1=0,w2=0,w3=0; //Stepper motor velocities
float vr=2; //Robot linear velocity
float omega=0; //Robot angular velocity

//Variables for arduino master slave
int g=0,h=0,i=0,j=0,k=0;


//The setup initializes the IMU, optical sensor and baud rate
void setup()
{
 //Setting up the IMU
 Wire.begin(); //beginning I2C transmission
 Wire.beginTransmission(mpuaddr); //Calling IMU address
 Wire.write(pwrmreg); //Moving the pointer towards the power management register
 Wire.write(0); //Waking up the IMU
 Wire.endTransmission(true); // Ending the I2C Transmisssion
 
 //Initializing the optical sensor
 mouse.mouse_init();

 //Setting the baud rate
 Serial.begin(9600);
}

//Function for reading values from IMU
void imuread()
{
 Wire.beginTransmission(mpuaddr); //Beginning I2C communication with IMU
 Wire.write(accaddr); //Moving pointer to first register address
 Wire.endTransmission(false); //Ending I2C communication with IMU
 Wire.requestFrom(mpuaddr,14,true); //Requesting 14 consecutive register values from IMU
 
 //Reading 16 bit values from IMU
 //Accelerometer raw values
 axr=Wire.read()<<8|Wire.read();
 ayr=Wire.read()<<8|Wire.read();
 azr=Wire.read()<<8|Wire.read();
 //Temperature raw value
 tmp=Wire.read()<<8|Wire.read();
 //Gyroscope raw values
 gxr=Wire.read()<<8|Wire.read();
 gyr=Wire.read()<<8|Wire.read();
 gzr=Wire.read()<<8|Wire.read();
}

//Function to calculate vertical angles
void verang()
{
 //Raw accelerometer values to processed values. The sensitivity chosen is +/-2g. The sentivity scale factor is 16384.0
 axp=axr/16384.0;
 ayp=ayr/16384.0;
 azp=azr/16384.0;

 //Raw gyroscope values to processed values. The sensitivity chosen is +/-250deg/s. The sensitivity scale factor is 131.0
 gxp=gxr/131.0;
 gyp=gyr/131.0;
 gzp=gzr/131.0;

 //Angle from accelerometer
 ax=(atan(axp/(sqrt((ayp*ayp)+(azp*azp))))*rtd);
 ay=(atan(ayp/(sqrt((azp*azp)+(axp*axp))))*rtd);
 az=(atan(azp/(sqrt((axp*axp)+(ayp*ayp))))*rtd);

 //Angle calculation from gyroscope
 gx=(gxp*dt*0.001)+x;
 gy=(gyp*dt*0.001)+y;
 gz=(gzp*dt*0.001)+z;

 //Complimentary filter
 x=(fg*gx)+((1-fg)*ax);
 y=(fg*gy)+((1-fg)*ay);
 z=(fg*gz)+((1-fg)*az);
}

//Function to calculate horizontal angle
void horang()
{
 phi=atan(y/x);
 if(x<0&&y<0)
   {
    phi=phi+0; 
   }
 else if(x>0&&y<0)
   {
    phi=PI+phi;        
   }
 else if(x>0&&y>0)
   {
    phi=PI+phi;
   }
 else
   {
    phi=(2*PI)+phi;  
   }
}

//Function for optical sensor
void mos()
{
 mouse.mouse_pos(stat,u,v); //Getting velocity of optical sensor
 p=float(u)*0.254; //x axis velocity;
 q=float(v)*0.254; //y axis velocity;
 //Calculating displacement
 dx=dx+(p*dt*0.001);
 dy=dy+(q*dt*0.001);  
}

//Function to calculate inverse kinematics
void invkin()
{
 cphi=cos(phi);
 sphi=sin(phi);
 ctheta=cos(theta);
 stheta=sin(theta);
 omega=l*omega;
 
 //Calculating stepper motor velocities using inverse kinematics
 w1=r*(omega-(vr*((cphi*((0.5*ctheta)-(0.866*stheta)))-(sphi*((0.5*stheta)+(0.866*ctheta))))));
 w2=r*(omega-(vr*((cphi*((0.5*ctheta)+(0.866*stheta)))+(sphi*((0.866*ctheta)-(0.5*stheta))))));
 w3=r*(omega-(vr*((sphi*stheta)-(cphi*ctheta)))); 
}

//Function for master slave communication
void mts()
{
 //StepperSlave 1
 if(w1>0.2)
   {
    k=0; 
   }
 else if(w1<-0.2)
   {
    k=1;
    w1=w1*-1;   
   }
 else
   {
    w1=0;   
   }
 g=int(w1);
 j=100*(w1-g);
 h=g/255;i=g%255;

 Wire.beginTransmission(4); //Transmitting for slave 1
 Wire.write(h);
 Wire.write(i);
 Wire.write(j);
 Wire.write(k);
 Wire.endTransmission();

 //StepperSlave 2
 if(w2>0.2)
   {
    k=0; 
   }
 else if(w2<-0.2)
   {
    k=1;
    w2=w2*-1;   
   }
 else
   {
    w2=0;   
   }
 
 g=int(w2);
 j=100*(w2-g);
 h=g/255;i=g%255;
 Wire.beginTransmission(2); //Transmitting for slave 2
 Wire.write(h);
 Wire.write(i);
 Wire.write(j);
 Wire.write(k);
 Wire.endTransmission();
 
 //StepperSlave 3
 if(w3>0.2)
   {
    k=0; 
   }
 else if(w3<-0.2)
   {
    k=1;
    w3=w3*-1;   
   }
 else
   {
    w3=0;   
   }
 
 g=int(w3);
 j=100*(w3-g);
 h=g/255;i=g%255;
 Wire.beginTransmission(3); //Transmitting for slave 3
 Wire.write(h);
 Wire.write(i);
 Wire.write(j);
 Wire.write(k);
 Wire.endTransmission(); 
}

//Function to display

void disp()
{
 Serial.print(" X:");Serial.print(dx);
 Serial.print(" Y:");Serial.print(dy);
 Serial.print(" A:");Serial.print(phi*rtd);
}

//Main loop
void loop()
{
 t=millis();
 Serial.println();
 imuread(); //IMU read
 verang(); //Vertical angle 
 horang(); //Horizonal angle
 invkin(); //Inverse kinematics
 mts(); //Master slave communication
 mos(); //Optical sensor
 disp(); //Display
 while((millis()-t)<dt){}
 Serial.print(" t:");Serial.print(millis()-t); 
}

