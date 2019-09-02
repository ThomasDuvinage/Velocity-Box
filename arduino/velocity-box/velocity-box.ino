/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Accelerometer Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>

#include <SPI.h>
#include <SD.h>

float period = 100.00;
unsigned long time_now = 0;
const float precision = 1.5;
float offset_X = 0,offset_Y = 0,offset_Z = 0, velocity[3] = {0}, acceleration[3] = {0};
int i;

float previous_velocity[3] = {0};

File myFile;

MPU6050 mpu;

void setup() 
{
  Serial.begin(9600);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  Vector normAccel = mpu.readNormalizeAccel();
  
  for(i = 0; i < 100;i++){
    offset_X += normAccel.XAxis;
    offset_Y += normAccel.YAxis;
    offset_Z += normAccel.ZAxis;
  }

  offset_X /= i;
  offset_Y /= i;
  offset_Z /= i;

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  SD.remove("data.csv");
  myFile = SD.open("data.csv", FILE_WRITE);
  myFile.println("Temps (ms);Acceleration_X (m/s^2);Acceleration_Y (m/s^2);Acceleration_Z(m/s^2);Vitesse_X(m/s);Vitesse_Y(m/s);Vitesse_Z(m/s)");
  myFile.close(); 
  if(SD.exists("data.csv")){
    Serial.println("initialization done.");
  }else{
    Serial.println("cannot create data.csv");
  }

  // if the file opened okay, write to it:
}

void loop()
{
  Vector normAccel = mpu.readNormalizeAccel();

  if(millis() > time_now + period){
    time_now = millis(); 
    myFile = SD.open("data.csv", FILE_WRITE);
    if (myFile) {

      acceleration[0] = (normAccel.XAxis - offset_X)*9.81;
      acceleration[1] = (normAccel.YAxis - offset_Y)*9.81;
      acceleration[2] = (normAccel.ZAxis - offset_Z)*9.81;

      
      if(acceleration[0] > precision || acceleration[0] < -precision ){
        //save velocity and plot it 
        velocity[0] = ((normAccel.XAxis - offset_X)*9.81) * (period / 1000.00);
        previous_velocity[0] = velocity[0]; 
      }else{
        velocity[0] = previous_velocity[0];//we add the previous velocity to the actual one 
      }
      
      if( acceleration[1] > precision  || acceleration[1] < -precision ){
        //save velocity and plot it 
        velocity[1] = ((normAccel.YAxis - offset_Y)*9.81) * (period / 1000.00);
        previous_velocity[1] = velocity[1]; 
      }else{
        velocity[1] = previous_velocity[1];//we add the previous velocity to the actual one 
      }
      
      if( acceleration[2] > precision  || acceleration[2] < -precision){
        //save velocity and plot it 
        velocity[2] = ((normAccel.ZAxis - offset_Z)*9.81) * (period / 1000.00);
        previous_velocity[2] = velocity[2];     
      }else{
        velocity[2] = previous_velocity[2];//we add the previous velocity to the actual one 
      }

      
      Serial.println("Writing to data.csv...");
      Serial.print(" Xnorm = ");
      Serial.print(acceleration[0]);
      Serial.print(" Ynorm = ");
      Serial.print(acceleration[1]);
      Serial.print(" Znorm = ");
      Serial.print(acceleration[2]);

      Serial.print(" X_velocity = ");
      Serial.print(velocity[0]);
      Serial.print(" Y_velocity = ");
      Serial.print(velocity[1]);
      Serial.print(" Z_velocity = ");
      Serial.println(velocity[2]);


      //add time
      myFile.print(time_now);
      myFile.print(";");
      //add accelaration 
      myFile.print((float)(normAccel.XAxis - offset_X)*9.81);
      myFile.print(";");
      myFile.print((float)(normAccel.YAxis - offset_Y)*9.81);
      myFile.print(";");
      myFile.print((float)(normAccel.ZAxis - offset_Z)*9.81);
      myFile.print(";");
      

      //add velocity
      myFile.print((float)((normAccel.XAxis - offset_X)*9.81) * (period / 1000.00));
      myFile.print(";");
      myFile.print((float)((normAccel.YAxis - offset_Y)*9.81) * (period / 1000.00));
      myFile.print(";");
      myFile.println((float)((normAccel.ZAxis - offset_Z)*9.81) * (period / 1000.00));
      
      myFile.close();// close the file:
    }
    
  }
}
