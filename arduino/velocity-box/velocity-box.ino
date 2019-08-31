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

int period = 200;
unsigned long time_now = 0;

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

  // If you want, you can set accelerometer offsets
  mpu.setAccelOffsetX(7);
  mpu.setAccelOffsetY(7);
  mpu.setAccelOffsetZ(7);
  
  //checkSettings();

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  SD.remove("data.csv");
  myFile = SD.open("data.csv", FILE_WRITE);
  myFile.println("Temps;Acceleration_X;Acceleration_Y;Acceleration_Z;Vitesse_X;Vitesse_Y;Vitesse_Z");
  myFile.close(); 
  if(SD.exists("data.csv")){
    Serial.println("initialization done.");
  }else{
    Serial.println("cannot create data.csv");
  }

  // if the file opened okay, write to it:
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}

void loop()
{
  Vector normAccel = mpu.readNormalizeAccel();

  if(millis() > time_now + period){
    time_now = millis(); 
    myFile = SD.open("data.csv", FILE_WRITE);
    if (myFile) {
      Serial.print("Writing to data.csv...");
      Serial.print(" Xnorm = ");
      Serial.print(normAccel.XAxis);
      Serial.print(" Ynorm = ");
      Serial.print(normAccel.YAxis);
      Serial.print(" Znorm = ");
      Serial.println(normAccel.ZAxis);

      //add time
      myFile.print(time_now);
      myFile.print(";");
      //add accelaration 
      myFile.print((float)normAccel.XAxis);
      myFile.print(";");
      myFile.print((float)normAccel.YAxis);
      myFile.print(";");
      myFile.print((float)normAccel.ZAxis);
      myFile.print(";");
      

      //add velocity
      myFile.print((float)(normAccel.XAxis * (period / 1000)));
      myFile.print(";");
      myFile.print((float)(normAccel.YAxis * (period / 1000)));
      myFile.print(";");
      myFile.println((float)(normAccel.ZAxis * (period / 1000)));
      
      myFile.close();// close the file:
    }
    
  }
}
