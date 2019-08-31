#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h> 

MPU6050 mpu;

int interval = 100;
unsigned long previousMillis = 0;

void checkSettings();

void setup() 
{
  Serial.begin(115200);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  delay(1000);
  Serial.print("We are going to set accelometer");

  // If you want, you can set accelerometer offsets
  mpu.setAccelOffsetX(0);
  mpu.setAccelOffsetY(0);
  mpu.setAccelOffsetZ(0);
  
  checkSettings();
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
  unsigned long currentMillis = millis();
  Vector normAccel = mpu.readNormalizeAccel();
  int Accel_data[3] = {0,0,0};
  int n_loop[3] = {0,0,0};

  Serial.print(" Xnorm = ");
  Serial.print(normAccel.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normAccel.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normAccel.ZAxis);

  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
      //save the average acceleration on sd card
      previousMillis = currentMillis;
  }
  else{
    Accel_data[0] += normAccel.XAxis; Accel_data[1] += normAccel.YAxis; Accel_data[2] += normAccel.ZAxis;
    n_loop[0]++ ; n_loop[1]++ ; n_loop[2]++;
  }


  
  delay(10);//you need this delay to not block the buffer
}