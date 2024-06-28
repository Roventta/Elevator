#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "./helper.h"

// object MPU
Adafruit_MPU6050 OMPU;
// Readings of acceleration, Gyro, and Temprature
sensors_event_t RAcc, RGyro, RTemp;

SerialPlotterWrapper<float> *Plotter;

void mpuSetup(){
  if(!OMPU.begin()){
    Serial.println("MPU6050 device missing");
    while(1){delay(10);}
  }
  OMPU.setAccelerometerRange(MPU6050_RANGE_8_G);
  OMPU.setGyroRange(MPU6050_RANGE_500_DEG);
  OMPU.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("MPU6050 activated");
}

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(10);}
  mpuSetup();
  Plotter = new SerialPlotterWrapper<float>(3);
  Plotter->addField("Gyro x", &RGyro.gyro.x);
  Plotter->addField("Gyro y", &RGyro.gyro.y);
  Plotter->addField("Gyro z", &RGyro.gyro.z);
}

void loop() {
  OMPU.getEvent(&RAcc, &RGyro, &RTemp);

  Plotter->emitPlot();

}
