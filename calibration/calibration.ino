#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int CalibLength = 5000000;

struct Trial{
  float sum = 0;
  int reads = 0; 

  public:
    void tick(){this->reads += 1;};
    void add(float in){this->sum += in;};
    float getError(){
      float out = this-> sum / this->reads;  
      return out;
    };
};

Adafruit_MPU6050 OMPU;
sensors_event_t RAcc, RGyro, RTemp;

//gyro's x, y, z
Trial Tgx, Tgy, Tgz;

int TotalTime = 0;

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

void setup(){
  Serial.begin(115200);
  while(!Serial){delay(10);}
  mpuSetup();
}

void loop(){
  int start = micros();
  OMPU.getEvent(&RAcc, &RGyro, &RTemp);
  Tgx.add(RGyro.gyro.x);
  Tgy.add(RGyro.gyro.y);
  Tgz.add(RGyro.gyro.z);
  Tgx.tick();Tgy.tick();Tgz.tick();
  int timePassed = micros() - start;
  TotalTime += timePassed;

  if(TotalTime > CalibLength){
    Serial.print("average error of\n");
    Serial.print("gyro x = ");
    Serial.print(Tgx.getError());
    Serial.print(" gyro y = ");
    Serial.print(Tgy.getError());
    Serial.print(" gyro z = ");
    Serial.print(Tgz.getError());

    while(1){}
  }
}