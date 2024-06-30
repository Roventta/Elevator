#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class Calibrator{
  struct Trial{
    float sum = 0;
    unsigned int reads = 0; 

    public:
      void add(float in){
        this->sum += in;
        this->reads += 1;  
      };
      double getError(){
        double out = this-> sum / this->reads;  
        return out;
      };
  };

  private: 
    unsigned int TimePassed = 0;
    const unsigned int CalibTime = 5000000;
    Adafruit_MPU6050 *OMPU;
    Trial gPhi, gTheta, gPsi;
    sensors_event_t RAcc, RGyro, RTemp;


  public:

  Calibrator(Adafruit_MPU6050 *mpu){
    this->OMPU = mpu;
  }

  void CalibGyro(float *errs){
    Serial.println("Calibrating Gyro");
    TimePassed = 0;
    while(TimePassed < CalibTime){ 
      unsigned int t1 = micros();
      this->OMPU->getEvent(&this->RAcc, &this->RGyro, &this->RTemp);
      gPhi.add(this->RGyro.gyro.x);
      gTheta.add(this->RGyro.gyro.y);
      gPsi.add(this->RGyro.gyro.z);
      TimePassed += (micros() - t1);
    }

    errs[0] = gPhi.getError();
    errs[1] = gTheta.getError();
    errs[2] = gPsi.getError();

    Serial.println("Gyro Error collected:");
    Serial.print(" gyro x = ");
    Serial.print(errs[0]);
    Serial.print(" gyro y = ");
    Serial.print(errs[1]);
    Serial.print(" gyro z = ");
    Serial.print(errs[2]);
    Serial.print("\n");

    delay(3000);
  }

};