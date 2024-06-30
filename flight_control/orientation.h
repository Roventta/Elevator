#ifndef ORIEN
#define ORIEN

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const float GRAV = 9.81;

class OrientationProcessor{
  private:
    Adafruit_MPU6050 *OMPU;
    sensors_event_t RAcc, RGyro, RTemp;
    float gphi_err = 0.0, gthe_err = 0.0, gpsi_err = 0.0; 

  public:
    //roll, pitch, yaw
    double Readings[3] = {0,0,0};
    double GPhi=0.0, GTheta = 0.0, GPsi = 0.0;

    OrientationProcessor(Adafruit_MPU6050 *mpu){
      this->OMPU = mpu;
    }

    void SetErr(float *aerr, float *gerr){
      this->gphi_err = gerr[0];
      this->gthe_err = gerr[1];
      this->gpsi_err = gerr[2];
    }

    // timepassed is in microsecond
    void PredictOri(int timepassed){
      // make a reading
      this->OMPU->getEvent(&this->RAcc, &this->RGyro, &this->RTemp);
      double ax = this->RAcc.acceleration.x,
            ay = this->RAcc.acceleration.y,
            az = this->RAcc.acceleration.z,
            phi_inc = this->RGyro.gyro.x-this->gphi_err,
            theta_inc = this->RGyro.gyro.y-this->gthe_err,
            psi_inc = this->RGyro.gyro.z-this->gpsi_err;

      // acceleration based predict 
      double phi_acc=0, theta_acc=0, psi_acc=0;
      phi_acc = atanf(ay/az);
      theta_acc = atanf(ax/GRAV);

      // gyro based predict
      this->GPhi += timepassed*phi_inc * 0.000001;
      this->GTheta += timepassed*theta_inc * 0.000001;
      this->GPsi += timepassed*psi_inc * 0.000001;

      this->Readings[0] = phi_acc;
      this->Readings[1] = theta_acc;
      
    }

};

#endif