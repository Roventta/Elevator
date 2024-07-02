#ifndef ORIEN
#define ORIEN

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const float GRAV = 9.81;

class Kalman{
  private:
        /* Kalman filter variables */
    float Q_angle = 0.001f; // Process noise variance for the accelerometer
    float Q_bias = 0.003f; // Process noise variance for the gyro bias
    float R_measure = 0.03f; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle = 0.0f; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias = 0.0f; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate = 0; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix

  public:

    Kalman(){
      P[0][0] = 0.0f;
      P[0][1] = 0.0f;
      P[1][0] = 0.0f;
      P[1][1] = 0.0f;
    }

    float Fuse(float state_meas, float newRate, float dt){
        //step 1, integrating
        rate = newRate - bias;
        angle += dt * rate;

        // step 2, update covariance matrix
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        //step 4, get variance of angle
        float S = P[0][0] + R_measure;
        //step 5 
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = state_meas - angle;

        angle += K[0] * y;
        bias += K[1] * y;

        float P00 = P[0][0];
        float P01 = P[0][1];

        P[0][0] -= K[0] * P00;
        P[0][1] -= K[0] * P01;
        P[1][0] -= K[1] * P00;
        P[1][1] -= K[1] * P01;

        return angle;
    }
    
};

class OrientationProcessor{
  private:
    Adafruit_MPU6050 *OMPU;
    sensors_event_t RAcc, RGyro, RTemp;
    // measured error of gyro angular momentumn 
    float gphi_err = 0.0, gthe_err = 0.0, gpsi_err = 0.0;

    Kalman phi_fuser, the_fuser;

  public:
    //roll, pitch, yaw
    float Readings[3] = {0,0,0};
    float G_p=0.0, G_q=0.0, G_r= 0.0, A_x=0.0, A_y=0.0, A_z=0.0; 

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
      float tpf = timepassed * 0.000001;
      // make a reading
      this->OMPU->getEvent(&this->RAcc, &this->RGyro, &this->RTemp);
      A_x = this->RAcc.acceleration.x;
      A_y = this->RAcc.acceleration.y;
      A_z = this->RAcc.acceleration.z;
      G_p = this->RGyro.gyro.x-this->gphi_err;
      G_q = this->RGyro.gyro.y-this->gthe_err;
      G_r = this->RGyro.gyro.z-this->gpsi_err;

      // acceleration based measurement 
      float phi_acc=0, theta_acc=0, psi_acc=0;
      phi_acc = atanf(A_y/A_z);
      theta_acc = atanf(A_x/GRAV);

      this->Readings[0] = phi_fuser.Fuse(phi_acc, G_p, tpf);
      this->Readings[1] = theta_acc;
      
    }

};

#endif