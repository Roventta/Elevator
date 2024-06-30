#include "./helper.h"
#include "./orientation.h"
#include "./calibration.h"

// object MPU
Adafruit_MPU6050 OMPU;
// Readings of acceleration, Gyro, and Temprature
sensors_event_t RAcc, RGyro, RTemp;
SerialPlotterWrapper<double> *Plotter;
OrientationProcessor *Ori;
Calibrator *Calib;

int LastTimeStamp = 0;

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

void calib(){
  Calib = new Calibrator(&OMPU);
  float gerr[3];
  Calib->CalibGyro(gerr);
  Ori = new OrientationProcessor(&OMPU);
  Ori->SetErr(nullptr, gerr);
}

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(10);}
  mpuSetup();
  calib();
  Plotter = new SerialPlotterWrapper<double>(3);
  Plotter -> addField("Ori_Gyro_Phi", &(Ori->GPhi));
  Plotter -> addField("Ori_Gyro_Theta", &(Ori->GTheta));
  LastTimeStamp = micros();
}

void loop() {
  delay(5);

  Ori->PredictOri(micros() - LastTimeStamp);
  Plotter->emitPlot();
  LastTimeStamp = micros();
}
