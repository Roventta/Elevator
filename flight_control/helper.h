#ifndef HELPER
#define HELPER

#define BUFSIZE 10

#include <Arduino.h>

template <typename T>
class SerialPlotterWrapper{

  private: 
  char** Fields;
  int MaxSize;
  int CurSize = 0;
  T** FieldReadings;

  public:
  SerialPlotterWrapper(int size){
    this->MaxSize = size;
    this -> Fields = (char**) malloc(size * sizeof(char*));
    for(int i=0; i<size; i++){
      this-> Fields[i] = (char*) malloc(BUFSIZE* sizeof(char));
    }

    this -> FieldReadings = (T**) malloc(size * sizeof(T*));
  }

  void addField(char *fieldName, T *fieldRef){
    if(this->CurSize < this->MaxSize){
      strcpy(Fields[CurSize], fieldName);
      FieldReadings[CurSize] = fieldRef;
      CurSize++;
    }
  }

  void emitPlot(){
    for(int i=0; i<CurSize; i++){
      Serial.print(Fields[i]);
      Serial.print(":");
      Serial.print(*FieldReadings[i]);

      if(i != CurSize-1){Serial.print(",");};
    }
    Serial.print("\n");
  }

};

#endif