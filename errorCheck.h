#ifndef errorCheck_H
#define errorCheck_H
#include "Arduino.h"
#include "structures.h"

//place variables and functions names here

class errorCheck {

  //Functions
  // in cpp heaterStatus::heaterStatus(double& glassTemp, double& maxGlasTemp, double& glassSetPt,char& errorMessage,double& heaterOutput, double& lastHeaterOutput, 
  //double& PWMOutputIfError, double& glassTemperatureSlope) 
  // collapse heater and thermistor values in structures to be passed

  //need to pass the lidPID structure, the lid structure, and msgBuffer
  
  //errorCheck(String& aBuffer, errorCode& someCode, double& glassTemp, double& maxGlasTemp, double& glassSetPt, char& errorMessage,double& heaterOutput, double& lastHeaterOutput, double& PWMOutputIfError, double& glassTemperatureSlope);
  errorCheck(String& aBuffer,char& errorMessage ,generalSensor& alidTemperature, PIDextras& aheaterValues);

  void graceTime(double); 
  
  void errorTimeOut(int,double);
  
  void update();

  public:
    //these public variables should be accessible to the sketch as 
    int errorCode;
    int errorCodePrevious;
    extern errorCode errorCodes[numberOfErrorCodes]; 
    
  private:
    
    String* _errorBuffer;
    char* _errorMessage;
    int _newErrorCode;
    generalSensor* lidTemperature;
    PIDextras* heaterValues;
    
  
  void setSampleTime(int);


}; //close heaterStatus class
#endif
