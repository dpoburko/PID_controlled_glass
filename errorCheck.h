#ifndef errorCheck_H
#define errorCheck_H
#include "Arduino.h"
#include "structures.h"

//The errorCodes array will be global using the extern declaration. It is compiled and memory allocated before classes or constructors
const int numberOfErrorCodes = 7;

extern errorCode errorCodes[numberOfErrorCodes]; 

//place variables and functions names here
class errorCheck {

  //Functions
  // in cpp heaterStatus::heaterStatus(double& glassTemp, double& maxGlasTemp, double& glassSetPt,char& errorMessage,double& heaterOutput, double& lastHeaterOutput, 
  //double& PWMOutputIfError, double& glassTemperatureSlope) 
  // collapse heater and thermistor values in structures to be passed

  public:
  
    //need to pass the lidPID structure, the lid structure, and msgBuffer
    errorCheck(String& amsgBuffer,String& aerrorBuffer ,generalSensor& alidTemperature, PIDextras& aheaterValues, long& astartUpTime);
    
    //these public variables should be accessible to the sketch as 
    int errorCode;
    int errorCodePrevious;

    void begin();
    
    void update();
    
  private:
    
    String& errorBuffer;
    String& msgBuffer;
    long& startUpTime;
    int newError;
    generalSensor* lidTemperature;
    PIDextras* heaterValues;

}; //close heaterStatus class
#endif
