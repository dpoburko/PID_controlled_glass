#ifndef errorCheck_H
#define errorCheck_H

#include "Arduino.h"
#include "structures.h"

//place variables and functions names here



class errorCheck {

  //Functions
  // in cpp heaterStatus::heaterStatus(double* glassTemp, double* maxGlasTemp, double* glassSetPt,char* errorMessage,double* heaterOutput, double* lastHeaterOutput, double* PWMOutputIfError, double* glassTemperatureSlope) 
  // collapse heater and thermistor values in structures to be passed

  //need to pass the lidPID structure, the lid structure, and msgBuffer
  
  errorCheck(double*, double*, double*,char*,double*, double*, double*, double*);
  void graceTime(double); 
  void errorTimeOut(int,double);
  void update();

  public:
  //these public variables should be accessible to the sketch as 
  int errorCode;
  int errorCodePrevious;

  private:
    int _newErrorCode;
    const _numberOfErrorCodes = 5;
    bool _errorCodesActive[numberOfErrorCodes];
    double _timesErrorsAcknowledged[numberOfErrorCodes];
    bool _errorsAcknowledged[numberOfErrorCodes];
    double _errorAcknowledgeTimeout[numberOfErrorCodes];
    double _errorGraceTime;
    //Pointers that will draw a value from referenced variables in the sketch
    double *_glassTemp;
    double *_maxGlassTemp;
    double *_glassSetPt
    char *_errorMessage;
    double *_heaterOutput;
    double *_lastHeaterOutput;
    double *_PWMOutputIfError; 
    double *_glassTemperatureSlope;

  void setSampleTime(int);


}; //close heaterStatus class
#endif
