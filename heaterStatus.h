#ifndef heaterStatus_H
#define heaterStatus_H

#include "Arduino.h"

#define LIBRARY_VERSION 1.0.0

//place variables and functions names here

class heaterStatus {
heaterStatus

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
    double _errorGraceTime;
    //Pointers that will draw a value from referenced variables in the sketch
    double *_glassTemp;
    double *_maxGlassTemp;
    double *_glassSetPt
    char *_errorMessage;
    double *_heaterOutput;
    double *_lastHeaterOutput;

}; //close heaterStatus class
#endif

