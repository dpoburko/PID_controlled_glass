#ifndef heaterStatus_H
#define heaterStatus_H

#include "Arduino.h"

#define LIBRARY_VERSION 1.0.0

//place variables and functions names here

class heaterStatus {
heaterStatus

  public:
  //constants used in functions
  int errorCode;

  private:
    int *thisErrorCode; //probably not needed
    int *thisErrorCodePrevious; //probably not needed
    int newErrorCode;
    const numberOfErrorCodes = 5;
    bool errorCodesActive[numberOfErrorCodes];
    double timesErrorsAcknowledged[numberOfErrorCodes];
    bool areErrorsAcknowledged[numberOfErrorCodes];
    double errorGraceTime;

}; //close heaterStatus class
#endif

