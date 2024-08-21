#include <Arduino.h>
#include "heaterStatus.h"

//This lirbary is meant to checking if the PID controlled glass heater is working as expected

//DP to Izzy
// See this URL for an overview of pointers (*) and references (&) https://www.arduino.cc/reference/en/language/structure/pointer-access-operators/dereference/
// also https://www.learncpp.com/cpp-tutorial/introduction-to-pointers/
// pointers in a constructor look like 'type* nameToPointer'


/*
Error variables used in .ino
int errorCode = 0; //Note that the ino file doesn't really need to have it's own variable for errorCode. It should simply be able to pull heaterStatus.errorCode if errorCode is public
int errorCodePrevious = 0; // keep track of previous error code
double timeErrorAcknowledged =0; //for keeping track of when error was acknowledged 
bool isErrorAcknowledged = true;
bool isErrorBuffered = false;
bool isUserOverride = false;
long startUpTime;
errorMsg - needs to be passed to the library too. 

we also need all the values that errorCheck relies on
  glassTemperature
  maxGlassTemperature
  glassSetpoint
  PWMOutput
  PWMOutputLast
  errorBuffer
  PWMOutputIfError
  
  @this point, having to pass so many variables, I wonder if it would make sense to create a structure to hold the glass-related variables
  (https://www.teachmemicro.com/arduino-programming-structs/)
  struct heater
  {
       double temperature;
       double maxTemperature;
       double setPoint;
  }
  which would be accessed as
  heater glassLid;
  glassLid.temperature = 37.0;
*/



//Constructor
heaterStatus::heaterStatus(double* glassTemp, double* maxGlasTemp, double* glassSetPt,char* errorMessage,double* heaterOutput, double* lastHeaterOutput) {

  //the constructor receives variables and pointer values from the .ino file
  //these values are assigned to variables that are cast/initiated in the matching .h files
  //make local version of variables that are received as pointers from .ino file

  // the _ denotes a private variable
  _glassTemp = glassTemp; 
  _maxGlassTemp = glassTemp;
  _glassSetPt = glassSetPt;
  _errorMessage = errorMessage;
  _heaterOutput = heaterOutput;
  _lastHeaterOutput = lastHeaterOutput;

  //when initialized, fill these arrays 
    for (int i = 0; i < _numberOfErrorCodes; i++) {
      _errorCodesActive[i]= false;
      _areErrorsAcknowledged[i] = true;
      _timesErrorsAcknowledged[i] = 0.0;
    }

    _errorGraceTime = 180000; // move this to only be in the library, but could be updated from .ino via a function    
  
}

heaterStatus::update() {

  //this section should essentially replicate errorCheck()  from the .ino file
  newErrorCode =0;
  
  //update the value at the memory address that thisErrorCode is pointing to with newErrorCode
  *thisErrorCode = newErrorCode;
  //this skips the step of the library needing to return the value of newErrorCode;
  
}


