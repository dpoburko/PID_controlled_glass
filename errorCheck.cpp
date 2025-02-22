#include <Arduino.h>
#include "errorCheck.h"

//This lirbary is meant to checking if the PID controlled glass heater is working as expected

//DP to Izzy
// See this URL for an overview of pointers (*) and references (&) https://www.arduino.cc/reference/en/language/structure/pointer-access-operators/dereference/
// also https://www.learncpp.com/cpp-tutorial/introduction-to-pointers/
// pointers in a constructor look like 'type* nameToPointer'

/*
errorMsg - needs to be passed to the library too. 
we also need all the values that errorCheck relies on
  glassTemperature
  maxGlassTemperature
  glassSetpoint
  PWMOutput
  PWMOutputLast
  _errorBuffer
  PWMOutputIfError
*/

String& aBuffer,
char& errorMessage,
//part of generalSensor lidTemperature
double& glassTemp,
double& maxGlasTemp,
double& glassSetPt,
double& glassTemperatureSlope
//part of PIDextras heaterValues
double& heaterOutput,
double& lastHeaterOutput,
double& PWMOutputIfError,

//re-write to pass 'generalSensor lidTemperature'

//Constructor
errorCheck::errorCheck(String& aBuffer,char& errorMessage ,generalSensor& alidTemperature, PIDextras& aheaterValues):
    _errorBuffer(&aBuffer),_errorMessage (&errorMessage),  lidTemperature(&alidTemperature), heaterValues(&aheaterValues)
  {

  _errorGraceTime = 180000; // move this to only be in the library, but could be updated from .ino via a function    

  
  //when initialized, fill these arrays 
    for (int i = 0; i < numberOfErrorCodes; i++) {
      _errorCodesActive[i]= false;
      _areErrorsAcknowledged[i] = true;
      _timesErrorsAcknowledged[i] = 0.0;
      _errorAcknowledgeTimeout[i] = _errorGraceTime; 
    }
 
}

errorCheck::graceTime(double graceTime) {
  //change the error grace time
  _errorGraceTime = graceTime;
  _errorBuffer += newError;
  _errorBuffer += " Error graceTime now ";
  _errorBuffer += String(graceTime,0);
  _errorBuffer += " ms.";
}

errorCheck::timeOut(int errorCode, double newTimeOut) {
  //change the error grace time
  _errorAcknowledgeTimeout[errorCode] = newTimeOut; 
  _errorBuffer += newError;
  _errorBuffer += " Error ";
  _errorBuffer += errorCode;
  _errorBuffer += " timeout now ";
  _errorBuffer += String(newTimeOut,0);
  _errorBuffer += " ms.";
}

errorCheck::update() {

  //this section should essentially replicate errorCheck()  from the .ino file
  newErrorCode = 0;
  
  //update the value at the memory address that thisErrorCode is pointing to with newErrorCode
  //this skips the step of the library needing to return the value of newErrorCode;
  *thisErrorCode = newErrorCode;
  
  //from errorCheck() from .ino
  double glassTempDiff = _glassTemp - _glassSetPt; // This was initially deltaFromSetPt and was never declared or set so I did that here

  // Assume all is well
  int newError = 0;
  
  // Check if thermistor is not connected. Temp will read as -273C
  if (_glassTemp < 0) 
  {
    newError = 1;
    
    // Assume PID will be full throttle. Override to 0.
    _heaterOutput = 0;
    if (!isErrorBuffered)
    {
      isErrorBuffered = true;
      _errorBuffer += newError;
      _errorBuffer += " Glass thermistor disconected. Output shut off.";
    }
    _errorCodesActive[0]= true;
  } 
  
  else if (_glassTemp >= _maxGlassTemp) 
  {
    newError = 2;
    // Gently throttle back the power output to reduce temperature
    // While too high output will halve recursively - eventually to 0
    _heaterOutput = PWMOutputIfError;
    if (!isErrorBuffered)
    {
      isErrorBuffered = true;
      _errorBuffer += newError;
      _errorBuffer += " Glass >= max temp. Throttling output to ";
      _errorBuffer += String(PWMOutputIfError, 0);
    }
    _errorCodesActive[1]= true;
  }

  // Thermal run away. Getting hotter than setpoint and beyond minor overshoot      
  // Do not trigger error if the program is just starting back up after a reset, as the glass will have cooled a bit.
  else if (glassTempDiff > 4.0 && (millis() - startUpTime) > errorTime) 
  {
    if (!isErrorBuffered)
    {
      newError = 3;
      errorCodePrevious = newError;
      _errorBuffer += newError;
      _errorBuffer += " Thermal runaway detected. Throttling output to ";			
      _errorBuffer += String(PWMOutputIfError, 0);
    }
    _errorCodesActive[2]= true;
  }
  
  else if (isHistoryArraysFilled) 
  {

    // Check if current power has led to stable temp below set point
    // If temp not reaching desired setpoint and slope is relatively flat, assume power is insufficient to reach setpoint
    // Slope (over 50 samples) when ramping up can be >11 C/min
    // When stable, always within +/-0.7C/min
    // When off, falling from 35C in ambient temp, goes to ~ -1C/min
    // Do not trigger when just starting up because the slope may be <-1, then power comes on and slope transits through 0. 
    
    if (glassTempDiff < -0.5 && _glassTemperatureSlope < 0.5 && _glassTemperatureSlope > -1 && (millis() - startUpTime) > _errorGraceTime) 
    {
      if (!isErrorBuffered)
      {
        newError = 4;
        _errorBuffer += newError;
  	    _errorBuffer += " Under-powered. Increase max output.";
      }
      _errorCodesActive[3]= true;
    }

    // Heater failure: temperature falling unexpectedly. Assume power not getting to the heating elements
    // Allow  period if system re-booting and below set point with transiently falling temp
    else if (glassTempDiff < -0.5 && _glassTemperatureSlope <= -1 && (millis() - startUpTime) > _errorGraceTime)  
    {
      if (!isErrorBuffered)
      {
        newError = 5;
        PWMOutput = 0;
  	    _errorBuffer += newError;
  	    _errorBuffer += " Heater failure. Output - off.";
        _errorCodesActive[4]= true;
      }
    }
    else 
    {
      _errorBuffer += newError;
    }
  }
	else 
  {
      _errorBuffer += newError;
  }
  
  *thisErrorCode = newErrorCode;
    
}
