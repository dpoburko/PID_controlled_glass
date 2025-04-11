#include <Arduino.h>
#include "errorCheck.h"
#include "structures.h"

//This lirbary checks if the PID controlled glass heater is working as expected

//DP to Izzy
// See this URL for an overview of pointers (*) and references (&) https://www.arduino.cc/reference/en/language/structure/pointer-access-operators/dereference/
// also https://www.learncpp.com/cpp-tutorial/introduction-to-pointers/
// pointers in a constructor look like 'type* nameToPointer'


// Define the errorCodes array. Memory if this global variables is allocated before objects like classes or constructors
errorCode errorCodes[numberOfErrorCodes]; 

//Constructor:
//Note that the String variables can be used directly a reference, and don't need to be converted to pointere. Hence msgBuffer(amsgBuffer) not msgBuffer(&amsgBuffer)

errorCheck::errorCheck(String& amsgBuffer,String& aerrorBuffer, generalSensor& alidTemperature, PIDextras& aheaterValues, long& astartUpTime):
    msgBuffer(amsgBuffer),errorBuffer (aerrorBuffer),  lidTemperature(&alidTemperature), heaterValues(&aheaterValues), startUpTime(astartUpTime)
{
}

//this might need to move into the .ino setup loop as errorCheck::begin();

void errorCheck::begin(){

  //number of elements must match 'numberOfErrorCodes' at top of errorCheck.h
  String errorCodeNames[] = {"OK","no thermistor","over temperature","Thermal runaway","Under powered","Heater failure","PSU unplugged"};
  
  long tic = millis();
  //Prior to checking for errors, check if any errors have timed out and reset active state
  for (int i = 0; i < numberOfErrorCodes; i++) {
    errorCodes[i].name = errorCodeNames[i];
    errorCodes[i].ID = i;
    errorCodes[i].active = false;
    errorCodes[i].silenced = false;
    errorCodes[i].buffered = false;
    errorCodes[i].userOverride = false;
    errorCodes[i].startTimer = tic;
    errorCodes[i].gracePeriod = 180000;
    errorCodes[i].graceTimer = tic;
    errorCodes[i].silencePeriod = 180000;
    errorCodes[i].silenceTimer = tic;
  }
}

void errorCheck::update() {
  
  //errorBuffer is referenced from the .ino such that it can be directly updated without de-referencing

  //Prior to checking for errors, check if any errors have timed out and reset active state
  for (int i = 0; i < numberOfErrorCodes; i++) {
    if (errorCodes[i].active) {
      if ( (errorCodes[i].silenced) || (errorCodes[i].userOverride) ) {
        if ( (millis() - errorCodes[i].silenceTimer )> errorCodes[i].silencePeriod) {
          errorCodes[i].silenced = false;
          errorCodes[i].buffered = false;
          errorCodes[i].userOverride = false;
        }
      }
    }
  }
  
  //from errorCheck() from .ino
  //double glassTempDifference = lidTemperature->value - lidTemperature->setpoint; // This was initially deltaFromSetPt and was never declared or set so I did that here
  // repalced by lidTemperature->deviation
  
  // Assume all is well
  int newError = 0;
  
  // ERROR1: Check if thermistor is not connected. Temp will read as -273C
  if (lidTemperature->value < 0) 
  {
    // If error is silenced,when errorCheck.update() is called, the current output will not be modified 
    // This can't be placed in the initial if statement, because that would result in the 'else' action
    // that would set errorCodes[1].active = false;
    if (!errorCodes[1].silenced) {
      
      newError = 1;
      
      //if the error code was not previously acitve, start the error timer and set active == true
      if (!errorCodes[1].active) {
        errorCodes[1].active = true;  
        errorCodes[1].startTimer = millis();  
      }
      
      // Assume PID will be full throttle. Override to 0.
      // If error is silenced,when errorChecl.update() is called, the current output will not be modified 
      heaterValues->outputToDevice = 0;

  
      //manage buffering the error codes
      if (!errorCodes[1].buffered) {
        errorCodes[1].buffered = true;
        errorBuffer += newError;
        errorBuffer += " Glass thermistor disconected. Output shut off.";
      }
    }
  } else {
    //since each error is now handled as it's own array, there needs to be a mechanism to reset or turn off errorcodes
    if (errorCodes[1].active) {
      //reset error1
      errorCodes[1].active = false;  
      errorCodes[1].buffered = false;
    }
  }
  
  //ERROR2: Over-temperature
  if (lidTemperature->value >= lidTemperature->upperLimit) 
  {
    // If error is silenced,when errorCheck.update() is called, the current output will not be modified 
    // This can't be placed in the initial if statement, because that would result in the 'else' action
    // that would set errorCodes[2].active = false;
    if (!errorCodes[2].silenced) {
          
      newError = 2;
      // Gently throttle back the power output to reduce temperature
      // While too high output will halve recursively - eventually to 0
  
      heaterValues->outputToDevice = heaterValues->errorOutput;
      
      if (!errorCodes[2].active) {
        errorCodes[2].active = true;
        errorCodes[2].startTimer = millis();
      }
      
      //manage buffering the error codes
      if (!errorCodes[2].buffered) {
        errorCodes[2].buffered = true;
        errorBuffer += newError;
        errorBuffer += " Glass >= max temp. Throttling output to ";
        errorBuffer += String(heaterValues->errorOutput, 0);
      }
    }
  } else {
    //reset error2
    if (errorCodes[2].active) {
      errorCodes[2].active = false;
      errorCodes[2].buffered = false;
    }
  }

  // ERROR3: Thermal run away. Getting hotter than setpoint and beyond minor overshoot      
  // Do not trigger error if the program is just starting back up after a reset, as the glass will have cooled a bit.
  // Also watch for climbing temperature (positivie slope) if thermal run away
  if (lidTemperature->deviation > 4.0 && (millis() - startUpTime) > errorCodes[3].gracePeriod && lidTemperature->slope[lidTemperature->index-1]>0.1)  {

    // If error is silenced,when errorCheck.update() is called, the current output will not be modified 
    // This can't be placed in the initial if statement, because that would result in the 'else' action
    // that would set errorCodes[3].active = false;
    if (!errorCodes[3].silenced) {
      
      newError = 3;
  
      heaterValues->outputToDevice = heaterValues->errorOutput;
      
      if (!errorCodes[3].active) {
        errorCodes[3].active = true;
        errorCodes[3].startTimer = millis();
      }
      
      if (!errorCodes[3].buffered) {
        errorCodes[3].buffered = true;
        errorBuffer += newError;
        errorBuffer += " Thermal runaway detected. Throttling output to ";      
        errorBuffer += String(heaterValues->errorOutput, 0);
      }
    }
        
  } else {
    //reset error3
    if (errorCodes[3].active) {
      errorCodes[3].active = false;
      errorCodes[3].buffered = false;
    }
  }
  
  if (lidTemperature->historyFilled) {

    // Check if current power has led to stable temp below set point
    // If temp not reaching desired setpoint and slope is relatively flat, assume power is insufficient to reach setpoint
    // Slope (over n samples) when ramping up can be >11 C/min
    // When stable, always within +/-0.7C/min
    // When off, falling from 35C in ambient temp, goes to ~ -1C/min
    // Do not trigger when just starting up because the slope may be <-1, then power comes on and slope transits through 0. 

    double averageDeviation = lidTemperature->average - lidTemperature->setpoint;
    
    if (averageDeviation < -0.5 && lidTemperature->slope[lidTemperature->index-1] < 0.5 && lidTemperature->slope[lidTemperature->index-1] > -1 && (millis() - startUpTime) > errorCodes[4].gracePeriod) 
    {
  
      // If error is silenced,when errorCheck.update() is called, the current output will not be modified 
      // This can't be placed in the initial if statement, because that would result in the 'else' action
      // that would set errorCodes[4].active = false;
      if (!errorCodes[4].silenced) {
    
        newError = 4;
    
        if (!errorCodes[4].active) {
          errorCodes[4].active = true;
          errorCodes[4].startTimer = millis();
        }
        
        if (!errorCodes[4].buffered) {
          errorCodes[4].buffered = true;
          errorBuffer += newError;
          errorBuffer += " Under-powered. Increase max output.";      
        }

      }    
    } else {
      //reset error3
      if (errorCodes[4].active) {
        errorCodes[4].active = false;
        errorCodes[4].buffered = false;
      }      
    }

    // Heater failure: temperature falling unexpectedly. Assume power not getting to the heating elements
    // Allow  period if system re-booting and below set point with transiently falling temp
    if (lidTemperature->deviation < -0.5 && lidTemperature->slope[lidTemperature->index-1] <= -1 && (millis() - startUpTime) > errorCodes[5].gracePeriod)  {

      // If error is silenced,when errorCheck.update() is called, the current output will not be modified 
      // This can't be placed in the initial if statement, because that would result in the 'else' action
      // that would set errorCodes[5].active = false;
      if (!errorCodes[5].silenced) {

        newError = 5;
        heaterValues->outputToDevice = 0;
        
        if (!errorCodes[5].active) {
          errorCodes[5].active = true;
          errorCodes[5].startTimer = millis();
        }
        
        if (!errorCodes[5].buffered) {
          errorCodes[5].buffered = true;
          errorBuffer += newError;
          errorBuffer += " Heater failure. Output - off.";      
        }
      }
    } else {
      if (errorCodes[5].active) {
        errorCodes[5].active = false;
        errorCodes[5].buffered = false;
      }   
    }
    
  } // if lidTemperature->historyFilled

  //ERROR6: No heater power - i.e. voltage in = 0
  if (heaterValues->maxInput < 1) 
  {
    // If error is silenced,when errorCheck.update() is called, the current output will not be modified 
    // This can't be placed in the initial if statement, because that would result in the 'else' action
    // that would set errorCodes[2].active = false;
    if (!errorCodes[6].silenced) {
          
      newError = 6;
  
      heaterValues->outputToDevice = 0;
      
      if (!errorCodes[6].active) {
        errorCodes[6].active = true;
        errorCodes[6].startTimer = millis();
      }
      
      //manage buffering the error codes
      if (!errorCodes[6].buffered) {
        errorCodes[6].buffered = true;
        errorBuffer += newError;
        errorBuffer += " Glass PSU unplugged? V = ";
        errorBuffer += String(heaterValues->maxInput, 0);
      }
    }
  } else {
    //reset error6
    if (errorCodes[6].active) {
      errorCodes[6].active = false;
      errorCodes[6].buffered = false;
    }
  }

     
}
