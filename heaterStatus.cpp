#include <Arduino.h>
#include "heaterStatus.h"

//checking if PID controlled glass heater is working as expected


//Constructor
// pointers look like type* nameToPointer
heaterStatus::heaterStatus(type* nameToPointer) {

  //make local version of variables that are received as pointers from .ino file
  localVariable = nameToPointer;
  
}
    
