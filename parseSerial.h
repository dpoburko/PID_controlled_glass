#ifndef parseSerial_H
#define parseSerial_H
#include "Arduino.h"
#include "structures.h"
#include "errorCheck.h"
#include <PID_v1.h>

// receive serial buffers and handle all outcomes

class parseSerial {
  
  //constructor - should just need the main serial message
  

  public:
    parseSerial(serialMsg& aSerial, PID& aHeaterPID, PIDextras& aHeaterValues, PID& aEnclosurePID, PIDextras& aEnclosureValues, String& aMsgBuffer, generalSensor& aTemp, generalSensor& bTemp);
    void parse();
      
  private:

    void parsePIDCmd();

    serialMsg* serialMain;
    PIDextras* heaterValues;
    PID* heaterPID;
    PIDextras* enclosureValues;
    PID* enclosurePID;
    
    String& msgBuffer;
    generalSensor* lidTemperature;
    generalSensor* enclosureTemperature;
};


#endif
