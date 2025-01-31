#ifndef parseSerial_H
#define parseSerial_H
#include "Arduino.h"
#include "structures.h"
#include "errorCheck.h"

// receive serial buffers and handle all outcomes

class parseSerial {
  
  public:

    //constructor - should just need the main serial message
    parseSerial(serialMsg& aSerial, PID& aHeaterPID, IDextras& aHeaterValues ,String& aMsgBUffer, generalSensor& aTemp, generalSensor& bTemp);

    void parse();
      
  private:

    void parsePIDCmd();

    //void readMX300B();

    serialMsg* serialMain;
    PIDextras* heaterValues;
    PID* heaterPID;
    String* msgBuffer
    
    generalSensor* lidTemperature;
    generalSensor* enclosureTemperature;
};


#endif
