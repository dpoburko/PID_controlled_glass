#ifndef structures_H
#define structures_H

#include "Arduino.h"


const int historyArraySize = 60;

  struct thermistor
  {
    byte pin; 
    double temperature;
    double maxTemperature;
    double prevTemperature;
    double prev2Temperature;
    double setPoint;
    bool setpointReached = false;
    bool setpointNoted = false;
    double deviation;
    double history[historyArraySize];
    bool historyFilled = false;
    double slope;
    int slopeInterval;
    int maxAutoIncrease; // maximum that set point can be increased when auto adapting to achieve a temp of another thermistor
    double cummSetpointChange; // cummulative automated changes from last used-defined setpoint
    double autoSetpointChange;
    String name;
    double rNominal;
    double rSeries;
    double tNominal; // nominal temperature
    double bCoefficient; // beta coefficient
  }

  struct PIDvalues {
    double P;
    double I;
    double D;
    mode; 
    double setpoint;
    double input;
    double output;
    double prevOutput;
    double maxNormalOutout;
    double maxHighOutput;
    double currMaxOutput;
    double deltaForMax;
  }
  
  struct heater
  {
    double output;
    double prevOutput;
    double errorOutput;
    char name[20];
  }
  
  struct errorCode {
    int ID;
    String name;
    bool active;
    double startTime;
    double graceTime; // time since activated that another message won't be sent
    bool silenced;
    double sleepTime; // duration that silenced error sleeps
    bool buffered;
    bool userOverride; 
  }

  struct timer {
    unsigned long start;
    unsigned long duration;
    bool active;
    char name[20];
  }

#endif
