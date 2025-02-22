#ifndef structures_H
#define structures_H

#include "Arduino.h"


const int historyArraySize = 60;

//this is meant primarily to hold the hardware details of a thermistor
struct thermistor
{
  String name = "name";
  byte pin; 
  //double temperature;
  //double maxTemperature;
  //double prevTemperature;
  //double prev2Temperature;
  //double setPoint;
  //bool setpointReached = false;
  //bool setpointNoted = false;
  //double deviation;
  //double history[historyArraySize];
  //bool historyFilled = false;
  //double slope;
  //int slopeInterval;
  int maxAutoIncrease; // maximum that set point can be increased when auto adapting to achieve a temp of another thermistor
  double cummSetpointChange; // cummulative automated changes from last used-defined setpoint
  double autoSetpointChange;
  
  double rNominal;
  double rSeries;
  double tNominal; // nominal temperature
  double bCoefficient; // beta coefficient

  thermistor(String thisName, byte thisPin, double thisrNominal = 10000, double thisrSeries = 9985, double thistNominal = 25.0, double thisCoef = 3435) :
   name(thisName), pin(thisPin), rNominal(thisrNominal), rSeries(thisrSeries), tNominal(thistNominal), bCoefficient(thisCoef)
  {
    maxAutoIncrease = 5;    
    cummSetpointChange = 0; 
    autoSetpointChange = 0; 
  }
  
};

//OK. To allow optional definition of default values at instantiation, the constructor needs to have all those possilbe arguments with default values that are then passed to 
//structure elements in the assignment list

// Initialization template
//generalSensor thisSensor(int arraySize, "name", value, setpoint, slopeInterval, slopeUnits(), upperLimit, lowerLimit) : 
struct generalSensor {
    String name;
    double value;
    double setpoint;
    double upperLimit;
    double lowerLimit;
    int slopeInterval;
    long slopeUnits; //in milliseconds, so 1000 = seconds, 60000 = minutes  
    bool setpointReached;
    bool setpointNoted;
    bool historyFilled;
    double prevValue;
    double minimum;
    double maximum;
    double average; //averaged over history length
    double deviation; //difference from setPoint
    int index;
    const int historySize;
    int setpointInterval = 150000;
    // Variable to store when the last glass setpoint update happened
    int setpointLastUpdate = 0;

    //These will be arrays that need to be defined at instantiation, so in the constructor
    double* history;
    double* slope;
    double* time;
    
    // Constructor to initialize size and allocate the array for history
    generalSensor(int arraySize, String aName = "sensor", double initVal = 0.0, float initSP = 0.0, int initSI = 5, long initSU = 1000, 
                  float initUL = 100, float initLL = 0.0) : 
    historySize(arraySize), name(aName),value(initVal), setpoint(initSP),upperLimit(initUL), lowerLimit(initLL),slopeInterval(initSI),slopeUnits(initSU)
    {
      history = new double[historySize];
      slope = new double[historySize];
      time = new double[historySize];
      setpointReached = false;
      setpointNoted = false; 
      historyFilled = false;
      prevValue = 0; 
      minimum = 10000000;
      maximum = -1000000;
      average = 0.0;
      deviation = 0; 
      index = 0;
    }  

    ~generalSensor() {
      delete[] history;
      delete[] slope;
      delete[] time;
    }
    
  };

struct PIDextras {
  double P;
  double I;
  double D;
  int mode; 
  double setpoint;
  double maxOutputNormal;
  double maxOutputHigh;
  double errorOutput;
  double deltaForMax;
  //less commonly changed    
  double maxOutputCUrrent;
  double outputFromPID;
  double outputToDevice;
  double prevOutput;
  bool newValue;

  PIDextras(double aP, double aI, double aD, double aSetpoint, double amaxOutputNormal,double amaxOutputHigh,double aErrorOutput, int aMode):
          P(aP),I(aI),D(aD),setpoint(aSetpoint),maxOutputNormal(amaxOutputNormal),maxOutputHigh(amaxOutputHigh),errorOutput(aErrorOutput), mode(aMode)
  {
    newValue = false;
    prevOutput = 0;
    outputToDevice =0;
    outputFromPID = 0; 
    deltaForMax = 3 ;
    maxOutputCUrrent = maxOutputNormal;
  }
};
  
  /*
  struct heater
  {
    double output;
    double prevOutput;
    double errorOutput;
    char name[20];

    heater(double aOutput = 0, double aPrevOutput = 0, double aErrorOutput = 10):
           outout(&aOutput),prevOutput(&aPrevOutput),errorOutput(&aErrorOutput) 
    {  }
  }
  */
  //define errorCode array size here
  const int numberOfErrorCodes = 5;
  
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
  };

  struct timers {
    unsigned long start;
    unsigned long duration;
    bool active;
    unsigned long counter;
    String name;
  };

#endif
