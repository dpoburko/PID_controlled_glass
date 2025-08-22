#ifndef structures_H
#define structures_H
#include <Arduino.h>

//Defined values

// Analog input for measuring the voltage delivered to the glass heater via a 1/10 voltage divider
#define VOLTMETERPIN A2
// PWM PID output pin to MOSFET (adjust as needed)
#define PWMPINOUT 9
// Thermistor 1 pin
#define THERMISTOR1PIN A0
// Thermistor 2 pin
#define THERMISTOR2PIN A4

// must be intialized with the maxlength (e.g. serialMsg serial(25);)
struct serialMsg{
  int size;
  char* incoming; //dynamically assigned char array to hold the serial message
  char end;
  int ind;  //index
  int length; //replaces lengthOfMessage

  // Constructor to initialize size and allocate the array
  serialMsg(int bufferSize): size(bufferSize), end('.'), ind(0), length(0) {

      incoming = (char*)malloc(size * sizeof(char));  // Allocate memory dynamically

      if (incoming != nullptr) {
          memset(incoming, 0, size);  // Initialize buffer with null characters
      } else {
          Serial.println("Error: Memory allocation failed!");
      }
  }

  // Function to assign a single character by index
  void setCharAt(int index, char value) {
      if (index >= 0 && index < size) {
          incoming[index] = value;
      } else {
          Serial.println("Error: Index out of bounds!");
      }
  }

  // Read a character at a specific index
  //Note that values can be accessed directly as serialMsg.incoming[i], but this function adds out of bounds safety
  char getCharAt(int index) {
      if (index >= 0 && index < size) {
          return incoming[index];  // ✅ Read character at index
      } else {
          Serial.println("Error: Index out of bounds!");
          return '\0';  // Return a safe value
      }
  }

  // Destructor to free the allocated memory
  ~serialMsg() {
        delete[] incoming;
    }
  /*
   * In sketch:
   * serialMsg srl(25);
   * //Set characters by index
   * srl.setCharAt(0,'H');
   * get characters directly by index
   * 3rdLetter = serial.incoming[2];
   */
    
};

const int historyArraySize = 60;

//this is meant primarily to hold the hardware details of a thermistor
struct thermistor
{
  String name = "name";
  byte pin; 
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
    maxAutoIncrease = 7;    
    cummSetpointChange = 0; 
    autoSetpointChange = 0; 
  }
  
};

/* 
 *  To allow optional definition of default values at instantiation, the constructor needs to have all 
 *  those possilbe arguments with default values that are then passed to structure elements in the assignment list
*/
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
    double meanSquareError; // mse wrt set point
    int index;
    const int historySize;
    int setpointInterval = 150000;
    // Variable to store when the last glass setpoint update happened
    int setpointLastUpdate = 0;

    //These will be arrays that need to be defined at instantiation, so in the constructor
    double* history;
    double* slope;
    double* time;
    double* errorHistory; //mean square error array
    
    // Constructor to initialize size and allocate the array for history
    generalSensor(int arraySize, String aName = "sensor", double initVal = 0.0, float initSP = 0.0, int initSI = 5, long initSU = 1000, 
                  float initUL = 100, float initLL = 0.0) : 
    historySize(arraySize), name(aName),value(initVal), setpoint(initSP),upperLimit(initUL), lowerLimit(initLL),slopeInterval(initSI),slopeUnits(initSU)
    {
      history = new double[historySize];
      slope = new double[historySize];
      time = new double[historySize];
      errorHistory = new double[historySize];
      for (int i=0;i<historySize;i++) {
        history[i] = 0;
        slope[i] = 0;
        time[i] = millis();
        errorHistory[i] = 0;
      }
      setpointReached = false;
      setpointNoted = false; 
      historyFilled = false;
      prevValue = 0; 
      minimum = 10000000;
      maximum = -1000000;
      average = 0.0;
      deviation = 0; 
      meanSquareError = 0;
      index = 0;
    }  

    ~generalSensor() {
      delete[] history;
      delete[] slope;
      delete[] time;
      delete[] errorHistory;
    }
    
  };

struct PIDextras {
  double P;
  double I;
  double D;
  int mode; // AUTOMATIC = 1, MANUAL = 0
  double setpoint;
  double maxOutputNormal;
  double maxOutputHigh;
  double errorOutput;
  double deltaForMax;
  //less commonly changed    
  double maxOutputCurrent;
  double outputFromPID;
  double outputToDevice;
  double prevOutput;
  double maxInput; //e.g.voltage input to MOSFET, max flow rate of a gas etc.
  bool newValue;

  PIDextras(double aP, double aI, double aD, double aSetpoint, double amaxOutputNormal,double amaxOutputHigh,double aErrorOutput, int aMode):
          P(aP),I(aI),D(aD),setpoint(aSetpoint),maxOutputNormal(amaxOutputNormal),maxOutputHigh(amaxOutputHigh),errorOutput(aErrorOutput), mode(aMode)
  {
    newValue = false;
    prevOutput = 0;
    outputToDevice =0;
    outputFromPID = 0; 
    deltaForMax = 3 ;
    maxOutputCurrent = maxOutputNormal;
  }
};
 
//define errorCode array size here
struct errorCode 
{
  int ID;
  String name;
  bool active;
  bool buffered;
  bool userOverride; 
  bool silenced;
  unsigned long startTimer; // time error.active set to true. If acknowledged, set active = false and 
  unsigned long gracePeriod; // time since activated that another message won't be sent
  unsigned long graceTimer; // time grace period started
  unsigned long silencePeriod; // duration that silenced error sleeps
  unsigned long silenceTimer;  //time that sleep started
  unsigned long timeout; // not sure what this will be used for
};

  //a generic timer structure
  // instantiate as timers myTimer("count down", <duration in ms>);
  struct timers {
    char name[30];
    unsigned long start;
    unsigned long duration;
    bool active;
    unsigned long counter;
    //constructor
    timers(const char*  tempName = "timer", unsigned long tempDuration = 10000, bool tempActive = false  ): 
                   duration(tempDuration),active(tempActive) 
    {
      strncpy(name, tempName, sizeof(name) - 1);  // Safe copy
      name[sizeof(name) - 1] = '\0';              // Ensure null-termination
      counter = 0;
      start = 0;
    }
  };

#endif
