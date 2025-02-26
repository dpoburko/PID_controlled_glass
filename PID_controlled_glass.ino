// *************************************************************************************************************************************
// INCUBATOR PID TEMPERATURE CONTROL
// *************************************************************************************************************************************

// For wiring details of voltmeter see:
// https://www.digikey.ca/en/maker/projects/how-to-make-a-simple-digital-voltmeter-with-an-arduino/082dff9d725549aea8bf84a7e302c1b2
// For thermistor details see:
// https://learn.adafruit.com/thermistor/using-a-thermistor

// *************************************************************************************************************************************
// LIBRARIES
// *************************************************************************************************************************************

// PID control library (must be downloaded)
#include <PID_v1.h>
// Wire library for I2C communication (must be downloaded)
#include <Wire.h>
// Steinhart library for the thermistor/calculations (must be downloaded)
#include "STEINHART.h"
#include "structures.h"
#include "errorCheck.h"
#include "parseSerial.h"

// *************************************************************************************************************************************
// PINS
// *************************************************************************************************************************************

// Analog input for measuring the voltage delivered to the glass heater via a 1/10 voltage divider
#define VOLTMETERPIN A2
// PWM PID output pin to MOSFET (adjust as needed)
#define PWMPINOUT 9
// Thermistor 1 pin
#define THERMISTOR1PIN A0
// Thermistor 2 pin
#define THERMISTOR2PIN A4
  
/* General approach to temperature control:
 *  1. temperature readings are stored in generalSensor structures - named as objectTemperature
 *  2. thermistor hardware and cummulitive deviations values are stored in thermistor structures - named as objectThermistor
 *  3. calculation of thermistor temperatures with the STEINHART library is assigned to objects named steinhart#
 */

// *************************************************************************************************************************************
// PID CONTROLLER VARIABLES
// *************************************************************************************************************************************
//generalSensor thisSensor(int arraySize, "name", value, setpoint, slopeInterval, slopeUnits(), upperLimit, lowerLimit) : 
  generalSensor lidTemperature(60, "lid temperature",22.0,55.0,10,1000,65.0,20.0);
//  lidTemperature.prevValue = 22.0; //this will be set after the first call to updateSensor();
  
  generalSensor enclosureTemperature(60, "enclosure temperature", 22.0, 32.0, 20, 1000, 40.0, 20.0);
//  enclosureTemperature.prevValue = 22.0;//this will be set after the first call to updateSensor();

  //PIDextra(double aP, double aI, double aD, double aSetpoint, double amaxOutputNormal,double amaxOutputHigh, double errorOutput int aMode)
  //PIDextras heaterValues(2, 96, 21, lidTemperature.setpoint, 40,45,10);
  PIDextras heaterValues(2.0, 96.0, 21.0, lidTemperature.setpoint, 40.0,45.0,10.0,1);
// Set glass temperature goal (in degrees Celsius)
//double glassSetpoint = 55.0; //need to figure out how to reference this to the thermistor setpoint in the new structure

// PID proportional, integral, and derivative constants (these values (2, 96, 21) are working well for the 5 mm thick glass)
//double PIDKp = 2, PIDKi = 96, PIDKd = 21;

// Define setpoint, input, and output variables (from PID library)
//double PIDSetpoint;
//double PIDInput;
//double PIDOutput;


// #define AUTOMATIC  1 & #define MANUAL  0 are defined in the PID library, meaning that they are visible to the sketch. No need to replace them. 
//enum enumPIDMode {
//  manualPIDMode,
//  automaticPIDMode
//};
//enumPIDMode PIDMode;

// Check if output has changed since last loop
//bool isNewPID = false; - replaced by  lidPD.newValue to be ready for the inclusion of a CO2 PID controller

//%%% Dumped into the generalSensor structure for easy hand off to libraries
// Interval for updating glass setpoint (10 minutes or 600000 milliseconds)
//int lidTemperature.setpointInterval = 150000;
// Variable to store when the last glass setpoint update happened
//int lidTemperature.setpointLastUpdate = 0;

// *************************************************************************************************************************************
// SAFETY VARIABLES
// *************************************************************************************************************************************

// Set maximum glass temperature allowed (in degrees Celsius)
//const int maxGlassTemperature = 65;

// Set current automated change in glass set point and maximum absolute increase in glass set point that the algorithm can set by itself
//int maxGlassAutoIncrease = 5;
//double autoGlassSetpointChange = 0;

// Distance from the setpoint where PID should start computing
//const double gapFromSetpointWherePIDStarts = 1.5;

// Set limits on output to pulse-width modulator (on an 8-bit scale) to limit total current based on limits of the PSU or Buck converter
// Aggressive limit used when the glass temperature is outside gapFromSetpointWherePIDStarts
//double maximumAggressivePIDOutput = 45.0;

// Conservative limit used when the glass temperature is inside gapFromSetpointWherePIDStarts
//double maximumConservativePIDOutput = 40.0;

// Set current maximum output to pulse-width modulator to the conservative limit initially
//double currentMaximumPIDOutput = maximumConservativePIDOutput;

//*** errorCodes Currently declared in errorCheck.h as extern and defined in errorCheck.cpp
//Initiate structure holding error codes
//const itn nErrorCodes = 6;
//double errorGraceTime = 180000; //in ms
//String codeNames = {"OK","no thermistor","over temperature","Thermal runaway","Under powered","Heater failure"};
//errorCode errorCodes[nErrorCodes]; //error codes now held in a structure that can be passed to errorCheck

String errorBuffer;
String msgBuffer;
String logBuffer;

long startUpTime = millis();


// *************************************************************************************************************************************
// STEINHART EQUATION VARIABLES - now all held in a structure for each thermistor. This allows for simple expansion to have additional thermistors
// *************************************************************************************************************************************

//it might be easier to keep the thermistor values to just the physical parameters of the thermistor
//then keep all temperature related values in a generalSensor structure for consistency and easy updating

//thermistor type should mostly refer to the hardware aspects of a thermistor rather than it's measurements.
//instantiate as thermistor(String thisName, byte thisPin, double thisrNominal = 10000, double thisrSeries = 9985, double thistNominal = 25.0, double thisCoef = 3435) 
thermistor lidThermistor("lid thermistor", THERMISTOR1PIN);
thermistor enclosureThermistor("enclosure thermistor",THERMISTOR2PIN);

//!!!! These values are now associated with the thermistor structures
// Thermistor resistance at nominal temperature (25 degrees Celsius)
//const long nominalResistance = 10000; 
// Temperature for nominal resistance (almost always 25 degrees Celsius)
//const long nominalTemperature = 25;
// B coefficient
//const long bCoefficient = 3435;
// Measured resistance for whatever series resistor (~10 kOhms) is used
//const long seriesResistance = 9985;

// *************************************************************************************************************************************
// VOLTAGE DIVIDER VARIABLES
// *************************************************************************************************************************************

// Coefficient for resistors 1 and 2 used for 1/10 voltage divider to detect input voltage
const double resistor1Coefficient = 9810;
const double resistor2Coefficient = 983;

// *************************************************************************************************************************************
// PULSE-WIDTH MODULATION VARIABLES
// *************************************************************************************************************************************

//I believe this is now ready to be depracated. 
//heater lidHeater; //structure holding current, and previous output to heater

// Pulse-width modulator output value needed to change glass temperature
//double PWMOutput = 0.0; // now in heaterValues strucutre

// Last pulse-width modulator output value
//double PWMOutputLast = 0.0; // now in heaterValues strucutre

//%%% to ErrorCheck library?
// Value used to hold glass temperature warm but safe
//double heaterValues->errorOutput = 10.0; // now in pidPID structure

// *************************************************************************************************************************************
// GLASS/THERMISTOR VARIABLES
// *************************************************************************************************************************************

// Variable to store the voltage of the glass read by the thermistor
double thermistorVoltage = 0.0; ///outdated????

// Set how often glass voltage will be read (in milliseconds)
int glassVoltageReadingInterval = 2000;

// Variable to store glass temperature (in degrees Celsius)
//double glassTemperature = 0.0;

// Variable to store previous glass temperature (in degrees Celsius)
//double previousGlassTemperature = 0.0;

// Variable to store the slopes of the glass and air temperature change
//int slopeInterval = 10;
//double glassTemperatureSlope = 0.0;
//double enclosureTemperature.valueSlope = 0.0;

// Variable to store air temperature
//double enclosureTemperature.value = 20.0; //enclosure

// Variable to store air temperature difference from setpoint
//double enclosureTemperature.valueDeviation = 0.0; replaced with enclosureTemperature.deviation

// Variable to store previous air temperature
//double previousAirTemperature = 0.0;

// Variable to store oldest air temperature
//double oldestAirTemperature = 0.0;

// Air temperature goal
//double enclosureTemperature.valueSetpoint = 37.0;

// Allowable difference betwee Air temperature and enclosureTemperature.valueSetpoint
//double enclosureTemperature.valueTolerance = 0.1;

// Boolean to check if the air temperature setpoint has been reached
//bool isAirTemperatureSetpointReached = false;

// Boolean to check if the air temperature setpoint has been noted
//bool isAirTemperatureSetpointNoted = false;

// yes if Air temperature < enclosureTemperature.valueSetpoint and reassessed at start-up and changes of enclosureTemperature.valueSetpoint
//bool isAirTemperatureClimbing = true;

// Size of arrays used for logging temperature and temperature error
//const int historyArraysSize = 60; // now in structure library

// Variable to store a counter for the index of the temperature and temperature error arrays
//int historyArraysIndex = 0;

// Variable to store when the index of the temperature and temperature error arrays ends
//int endHistoryArraysIndex = 0;

// Variable to store whether or not the temperature and temperature error arrays have been filled
//bool isHistoryArraysFilled = false;

// Array to store glass temperature
//double glassTemperatureHistory[historyArraysSize];

// Array to store glass temperature
//double enclosureTemperature.valueHistory[historyArraysSize];

// Array to store mean square error of glass temperature
//double mseGlassTemperatureHistory[historyArraysSize];

// Variable to store the mean square error of the glass temperature
//double mseGlassTemperature = 0.0;

// Boolean to check whether or not there is a new glass temperature
//bool isNewGlassTemperature = false; // - call to Steinhardt is a boolean 

// Boolean to check whether or not there is a new air temperature
//bool isNewsAirTemperature = false;


// *************************************************************************************************************************************
// SERIAL VARIABLES
// *************************************************************************************************************************************

//Initialize serialMsg
serialMsg serialMain(23); //hold buffer and variables related to serial monitor messages

// Maximum message length for serial communication (in characters)
//const int serialMaxMessageLength = 12;

// Character that will signal end of message from the PC
//char endSerialMessageCharacter = '.';

// Array for serial communication, of length determined by max message length variable
//char incomingSerial[serialMaxMessageLength];

// Variable to store message length
//int lengthOfSerialMessage = 0;

// How long to wait before displaying information on the serial
int serialDisplayInterval = 2000;

// Variable to store when the information was last displayed on the serial
int lastSerialDisplay = 0;

// not if output headers have been printed
bool headersPrinted = false;

// *************************************************************************************************************************************
// OTHER VARIABLES
// *************************************************************************************************************************************

// Indicate the voltage of the dev board digital outputs
const double boardVoltageOut = 5.0; //Arduino Uno is 5V, Adalogger is 3.3V

// Variable to store buck converter voltage
double buckConverterVoltage = 0;

// Set number of samples to take in order to get an average of the voltage data (needs to be a double for average to have decimals)
double numberOfSampleVoltageReadings = 11.0;

// Data logging
uint16_t nLogged = 0;

// *************************************************************************************************************************************
// CONSTRUCTORS
// *************************************************************************************************************************************

// Create thermistor object, including relevant information about the thermistor and the required constants for the Steinhart equation
//STEINHART steinhardt1(THERMISTOR1PIN, &glassTemperature, nominalResistance, nominalTemperature, bCoefficient, seriesResistance);
//STEINHART steinhardt2(THERMISTOR2PIN, &enclosureTemperature.value, nominalResistance, nominalTemperature, bCoefficient, seriesResistance);
STEINHART steinhardt1(lidThermistor.pin, &lidTemperature.value, lidThermistor.rNominal, lidThermistor.tNominal, lidThermistor.bCoefficient, lidThermistor.rSeries);
STEINHART steinhardt2(enclosureThermistor.pin, &enclosureTemperature.value, enclosureThermistor.rNominal, enclosureThermistor.tNominal, enclosureThermistor.bCoefficient, enclosureThermistor.rSeries);

// Create PID controller for the glass, including relevant information for the PID such as input, output, setpoint, and constants
//PID heaterPID(&lidTemperature.value, &PIDOutput, &glassSetpoint, PIDKp, PIDKi, PIDKd, DIRECT);
PID heaterPID(&lidTemperature.value, &heaterValues.outputFromPID, &lidTemperature.setpoint, heaterValues.P, heaterValues.I, heaterValues.D, DIRECT);

//!!!!!! About controlling the glass temp. In principle, there's no reason that we couldn't control the enclosure temp, where the output is the glass temp if we use
// a sufficiently slow update cycle (every 60 - 90 seconds?) and share glass max time with the enclosure PID.
// looks pretty easy! Just need time to tune the PID values. 
//PID enclosurePID(&enclosureTemperature.value, &lidTemperature.setpoint,&enclosureTemperature.setpoint, enclosureValues.P, enclosureValues.I, enclosureValues.D, DIRECT);


//instantiate the errorCheck library with references to needed variables. Note that errorCodes is a global variable, so does not need to be transfered.

//250224 - I am not convinced that errorCheck and parseSerial instantiation don't need to be references

errorCheck errorCheck(msgBuffer,errorBuffer, lidTemperature, heaterValues, startUpTime);

parseSerial parseSerial(serialMain, heaterPID, heaterValues, msgBuffer, lidTemperature, enclosureTemperature);

// *************************************************************************************************************************************
// SETUP
// *************************************************************************************************************************************

void setup()
{
  
  while(!Serial){}  
  // Open serial port, set data rate to 57600 bps
  Serial.begin(57600);	
  
  // Delay helps fix serial glitch
  delay(2000);

  // Initialize communication with I2C devices
  Wire.begin();

  errorCheck.begin();

  // Initialize buffers
  errorBuffer.reserve(256);
  msgBuffer.reserve(128);
  logBuffer.reserve(512);
  
  //this needs to be done in setup, not prior to >>>>>>> Now done in errorCheck.cpp
//  for (i=0 ; i < nErrorCodes; i++) {
//    errorCodes[i].ID = i;
//    name = codeNames[i];
//    active = false;
//    startTime = 0;
//    graceTime = errorGraceTime;
//    silenced = false;
//    sleepTime = 180000;
//    buffered = false;
//    userOverride = false;
//  }
 
  // Initialize PID (automaticPIDMode mode)
  //PIDMode = automaticPIDMode;
  heaterPID.SetMode(AUTOMATIC); //note that AUTOMATIC is inherited as #define from the PID library.

  // Link PID setpoint with glass setpoint
  //PIDSetpoint = glassSetpoint;   //vestigial, not longer used

  // Set up PID output limits  
  heaterPID.SetOutputLimits(0, heaterValues.maxOutputHigh); 
  heaterPID.SetSampleTime(glassVoltageReadingInterval); 

  // Set frequency of voltage readings for thermistor 1
  steinhardt1.setSampleTime(glassVoltageReadingInterval);

  // Read the temperature from thermistor 1
  steinhardt1.read();

  // Repeat for thermistor 2
  steinhardt2.setSampleTime(glassVoltageReadingInterval);
  //isNewsAirTemperature = steinhardt2.read();
  enclosureTemperature.prevValue = enclosureTemperature.value;
  //enclosureTemperature.prev2Temperature = enclosureTemperature.value; //deprecated 250123
  //oldestAirTemperature = enclosureTemperature.value;
  // = enclosureTemperature.value;

  // Print initialization message and temperature to serial
  Serial.print("   ===== PID controlled glass heater =====================");
  Serial.println("   Stage-Top Incubator component"); 
  Serial.print("   Initial Temperature:");
  Serial.println(lidTemperature.value, 2);

   // Variable to store when the last glass setpoint update happened
   lidTemperature.setpointInterval = 150000;
   lidTemperature.setpointLastUpdate = 0;


  startUpTime = millis();
}

// *************************************************************************************************************************************
// MAIN LOOP
// *************************************************************************************************************************************

void loop() 
{

  // Call STEINHART library, check if value is updated  
  //isNewGlassTemperature = steinhardt1.read();  //...since steinhardt.read() is a bool, we can simple use the function call as the logical test
  if (steinhardt1.read()==true) // this should both take a reading and service as the logical test.
  // Add the new temp reading to the history array if a new value is available
  //if (isNewGlassTemperature) 
  {
    // Read temperature from thermistor 2 - air temperature
    steinhardt2.read();

    // Get the voltage from the tunable buck converter (what do we do with this?)
    GetBuckConverterVoltage(buckConverterVoltage);

    // Check to make sure the glass temperature data point makes sense, and reassign the value if necessary
    //250123 - DP revised this function to work for any generalSensor structure
    RemoveErroneousSensorReadings(lidTemperature,0.25);
    RemoveErroneousSensorReadings(enclosureTemperature,0.25);

    // Log the glass temperature in an array
    //AppendToHistory(lidTemperature.value,enclosureTemperature.value ); //deprecated
    //sensorUpdate handles slope calculation and history appending
    sensorUpdate(lidTemperature);
    sensorUpdate(enclosureTemperature);

    //sensorUpdate handles slope calculations and current average value
    /*
    // If the arrays are not yet full, calculate the slope of the glass temperature change
    if (lidTemperature.historyFilled == false) 
    {
      // If this is the first value in the array, the slope cannot yet be calculated, so set it to zero
      if (lidTemperature.index == 0) 
      {
        //glassTemperatureSlope = 0.0;
        //enclosureTemperature.valueSlope = 0.0;
        lidThermistor.slope = 0.0;
        enclosureTemperature.slope = 0.0;      
        
      } 
      else 
      {
        // In degrees/minute
        //glassTemperatureSlope = (glassTemperature - glassTemperatureHistory[0]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
          //glassTemperatureSlope = (glassTemperature - glassTemperatureHistory[(historyArraysIndex-slopeInterval)%historyArraysSize]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
          //enclosureTemperature.valueSlope = (enclosureTemperature.value - enclosureTemperature.valueHistory[(historyArraysIndex-slopeInterval)%historyArraysSize]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
          lidThermistor.slope = (lidTemperature.value - lidTemperature.history[(historyArraysIndex-lidThermistor.slopeInterval)%historyArraysSize]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
          enclosureTemperature.slope = (enclosureTemperature.value - enclosureTemperature.history[(historyArraysIndex-enclosureTemperature.slopeInterval)%historyArraysSize]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
      }
    }
    else 
      {
        // In degrees/minute
        lidThermistor.slope = (lidTemperature.value - lidTemperature.history[endHistoryArraysIndex]) / (historyArraysSize * glassVoltageReadingInterval/60000);
        enclosureTemperature.slope = (enclosure.temperature - enclosureTemperature.history[endHistoryArraysIndex]) / (historyArraysSize * glassVoltageReadingInterval/60000);
      }
    

    // Compute the average of the arrays
    GetArrayAverage(mseGlassTemperatureHistory, mseGlassTemperature);
    */
    
    // If glass temperature is far from the setpoint, set the maximum PID output to a pre-determined aggressive value
    if ( (lidTemperature.setpoint - lidTemperature.value) >  heaterValues.deltaForMax )
    {
      // If the current maximum PID output is not already set to aggressive, set it to aggressive and print a message to the serial
      if (heaterValues.maxOutputCurrent != heaterValues.maxOutputHigh) 
      {
        // Set the current PID output to aggressive
        heaterValues.maxOutputCurrent = heaterValues.maxOutputHigh;
        heaterPID.SetOutputLimits(0, heaterValues.maxOutputCurrent);

        // Print a message to the serial
        msgBuffer += " Switching to heaterValues.maxOutputCurrent_";
	      msgBuffer += String(heaterValues.maxOutputCurrent);
      } 
    }
    // If the glass temperature is close to the setpoint, set the maximum PID output to a pre-determined conservative value 
    else 
    {
      // If the current maximum PID output is not already set to conservative, set it to conservative and print a message to the serial
      if (heaterValues.maxOutputCurrent != heaterValues.maxOutputNormal) 
      {
        // Set the current PID output to conservative
        heaterValues.maxOutputCurrent = heaterValues.maxOutputNormal;
        heaterPID.SetOutputLimits(0, heaterValues.maxOutputCurrent);

        // Print a message to the serial
        msgBuffer += " Switch to heaterValues.maxOutputNormal_";
	      msgBuffer += String(heaterValues.maxOutputCurrent);
      }
    }
  }
  
  // If the PID is in automaticPIDMode, compute output
  if (heaterPID.GetMode() == 1) 
  {
    // Make sure it is time to recalculate
    //heaterValues->newValue = heaterPID.Compute();

    // Define output of PID as the pulse-width modulated output (will be at 0 the first time around as the PID needs two data points for computation)
    //if (heaterValues->newValue == true) 
    if (heaterPID.Compute() == true) 
    {
    	heaterValues.outputToDevice = heaterValues.outputFromPID;
    	// handoff the PID's intended output to the heater structure
    	// Perform a series of tests on thermistor reading and temperature vs. setpoint and over time

     //error check will likely neeed a bunch of structures passed to it: buffers, lidTemperature, enclosureTemperature, CO2 eventually, heaterValues
     errorCheck.update();
     
    } 
  }

   //%%% @Izzy, I have just noticed that we have no errorChecking when in manual mode. This seems prone to troubles. Let's rethink this.
   // DP - isAirTempuratureCliminb might more easily be determined as a slope from the enclosureTemperature.valueHistory

  //It might be helpful to log whether temp should be increasing or decreasing to reach setpoint
  // currently, there is not enclosure.setpointReached = false. 
  // this needs to be changed if setpoint changes
  
  // Check if the air temperature is rising or fall
  //If rising....
  if ( enclosureTemperature.slope[enclosureTemperature.index-1] > 0.2  )
  //if (isAirTemperatureClimbing == true) //should eventually be replaced
  {
    //if setpoint reached.
    if (enclosureTemperature.value >= enclosureTemperature.setpoint)
    {
      if (enclosureTemperature.setpointReached = false) {
        enclosureTemperature.setpointReached = true;
      }
      if (enclosureTemperature.setpointNoted == false) {
        msgBuffer += " air temperature set point reached"; 
        enclosureTemperature.setpointNoted = true;
      }
    }
    //what if setpoint not reached?

  //If temp falling
  } else {
    
    //assume air temp needs to fall to reach set point
    if (enclosureTemperature.value <= enclosureTemperature.setpoint)
    {
      if (enclosureTemperature.setpointReached = false) {
        enclosureTemperature.setpointReached = true;
      }
      if (enclosureTemperature.setpointNoted == false) {
        msgBuffer += " air temperature set point reached"; 
        enclosureTemperature.setpointNoted = true;
      }
    }
  }
  
  // Check to see if the glass setpoint needs to be updated, only if enclosureTemperature.valueSetpointReached is true
  if (enclosureTemperature.setpointReached)
  {
    CheckGlassSetpoint();
  }
	
  // If the output from the PID is different from the previous output, adjust the pulse-width modulator duty cycle
  if (heaterValues.outputToDevice!= heaterValues.prevOutput) 
  {
    // Adjust the duty cycle of the PWM pin connected to the MOSFET fron the PID output if the value has changed
    analogWrite(PWMPINOUT, heaterValues.outputToDevice);

    // Log the new output value as the last output value received
    heaterValues.prevOutput = heaterValues.outputToDevice;
  }

  // Set the current time to the number of milliseconds the Arduino has been powered up for
  int currentTimeMilliseconds = millis();

  // If enough time has passed (based on the serial display interval), print all parameters
  if ((currentTimeMilliseconds - lastSerialDisplay) >= serialDisplayInterval) 
  {
    // Print buck converter voltage, glass temperature, and PWMOutput to serial
    PrintParametersToSerial();

    // Log the current time in milliseconds as the most recent time in which the parameters were printed to the serial
    lastSerialDisplay = currentTimeMilliseconds;
  }

  //%%% If ErrorCheck library goes smoothly, consider moving SerialParsing to library with pointers to all variables control/manipulated.
  // If the user has typed a message to the serial monitor, read the message

  if (Serial.available() > 0) 
  {

    parseSerial.parse();
    /*
    // Read the number of characters until end character is reached or maxMessageLength character is read
    lengthOfSerialMessage = Serial.readBytesUntil(endSerialMessageCharacter, incomingSerial, serialMaxMessageLength);

    // If the message length is less than the maximum allowed message length, read the message
    if (lengthOfSerialMessage <= serialMaxMessageLength) 
    {
      // If user inputted P, T, or E, go to parse function to read the next message inputted
      if (incomingSerial[0] == 'P' || incomingSerial[0] == 'T'|| incomingSerial[0] == 'S'|| incomingSerial[0] == 'E' || incomingSerial[0] == 'L') 
      {
        // Go to parse function to read the next message inputted
        ParsePIDCmd();

        // User override
        isUserOverride = true;  
      }
      //%% This will be obsolete when errorCheck moves to library

      // If user inputted h, H, or ?, print out available serial commands
      else if (incomingSerial[0] == 'h' || incomingSerial[0] == 'H' || incomingSerial[0] == '?') 
      {
        Serial.println("\n >>>> PID Serial Commands <<<<\n");
        Serial.println(" End commands with '.' for faster responses");
        Serial.println(" Interacting with the PID controller");
        Serial.println("   Pma - call  heaterPID.SetMode(AUTOMATIC");
        Serial.println("   Pmm - call  heaterPID.SetMode(MANUAL)");
        Serial.println("   Pmg - call  heaterPID.GetMode()");
        Serial.println("   Psnnn - update glassSetpoint as nn.n degrees");
        Serial.println("   Pannn - update enclosureTemperature.valueSetpoint as nn.n degrees");
        Serial.println("   Ponnn. - manually set PWMOutput (only usual in manualPIDMode mode");
        Serial.println("   Plnnn. - call heaterPID.SetOutputLimits(0, nnn)");
        Serial.println("   Phnnn. - updated holding PWM");
        Serial.println("   Ptpnnn. - heaterPID.SetTunings(nnn,iii,ddd)");
        Serial.println("   Ptinnn. - heaterPID.SetTunings(ppp,nnn,ddd)");
        Serial.println("   Ptdnnn. - heaterPID.SetTunings(ppp,iii,nnn)");
        Serial.println("   Pdnnnn. - call heaterPID.SetSampleTime(nnnn) & STEINHART::setSampleTime(nnnn)");
        Serial.println("   L - List column headers");
        Serial.println("   St - Show temperature history in serial monitor");
        Serial.println("   Se - Show mse history in serial monitor");
        Serial.println("   Sa - Show air temp history in serial monitor");
	      Serial.println("   Eax - Acknowledge error #n and ignore for grace time period");
	      Serial.println("   Egm - update error gracetime period in minutes");
        Serial.println("   >>>>>>>>>>>>>><<<<<<<<<<<<<<<<");
      }
      // If another message is inputted, print to the serial that the message was not recognized
      else 
      {
        Serial.println("   Serial command not recognized. Press 'h' for help");
      }
    }
    */

  }


//since glass and air temp are logged in history indices, we could just skip this and pull the values from arrays when needed
  // Set the previousGlassTemperature value to the current glass temperature
  //  previousGlassTemperature = glassTemperature;
  lidTemperature.prevValue = lidTemperature.value;
  // Set the previousAirTemperature value to the current air temperature
  //previousAirTemperature = enclosureTemperature.value;
  //enclosureTemperature.prev2Temperature = enclosureTemperature.prevValue; //Deprecated - need to ref enclosureTemperature.history
  
  // Set the oldestAirTemperature value to the previousAirTemperature value
  //oldestAirTemperature = previousAirTemperature;
  enclosureTemperature.prevValue = enclosureTemperature.value;

  // Delay determines how often loop repeats
  delay(20); 
}

// *************************************************************************************************************************************
// FUNCTIONS
// *************************************************************************************************************************************

// Read serial message, consider moving this to a library
/*
void ParsePIDCmd()
{
  if (incomingSerial[0] == 'P') 
  {
    if (incomingSerial[1] == 'm') 
    {
      if (incomingSerial[2] == 'a') 
      {
        //PIDMode = automaticPIDMode;
        heaterPID.SetMode(AUTOMATIC);
        msgBuffer += "PID set to Automatic"; 
      }

      if (incomingSerial[2] == 'm') 
      {
        //PIDMode = manualPIDMode;
        heaterPID.SetMode(MANUAL); 
        msgBuffer += " PID set to MANUAL"; 
      } 
      
      if (incomingSerial[2] == 'g') 
      {
        //PIDMode = manualPIDMode;
        //heaterPID.SetMode(PIDMode); 
        msgBuffer += " PID mode is ";
				msgBuffer += heaterPID.GetMode();  
      }         
    } 
    else if (incomingSerial[1] == 'o') 
    {
      char newOutput[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      PWMOutput = atof(newOutput);
      msgBuffer += " PWMoutput now "; 
			msgBuffer += String(PWMOutput);  
    }
    else if (incomingSerial[1] == 'e') 
    {
      char newOutput[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      heaterValues->errorOutput = atof(newOutput); 
			msgBuffer += " heaterValues->errorOutput now "; 
			msgBuffer += String(heaterValues->errorOutput, 1);
           
    }
    else if (incomingSerial[1] == 's') 
    {
      char newSetpoint[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      lidTemperature.setpoint = atof(newSetpoint)/10.0;
      msgBuffer += " glassSetpoint now ";
			msgBuffer += String(lidTemperature.setpoint,1); 
    }
    else if (incomingSerial[1] == 'a') 
    {
      char newSetpoint[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      enclosureTemperature.setpoint = atof(newSetpoint)/10.0;
      msgBuffer += " enclosureTemperature.setpoint now ";
      msgBuffer += String(enclosureTemperature.setpoint,1); 

      if (enclosureTemperature.value < enclosureTemperature.setpoint) {
        isAirTemperatureClimbing  = true;
        msgBuffer += " isAirTemperatureClimbing  = true";
      } else {
        isAirTemperatureClimbing  = false;
        msgBuffer += " isAirTemperatureClimbing  = false";
      }
    
      enclosureTemperature.setpointNoted = false;

    }
    
    else if (incomingSerial[1] == 'a') 
    {
      char newSetpoint[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      
      double newAirTemperatureSetpoint = atof(newSetpoint)/10.0;

      
    }
    
    else if (incomingSerial[1] == 'l') 
    {
      char upper[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      maximumAggressivePIDOutput = atof(upper);
      heaterPID.SetOutputLimits(0,maximumAggressivePIDOutput);
      msgBuffer += " heaterPID.SetOutputLimits(0,";
			msgBuffer += String(maximumAggressivePIDOutput);
			msgBuffer += ")";     
    }
    else if (incomingSerial[1] == 'h') 
    {
      char holding[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      maximumConservativePIDOutput = atof(holding);
      msgBuffer += " currentMaximumPIDOutput now";
			msgBuffer += String(currentMaximumPIDOutput, 2);     
    }
    else if (incomingSerial[1] == 'd') 
    {
      char newDelay[4] = {incomingSerial[2],incomingSerial[3],incomingSerial[4],incomingSerial[5]};
      int nd = atol(newDelay);
      heaterPID.SetSampleTime(nd);
      msgBuffer +=" PID interval now ";
			msgBuffer += String(newDelay, 2);
    }
    else if (incomingSerial[1] == 't') 
    {
      if (incomingSerial[2] == 'p') 
      {
        char ppp[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
        heaterValues->P = 0.0;
        heaterValues->P = atol(ppp);
        msgBuffer += " Setting P = ";
        msgBuffer += String(heaterValues->P, 1);
      }
      else if (incomingSerial[2] == 'i') 
      {
        char iii[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
        heaterValues->I = 0.0;
        heaterValues->I = atol(iii);
        msgBuffer += " Setting I = ";
        msgBuffer += String(heaterValues->I);
      }
      else if (incomingSerial[2] == 'd') 
      {
        char ddd[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
        heaterValues->D = 0.0;
        heaterValues->D = atol(ddd);
        msgBuffer += " Setting D = ";
        msgBuffer += String(heaterValues->D);
      }
      
      heaterPID.SetTunings(heaterValues->P,heaterValues->I,heaterValues->D);
    }
  } 
  else if(incomingSerial[0] == 'S' && incomingSerial[1] == 't' )
  {
    for(int i = 0; i < lidTemperature.size; i++) 
    {
      Serial.print(lidTemperature.history[i]);
      Serial.print(", "); 
    }
    Serial.print(", index = ");
    Serial.println(lidTemperature.index);
  } 

  //Do we really need MSE? It now feels like a vestige of testing. 

  else if(incomingSerial[0] == 'S' && incomingSerial[1] == 'e' )
  {
    for(int i = 0; i < historyArraysSize; i++) 
    {
      Serial.print(mseGlassTemperatureHistory[i]);
      Serial.print(", "); 
    }
    Serial.print(", index = ");
    Serial.println(historyArraysIndex);
  } 
  else if(incomingSerial[0] == 'S' && incomingSerial[1] == 'a' )
  {
    for(int i = 0; i < enclosureTemperature.size; i++) 
    {
      Serial.print(enclosureTemperature.history[i]);
      Serial.print(", "); 
    }
    Serial.print(", index = ");
    Serial.println(enclosureTemperature.index);
  } 
  else if(incomingSerial[0] == 'E' && incomingSerial[1] == 'a' )
  {
       int thisError = incomingSerial[3];
       
       timesErrorsAcknowledged[thisError] = millis(); 
       areErrorsAcknowledged[thisError] = true;

       msgBuffer += " Error ";
       msgBuffer += thisError;	  
       msgBuffer += " acknolwedged & silenced for ";
       msgBuffer += String(errorGraceTime/6000,1);
       msgBuffer += " min.";
  } 
  else if(incomingSerial[0] == 'E' && incomingSerial[1] == 'g' )
  {
        int minGraceTime = incomingSerial[3];
	errorGraceTime = minGraceTime * 60000;
        msgBuffer += " errorGraceTime now ";
        msgBuffer += String(minGraceTime, 0);
	msgBuffer += " min";	  
  } 
  else if(incomingSerial[0] == 'L')
  {
    Serial.println("\tT(enclosure):\tT(lid):\tT(encl.slope):\tT(lidThermistor.slope):\tT(mse):\tsetPt(lid):\tsetPt(enclosure):\tPWMOut:\tV(in):\tPID:\tError:\tMsg:";); 
  } 
  else 
  {
    Serial.println(" Command not recognized. Press 'h' for index of commands");
  }
}
*/
// Get the average voltage from the tunable buck converter, return tunable buck converter voltage
void GetBuckConverterVoltage(double &newVoltage)
{
  // Read voltage from voltmeter 
  double thisVoltage = 0.0;
	
  // Average numberOfSampleVoltageReadings from heater voltage source with a slight delay
  for (int i = 0; i < numberOfSampleVoltageReadings; i++) 
  {
    thisVoltage += analogRead(VOLTMETERPIN)/ numberOfSampleVoltageReadings;
    delay(1);
  }
 
  // Convert the 0-1024 analog input to a voltage
  thisVoltage = (thisVoltage * boardVoltageOut) / 1023.0;   
  //Correct for the voltage-divide upstream of the measured voltage
  thisVoltage = thisVoltage / (resistor2Coefficient / (resistor1Coefficient + resistor2Coefficient));

  // If voltage is very small, consider it negligible and set it to 0        
  if (thisVoltage < 0.1)  
  {  
    thisVoltage = 0.0;   
  }
  newVoltage = thisVoltage; 
}

//%%% to ErrorCheck library 

//void ErrorCheck(int &thisError)
//{
//  double lidTemperature.deviation = lidTemperature.value - lidTemperature.setpoint; // This was initially deltaFromSetPt and was never declared or set so I did that here
//
//  // Assume all is well
//  int newError = 0;
//  
//  // Check if thermistor is not connected. Temp will read as -273C
//  if (lidTemperature.value < 0) 
//  {
//    newError = 1;
//    // Assume PID will be full throttle. Override to 0.
//    PWMOutput = 0;
//    if (!isErrorBuffered)
//    {
//      isErrorBuffered = true;
//      errorBuffer += newError;
//      errorBuffer += " Glass thermistor disconected. Output shut off.";
//    }
//  } 
//
//  
//  else if (lidTemperature.value >= lidTemperature.upperLimit) 
//  {
//    newError = 2;
//    // Gently throttle back the power output to reduce temperature
//    // While too high output will halve recursively - eventually to 0
//    PWMOutput = heaterValues->errorOutput;
//    if (!isErrorBuffered)
//    {
//      isErrorBuffered = true;
//      errorBuffer += newError;
//      errorBuffer += " Glass >= max temp. Throttling output to ";
//      errorBuffer += String(heaterValues->errorOutput, 0);
//    }
//  }
//
//  // Thermal run away. Getting hotter than setpoint and beyond minor overshoot      
//  // Do not trigger error if the program is just starting back up after a reset, as the glass will have cooled a bit.
//  else if (lidTemperature.deviation > 4.0 && (millis() - startUpTime) > errorGraceTime) 
//  {
//    if (!isErrorBuffered)
//    {
//      newError = 3;
//      errorCodePrevious = newError;
//      errorBuffer += newError;
//      errorBuffer += " Thermal runaway detected. Throttling output to ";			
//      errorBuffer += String(heaterValues->errorOutput, 0);
//    }
//  }
//  
//  else if (lidTemperature.historyFilled) 
//  {
//
//    // Slope (over 50 samples) when ramping up can be >11 C/min 
//    // When stable, always within +/-0.7C/min
//    // When off, falling from 35C in ambient temp, goes to ~ -1C/min
////%%% Issue to address. How to avoid this error being thrown if system shuts off, slope<-1, then power comes on and slope transits through 0. 
//    // If temp not reaching desired setpoint and slope is relatively flat, assume power is insufficient to reach setpoint
//    if (lidTemperature.deviation < -1.0 && lidTemperature.slope[lidTemperature.index-1] < 0.5 && lidTemperature.slope[lidTemperature.index-1] > -1) 
//    {
//      if (!isErrorBuffered)
//      {
//        newError = 4;
//        errorBuffer += newError;
//  	    errorBuffer += " Under-powered. Increase max output.";
//      }
//    }
//
//    // Heater failure: temperature falling unexpectedly. Assume power not getting to the heating elements
//    else if (lidTemperature.deviation < -0.5 && lidTemperature.slope[lidTemperature.index-1] <= -1 && (millis() - lidTemperature.setpointLastUpdate) >= lidTemperature.setpointInterval/2) 
//  //%%%need some additional metric to indicate that glass temp is not simply descreasing to a new set point, then some delay to account for stabilization
//    {
//      if (!isErrorBuffered)
//      {
//        newError = 5;
//        PWMOutput = 0;
//  	    errorBuffer += newError;
//  	    errorBuffer += " Heater failure. Output - off.";
//      }
//    
//    else 
//    {
//      errorBuffer += newError;
//    }
//  }
//	else 
//  {
//      errorBuffer += newError;
//  }
//
//  thisError = newError;
//}


//deprecated and replaced by updateSensor
/*
void AppendToHistory(double gValue, double aValue)
{
  lidTemperature.history[historyArraysIndex] = gValue;
  enclosureTemperature.history[historyArraysIndex] = aValue;
  mseGlassTemperatureHistory[historyArraysIndex] = sqrt(sq(gValue-lidThermistor.setpoint));
  historyArraysIndex++;
  
  if (historyArraysIndex >= historyArraysSize) 
  {
    historyArraysIndex = 0;
    lidTemperature.historyArraysFilled = true;
    enclosureTemperature.historyArraysFilled = true;
  }
  endHistoryArraysIndex = historyArraysIndex + 1;

  if (historyArraysIndex == historyArraysSize - 1) 
  {
    endHistoryArraysIndex = 0;
  }
}

void GetArrayAverage(double thisArray[], double &average)
{
  double arrAvg = 0.0;
  for (int i = 0; i < historyArraysSize; i++) 
  {
    arrAvg += thisArray[i];
  }
  if (isHistoryArraysFilled) 
  {
    arrAvg /= historyArraysSize;   
  } 
  else 
  {
    arrAvg /= long(historyArraysIndex + 1);
  }

  average = arrAvg;
}
*/

// Print all relevant values to serial
// Eventually this will need to merge with logToSD.cpp in IncuStage
void PrintParametersToSerial()
{

  if (nLogged == 0) {
    Serial.println(" ");
    logBuffer += "\tT(enclosure):\tT(lid):\tT(encl.slope):\tT(lidTemperature.slope):\tT(mse):\tsetPt(lid):\tsetPt(enclosure):\tPWMOut:\tV(in):\tPID:\tError:\tMsg:";
    Serial.println(logBuffer);
    logBuffer = "";
  }

  logBuffer = "";
  logBuffer += "\t";
  logBuffer += String(enclosureTemperature.value, 2);
  logBuffer += "\t";
  logBuffer += String(lidTemperature.value, 2);
  logBuffer += "\t";
  logBuffer += String(enclosureTemperature.slope[enclosureTemperature.index-1], 3);
  logBuffer += "\t";
  logBuffer += String(lidTemperature.slope[lidTemperature.index-1], 3);
  logBuffer += "\t";
  logBuffer += String(lidTemperature.meanSquareError, 3);
  logBuffer += "\t";
  logBuffer += String(lidTemperature.setpoint, 2);
  logBuffer += "\t";
  logBuffer += String(enclosureTemperature.setpoint, 1);
  logBuffer += "\t";
  logBuffer += String(heaterValues.outputToDevice);
  logBuffer += "\t";
  logBuffer += String(buckConverterVoltage, 2);
  logBuffer += "\t";
  logBuffer += String(heaterValues.P);
  logBuffer += "/";
  logBuffer += String(heaterValues.I);
  logBuffer += "/";
  logBuffer += String(heaterValues.D);
  logBuffer += "\t";
  logBuffer += errorBuffer;
  logBuffer += "\t";
  logBuffer += msgBuffer;
  logBuffer += "\n";
  Serial.print(logBuffer);
  logBuffer = "";
  msgBuffer = " ";
  errorBuffer = " ";
  //isErrorBuffered = false;
  // for (int i = 0; i < numberOfErrorCodes; i++) {
  //  errorCodes.buffered[i] = false;
  //}

  nLogged++;
}

void CheckGlassSetpoint()
{
  // Current time in milliseconds since the program has been running
  int currentTimeMilliseconds = millis();

  enclosureTemperature.deviation = enclosureTemperature.value - enclosureTemperature.setpoint;
  //enclosureTemperature.deviation = enclosureTemperature.value - enclosureTemperature.valueSetpoint;
  lidThermistor.autoSetpointChange = 0;

  // If it has been long enough since the last glass setpoint update and the history array has enough data, check if the glass setpoint needs to be updated
  if ((currentTimeMilliseconds - lidTemperature.setpointLastUpdate) >= lidTemperature.setpointInterval && lidTemperature.historyFilled == true)
  {
    // Calculate gap from air temperature setpoint

    // If the glass temperature has been stable and the air temperature is significantly higher than the air temperature setpoint, update the glass setpoint
    {

      //If air temp is too high, reduce glass temp, if too low, increase glass temp. This toggles direction of change.
      float deviationDirection = 0.0;
      if (enclosureTemperature.deviation>=0) deviationDirection = -1.0;
      if (enclosureTemperature.deviation<0) deviationDirection = 1.0;

      // Very basic proportional control of glass temperature relative to air temp
      
      if( abs(enclosureTemperature.deviation) > 0.4){
        // Reduce the glass setpoint
        lidThermistor.autoSetpointChange = deviationDirection * 2.0;
      }
      else if( abs(enclosureTemperature.deviation) > 0.2){
        // Reduce the glass setpoint
        lidThermistor.autoSetpointChange = deviationDirection * 1.5;
      }
      else if( abs(enclosureTemperature.deviation) > 0.15){
        // Reduce the glass setpoint
        lidThermistor.autoSetpointChange = deviationDirection * 0.75;
      }
      else if( abs(enclosureTemperature.deviation) > 0.10){
        // Reduce the glass setpoint
        lidThermistor.autoSetpointChange = deviationDirection * 0.25;
      }  
      else {
        // Reduce the glass setpoint
        lidThermistor.autoSetpointChange = 0.0;
      }  

      //increment the deviation of glassSetpoint from the most recent user-defined glassSetpoint
      lidThermistor.cummSetpointChange += lidThermistor.autoSetpointChange;
      
      // if the delta is positive, limit the total possible increase to 5C. The user can reset this by manually increasing the set point.
      //The should limit overshoot in the automatic increases in glass set point
      if (lidThermistor.autoSetpointChange>0 && lidThermistor.cummSetpointChange >= lidThermistor.maxAutoIncrease) {
        lidThermistor.cummSetpointChange = lidThermistor.maxAutoIncrease;
        lidThermistor.autoSetpointChange = 0;
      }

      lidTemperature.setpoint += lidThermistor.autoSetpointChange;

      // Update the last glass setpoint update time to the current time
      lidTemperature.setpointLastUpdate = currentTimeMilliseconds;

      // Send a message to the serial
      msgBuffer += "Glass Setpoint updated to ";
      msgBuffer += String(lidTemperature.setpoint);

    }
  }
}


//Note, with change to generalSensor for temperatures and use of sensorUpdate():
// 1. the erroneous check needs to come before sensorUpdate and the values are logged - Done
// 2. the erroneous check should work on a single general sensor, i.e. lidTemperature or enclosureTemperature
// 3. it should include a double Tolerance to identify erroneous readings

void RemoveErroneousSensorReadings(generalSensor &sensor, double tolerance)
{
  // Calculate difference between current glass temperature and previous glass temperature reading
  //double glassTemperatureDataDifference = abs(lidTemperature.value - lidThermistor.prevTtemperature);

  //Since glassTemperatureHistory is a circular buffer, we access the values 1 and 2 indices before the current historyArraysIndex
  if ( (sensor.historyFilled == true) && (sensor.history[(sensor.index-1)%sensor.historySize] != sensor.history[(sensor.index-1)%sensor.historySize]) )
  {
    if ( (abs(sensor.value - sensor.history[(sensor.index-1)%sensor.historySize])>tolerance) && (abs(sensor.value - sensor.history[(sensor.index-2)%sensor.historySize])>tolerance) ) 
    {
     
      // Send a message to the serial
      msgBuffer += "Erroneous ";
      msgBuffer += sensor.name;
      msgBuffer += " reading of ";
      msgBuffer += sensor.value;
      msgBuffer += " data point reassigned to ";
      sensor.value = sensor.history[(sensor.index-2)%sensor.historySize];
      msgBuffer += sensor.value;
      
    }
  }

 /*
  
  // Calculate difference between current air temperature and previous air temperature reading
  //double previousAirTemperatureDataDifference = abs(enclosureTemperature.value - enclosureTemperature.prevValue);

  // Calculate difference between current air temperature and oldest air temperature reading
  //double oldestAirTemperatureDataDifference = abs(enclosureTemperature.value - enclosureTemperature.prev2Temperature);

  //!!!!enclosureTemperature.prev2Temperature deprecated. Replace with reference to enclosureTemperature.history[(index-2)%historyArraysSize]

  // If the differences are significant and a previousAirTemperature and oldestAirTemperature value exists, reassign the reading a new value
  if ( (abs(enclosureTemperature.value - enclosureTemperature.history[(enclosureTemperature.index-1)%enclosureTemperature.size])) > 0.25 && (abs(enclosureTemperature.value - enclosureTemperature.history[(enclosureTemperature.index-2)%enclosureTemperature.size])) > 0.25 && enclosureTemperature.prevValue != 0 && enclosureTemperature.history[(enclosureTemperature.index-2)%enclosureTemperature.size] != 0)
  {
    // Reassign this inaccurate data point to the value of the previous reading
    // Send a message to the serial
    msgBuffer += "Erroneous air temperature (";
    msgBuffer += enclosureTemperature.value; 
    msgBuffer += ") reassigned, ";
    msgBuffer += " 1 & 2 previous readings were ";
    msgBuffer += enclosureTemperature.history[(historyArraysIndex-1)%historyArraysSize];
    msgBuffer += " and ";
    msgBuffer += enclosureTemperature.history[(historyArraysIndex-2)%historyArraysSize];
    enclosureTemperature.value = enclosureTemperature.prev2Temperature;
    
  }
  */
  
}

void sensorUpdate(generalSensor& sensor) {

  //add the value to the array and note the time that value is added to the history in millis
  sensor.history[sensor.index] = sensor.value;
  sensor.deviation = sensor.value - sensor.setpoint;
  sensor.time[sensor.index] = millis();
  if (sensor.value < sensor.minimum) sensor.minimum = sensor.value;
  if (sensor.value > sensor.maximum) sensor.maximum = sensor.value;

  //determine slope and append to slope history
  if (sensor.historyFilled) {
    //determine slope
    sensor.slope[sensor.index] = (sensor.value - sensor.history[(sensor.index - sensor.slopeInterval) % sensor.historySize]) / 
                                 ( (sensor.time[sensor.index] - sensor.time[(sensor.index - sensor.slopeInterval) % sensor.historySize]) / sensor.slopeUnits);
    if (abs(sensor.slope[sensor.index])>10000) {
      sensor.slope[sensor.index] = 0;
    }

    //calculate deviation squared for mse
    sensor.errorHistory[sensor.index] = (sensor.deviation)*(sensor.deviation);

    //calculate average and mean squared error over the history
    sensor.average = 0;
    sensor.meanSquareError = 0;
    
    for (int i = 0; i < sensor.historySize; i++) {
      
      sensor.average += sensor.history[i];
      sensor.meanSquareError += sensor.errorHistory[i];
      
    }
    
    sensor.average /= sensor.historySize;
    sensor.meanSquareError /= sensor.historySize;
    sensor.meanSquareError = sqrt(sensor.meanSquareError);
    
  } else {
    //if history is not yet filled ....

      sensor.slope[sensor.index] = 0;
      sensor.meanSquareError = 0;
      sensor.average = 0;
      
      if (sensor.index==0 ) {
        sensor.errorHistory[sensor.index] = 0;
      } else {
        sensor.errorHistory[sensor.index] = (sensor.value-sensor.setpoint)*(sensor.value-sensor.setpoint);
      }
      
      for (int i = 0; i < sensor.index; i++) {
        sensor.average += sensor.history[i];
        sensor.meanSquareError += sensor.errorHistory[i];
      }
      sensor.average /= (sensor.index+1);
      sensor.meanSquareError /= (sensor.index+1);
      sensor.meanSquareError = sqrt(sensor.meanSquareError);
  }

  sensor.index++;

  //this is essentially used like a circular buffer
  if (sensor.index == (sensor.historySize)) {
    sensor.historyFilled = true;
    sensor.index = 0;  // reset
  }
}
