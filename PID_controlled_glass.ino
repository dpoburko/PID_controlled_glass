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

// Initialize errorCheck library
errorCheck ErrorCheck; 


// *************************************************************************************************************************************
// PID CONTROLLER VARIABLES
// *************************************************************************************************************************************

// Set glass temperature goal (in degrees Celsius)
double glassSetpoint = 55.0; //need to figure out how to reference this to the thermistor setpoint in the new structure

// PID proportional, integral, and derivative constants (these values (2, 96, 21) are working well for the 5 mm thick glass)
//double PIDKp = 2, PIDKi = 96, PIDKd = 21;

// Define setpoint, input, and output variables (from PID library)
double PIDSetpoint, PIDInput, PIDOutput;

  PIDvalues lidPID{
    P = 2;
    I = 96;
    D = 21;
    setpoint  = glassSetpoint; // will be set to lid.setPoint
    output = 0;
    prevOutput= 0;
    maxNormalOutout = 40;
    maxHighOutput = 45;
    currMaxOutput = maxNormalOutout;
    deltaForMax = 3.0;
  }

// Enumerate PID modes
enum enumPIDMode {
  manualPIDMode,
  automaticPIDMode
};

enumPIDMode PIDMode;

// Check if output has changed since last loop
bool isNewPID = false;

//%%% Could this be replaced with a simple timer? e.g. timer glassSetpointUpdate using a structure with .start and .duration elements 
// Interval for updating glass setpoint (10 minutes or 600000 milliseconds)
int glassSetpointInterval = 150000;
// Variable to store when the last glass setpoint update happened
int lastGlassSetpointUpdate = 0;

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

//Initiate structure holding error codes
const itn nErrorCodes = 6;
double errorGraceTime = 180000; //in ms
String codeNames = {"OK","no thermistor","over temperature","Thermal runaway","Under powered","Heater failure"};

errorCode errorCodes[nErrorCodes]; //error codes now held in a structure that can be passed to errorCheck

for (i=0 ; i < nErrorCodes; i++) {
  errorCodes[i].ID = i;
  name = codeNames[i];
  active = false;
  startTime = 0;
  graceTime = errorGraceTime;
  silenced = false;
  sleepTime = 180000;
  buffered = false;
  userOverride = false;
}
String errorBuffer,  msgBuffer, logBuffer;
long startUpTime;


// *************************************************************************************************************************************
// STEINHART EQUATION VARIABLES - now all held in a structure for each thermistor. This allows for simple expansion to have additional thermistors
// *************************************************************************************************************************************

  thermistor lid
  {
    pin = THERMISTOR1PIN; 
    temperature = 22.0; 
    maxTemperature = 65.0;;
    prevTemperature = 22.0;
    prevTemperature2 = 22.0;
    setPoint = glassSetpoint;  //evenutally set to be determined by PID setpoint or vice versa
    slope = 0;
    slopeInterval = 10;
    maxAutoIncrease = 5;
    cummSetpointChange = 0;
    name = "glass lid";
    rNominal = 10000;
    rSeries = 9985;
    tNomimal = 25.0;
    bCoefficient = 3435;
  }

  thermistor enclosure
  {
    pin = THERMISTOR2PIN; 
    temperature = 22.0; 
    maxTemperature = 40.0;;
    prevTemperature = 22.0;
    prevTemperature2 = 22.0;
    setPoint = 32.0;  //evenutally set to be determined by PID setpoint or vice versa
    slope = 0;
    slopeInterval = 20;
    name = "enclosure temperature";
    rNominal = 10000;
    rSeries = 9985;
    tNomimal = 25.0;
    bCoefficient = 3435;
  }


// Thermistor resistance at nominal temperature (25 degrees Celsius)
const long nominalResistance = 10000; 
// Temperature for nominal resistance (almost always 25 degrees Celsius)
const long nominalTemperature = 25;
// B coefficient
const long bCoefficient = 3435;
// Measured resistance for whatever series resistor (~10 kOhms) is used
const long seriesResistance = 9985;

// *************************************************************************************************************************************
// VOLTAGE DIVIDER VARIABLES
// *************************************************************************************************************************************

// Coefficient for resistors 1 and 2 used for 1/10 voltage divider to detect input voltage
const double resistor1Coefficient = 9810;
const double resistor2Coefficient = 983;

// *************************************************************************************************************************************
// PULSE-WIDTH MODULATION VARIABLES
// *************************************************************************************************************************************

heater lidHeater{
  output = 0;
  prevOut = 0;
  errorOutput = 10;
}

// Pulse-width modulator output value needed to change glass temperature
//double PWMOutput = 0.0; // now in lidPID strucutre


// Last pulse-width modulator output value
//double PWMOutputLast = 0.0; // now in lidPID strucutre

//%%% to ErrorCheck library?
// Value used to hold glass temperature warm but safe
//double PWMOutputIfError = 10.0; // now in pidPID structure

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
//double airTemperatureSlope = 0.0;


// Variable to store air temperature
//double airTemperature = 20.0; //enclosure

// Variable to store air temperature difference from setpoint
double airTemperatureDeviation = 0.0;

// Variable to store previous air temperature
//double previousAirTemperature = 0.0;

// Variable to store oldest air temperature
//double oldestAirTemperature = 0.0;

// Air temperature goal
//double airTemperatureSetpoint = 37.0;

// Allowable difference betwee Air temperature and airTemperatureSetpoint
double airTemperatureTolerance = 0.1;

// Boolean to check if the air temperature setpoint has been reached
bool isAirTemperatureSetpointReached = false;

// Boolean to check if the air temperature setpoint has been noted
bool isAirTemperatureSetpointNoted = false;

// yes if Air temperature < airTemperatureSetpoint and reassessed at start-up and changes of airTemperatureSetpoint
bool isAirTemperatureClimbing = true;

// Size of arrays used for logging temperature and temperature error
//const int historyArraysSize = 60; // now in structure library

// Variable to store a counter for the index of the temperature and temperature error arrays
int historyArraysIndex = 0;

// Variable to store when the index of the temperature and temperature error arrays ends
int endHistoryArraysIndex = 0;

// Variable to store whether or not the temperature and temperature error arrays have been filled
//bool isHistoryArraysFilled = false;

// Array to store glass temperature
//double glassTemperatureHistory[historyArraysSize];

// Array to store glass temperature
//double airTemperatureHistory[historyArraysSize];

// Array to store mean square error of glass temperature
double mseGlassTemperatureHistory[historyArraysSize];

// Variable to store the mean square error of the glass temperature
double mseGlassTemperature = 0.0;

// Boolean to check whether or not there is a new glass temperature
bool isNewGlassTemperature = false;

// Boolean to check whether or not there is a new air temperature
bool isNewsAirTemperature = false;


// *************************************************************************************************************************************
// SERIAL VARIABLES
// *************************************************************************************************************************************

// Maximum message length for serial communication (in characters)
const int serialMaxMessageLength = 12;

// Character that will signal end of message from the PC
char endSerialMessageCharacter = '.';

// Array for serial communication, of length determined by max message length variable
char incomingSerial[serialMaxMessageLength];

// Variable to store message length
int lengthOfSerialMessage = 0;

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
//STEINHART thermistor1(THERMISTOR1PIN, &glassTemperature, nominalResistance, nominalTemperature, bCoefficient, seriesResistance);
//STEINHART thermistor2(THERMISTOR2PIN, &airTemperature, nominalResistance, nominalTemperature, bCoefficient, seriesResistance);
//%%%% In theory, now we should simple be able to pass each thermistor structure to the steinhard function, though this is less simple for others to reproduce
STEINHART thermistor1(lid.pin, &lid.temperature, lid.rNominal, ld.tNominal, lid.bCoefficient, lid.rSeries);
STEINHART thermistor2(enclosure.pin, &enclosure.temperature, enclosure.rNominal, enclosure.tNominal, enclosure.bCoefficient, enclosure.rSeries);

// Create PID controller for the glass, including relevant information for the PID such as input, output, setpoint, and constants
//PID heaterPID(&lid.temperature, &PIDOutput, &glassSetpoint, PIDKp, PIDKi, PIDKd, DIRECT);
PID heaterPID(&lid.temperature, &lidPID.output, &lid.setpoint, lidPID.P, lidPID.I, lidPID.D, DIRECT);

errorCheck errorCheck(&errorCodes, &lid, &errorBuffer, &msgBuffer);

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

  // Initialize buffers
  errorBuffer.reserve(256);
  msgBuffer.reserve(128);
  logBuffer.reserve(512);
 
  // Initialize PID (automaticPIDMode mode)
  PIDMode = automaticPIDMode;
  heaterPID.SetMode(automaticPIDMode);

  // Link PID setpoint with glass setpoint
  //PIDSetpoint = glassSetpoint;   //vestigial, not longer used

  // Set up PID output limits  
  heaterPID.SetOutputLimits(0, lidPID.maxHighOutput); 
  heaterPID.SetSampleTime(glassVoltageReadingInterval); 

  // Set frequency of voltage readings for thermistor 1
  thermistor1.setSampleTime(glassVoltageReadingInterval);

  // Read the temperature from thermistor 1
  thermistor1.read();

  // Repeat for thermistor 2
  thermistor2.setSampleTime(glassVoltageReadingInterval);
  isNewsAirTemperature = thermistor2.read();
  enclosure.prevTemperature = enclosure.temperature;
  enclosure.prev2Temperature = enclosure.temperature;
  //oldestAirTemperature = airTemperature;
  // = airTemperature;

  // Print initialization message and temperature to serial
  Serial.print("   ===== PID controlled glass heater =====================");
  Serial.println("   Stage-Top Incubator component"); 
  Serial.print("   Initial Temperature:");
  Serial.println(glassTemperature, 2);

  startUpTime = millis();
}

// *************************************************************************************************************************************
// MAIN LOOP
// *************************************************************************************************************************************

void loop() 
{

  // Call STEINHART library, check if value is updated  
  isNewGlassTemperature = thermistor1.read();
  
  // Add the new temp reading to the history array if a new value is available
  if (isNewGlassTemperature) 
  {
    // Read temperature from thermistor 2 - air temperature
    thermistor2.read();

    // Get the voltage from the tunable buck converter (what do we do with this?)
    GetBuckConverterVoltage(buckConverterVoltage);

    // Check to make sure the glass temperature data point makes sense, and reassign the value if necessary
    RemoveErroneousTemperatureReadings();

    // Log the glass temperature in an array
    AppendToHistory(lid.Temperature,airTemperature );
    
    // If the arrays are not yet full, calculate the slope of the glass temperature change
    if (lid.historyFilled == false) 
    {
      // If this is the first value in the array, the slope cannot yet be calculated, so set it to zero
      if (historyArraysIndex == 0) 
      {
        //glassTemperatureSlope = 0.0;
        //airTemperatureSlope = 0.0;
        lid.slope = 0.0;
        enclosure.slope = 0.0;      
        
      } 
      else 
      {
        // In degrees/minute
        //glassTemperatureSlope = (glassTemperature - glassTemperatureHistory[0]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
          //glassTemperatureSlope = (glassTemperature - glassTemperatureHistory[(historyArraysIndex-slopeInterval)%historyArraysSize]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
          //airTemperatureSlope = (airTemperature - airTemperatureHistory[(historyArraysIndex-slopeInterval)%historyArraysSize]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
          lid.slope = (lid.temperature - lid.history[(historyArraysIndex-lid.slopeInterval)%historyArraysSize]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
          enclosure.slope = (enclosure.temperature - enclosure.history[(historyArraysIndex-enclosure.slopeInterval)%historyArraysSize]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
      }
    }
    else 
      {
        // In degrees/minute
        lid.slope = (lid.temperature - lid.history[endHistoryArraysIndex]) / (historyArraysSize * glassVoltageReadingInterval/60000);
        enclosure.slope = (enclsoure.temperature - enclosure.history[endHistoryArraysIndex]) / (historyArraysSize * glassVoltageReadingInterval/60000);
      }
    
    // Compute the average of the arrays
    GetArrayAverage(mseGlassTemperatureHistory, mseGlassTemperature);

    // If glass temperature is far from the setpoint, set the maximum PID output to a pre-determined aggressive value
    if ( (lid.setpoint - lid.temperature) >  lidPID.deltaForMax )
    {
      // If the current maximum PID output is not already set to aggressive, set it to aggressive and print a message to the serial
      if (lidPID.currMaxOutput != lidPID.maxHighOutput) 
      {
        // Set the current PID output to aggressive
        lidPID.currMaxOutput = lidPID.maxHighOutput;
        heaterPID.SetOutputLimits(0, lidPID.currMaxOutput);

        // Print a message to the serial
        msgBuffer += " Switching to lidPID.currMaxOutput_";
	      msgBuffer += String(lidPID.currMaxOutput);
      } 
    }
    // If the glass temperature is close to the setpoint, set the maximum PID output to a pre-determined conservative value 
    else 
    {
      // If the current maximum PID output is not already set to conservative, set it to conservative and print a message to the serial
      if (lidPID.currMaxOutput != lidPID.maxNormalOutput) 
      {
        // Set the current PID output to conservative
        lidPID.currMaxOutput = maxNormalOutput;
        heaterPID.SetOutputLimits(0, lidPID.currMaxOutput);

        // Print a message to the serial
        msgBuffer += " Switch to lidPID.maxNormalOutput_";
	      msgBuffer += String(lidPID.currMaxOutput);
      }
    }
  }
  
  // If the PID is in automaticPIDMode, compute output
  if (heaterPID.GetMode() == 1) 
  {
    // Make sure it is time to recalculate
    isNewPID = heaterPID.Compute();

    // Define output of PID as the pulse-width modulator output (will be at 0 the first time around as the PID needs two data points for computation)
    if (isNewPID == true) 
    {
    	lidHeater.output = PIDOutput; // handoff the PID's intended output to the heater structure
    	// Perform a series of tests on thermistor reading and temperature vs. setpoint and over time
      ErrorCheck(errorCode);
    } 
  }

   //%%% @Izzy, I have just noticed that we have no errorChecking when in manual mode. This seems prone to troubles. Let's rethink this.
   // DP - isAirTempuratureCliminb might more easily be determined as a slope from the airTemperatureHistory
	
  // Check if the air temperature is sufficiently heated
  if (isAirTemperatureClimbing == true)
  {
    //if air temp needs to increase to reach set point
    if (enclosure.temperature >= enclosure.setpoint)
    {
      enclsoure.setpointReached = true;
      if (enclosure.setpointNoted == false) {
        msgBuffer += " air temperature set point reached"; 
        enclosure.setpointNoted = true;
      }
    }
  } else {
    //assume air temp needs to fall to reach set point
    if (enclosure.temperature <= enclosure.setpoint)
    {
      enclsoure.setpointReached = true;
      if (enclosure.setpointNoted == false) {
        msgBuffer += " air temperature set point reached"; 
        enclosure.setpointNoted = true;
      }
    }
  }
  
  // Check to see if the glass setpoint needs to be updated, only if airTemperatureSetpointReached is true
  if (enclsoure.setpointReached)
  {
    CheckGlassSetpoint();
  }
	
  // If the output from the PID is different from the previous output, adjust the pulse-width modulator duty cycle
  if (lidHeater.output!= lidHeater.prevOutput) 
  {
    // Adjust the duty cycle of the PWM pin connected to the MOSFET fron the PID output if the value has changed
    analogWrite(PWMPINOUT, lidHeater.output);

    // Log the new output value as the last output value received
    lidHeater.prevOutput = lidHeater.output;
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
      /*
      // If user inputted OK, indicate that the error acknowledgement has been received
      else if ((incomingSerial[0] == 'O') & (incomingSerial[1] == 'K')) 
      {
        // Print acknowledgement to serial
        Serial.println("OK. Error acknowledgement received");

        // Set error acknowledged boolean to true
        isErrorAcknowledged = true; 
        
      }
      */
      // If user inputted h, H, or ?, print out available serial commands
      else if (incomingSerial[0] == 'h' || incomingSerial[0] == 'H' || incomingSerial[0] == '?') 
      {
        Serial.println("\n >>>> PID Serial Commands <<<<\n");
        Serial.println(" End commands with '.' for faster responses");
        Serial.println(" Interacting with the PID controller");
        Serial.println("   Pma - call  heaterPID.SetMode(automaticPIDMode)");
        Serial.println("   Pmm - call  heaterPID.SetMode(manualPIDMode)");
        Serial.println("   Pmg - call  heaterPID.GetMode()");
        Serial.println("   Psnnn - update glassSetpoint as nn.n degrees");
        Serial.println("   Pannn - update airTemperatureSetpoint as nn.n degrees");
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
  }

//since glass and air temp are logged in history indices, we could just skip this and pull the values from arrays when needed
  // Set the previousGlassTemperature value to the current glass temperature
  //  previousGlassTemperature = glassTemperature;
  lid.prevTemperature = lid.temperature;
  // Set the previousAirTemperature value to the current air temperature
  //previousAirTemperature = airTemperature;
  enclosure.prev2Temperature = enclosure.prevTemperature;
  // Set the oldestAirTemperature value to the previousAirTemperature value
  //oldestAirTemperature = previousAirTemperature;
  enclosure.prevTemperature = enclosure.temperature;

  // Delay determines how often loop repeats
  delay(20); 
}

// *************************************************************************************************************************************
// FUNCTIONS
// *************************************************************************************************************************************

// Read serial message, consider moving this to a library
void ParsePIDCmd()
{
  if (incomingSerial[0] == 'P') 
  {
    if (incomingSerial[1] == 'm') 
    {
      if (incomingSerial[2] == 'a') 
      {
        PIDMode = automaticPIDMode;
        heaterPID.SetMode(PIDMode);
        msgBuffer += "PID set to Automatic"; 
      }

      if (incomingSerial[2] == 'm') 
      {
        PIDMode = manualPIDMode;
        heaterPID.SetMode(PIDMode); 
        msgBuffer += " PID set to MANUAL"; 
      } 
      
      if (incomingSerial[2] == 'g') 
      {
        PIDMode = manualPIDMode;
        heaterPID.SetMode(PIDMode); 
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
      PWMOutputIfError = atof(newOutput); 
			msgBuffer += " PWMoutputIfError now "; 
			msgBuffer += String(PWMOutputIfError, 1);
           
    }
    else if (incomingSerial[1] == 's') 
    {
      char newSetpoint[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      glassSetpoint = atof(newSetpoint)/10.0;
      msgBuffer += " glassSetpoint now ";
			msgBuffer += String(glassSetpoint,1); 
    }
    else if (incomingSerial[1] == 'a') 
    {
      char newSetpoint[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      airTemperatureSetpoint = atof(newSetpoint)/10.0;
      msgBuffer += " airTemperatureSetpoint now ";
      msgBuffer += String(airTemperatureSetpoint,1); 

      if (airTemperature < airTemperatureSetpoint) {
        isAirTemperatureClimbing  = true;
        msgBuffer += " isAirTemperatureClimbing  = true";
      } else {
        isAirTemperatureClimbing  = false;
        msgBuffer += " isAirTemperatureClimbing  = false";
      }
    
      isAirTemperatureSetpointNoted = false;

    }
    /*
    else if (incomingSerial[1] == 'a') 
    {
      char newSetpoint[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      
      double newAirTemperatureSetpoint = atof(newSetpoint)/10.0;

      
    }
    */
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
        PIDKp = 0.0;
        PIDKp = atol(ppp);
        msgBuffer += " Setting P = ";
        msgBuffer += String(PIDKp, 1);
      }
      else if (incomingSerial[2] == 'i') 
      {
        char iii[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
        PIDKi = 0.0;
        PIDKi = atol(iii);
        msgBuffer += " Setting I = ";
        msgBuffer += String(PIDKi);
      }
      else if (incomingSerial[2] == 'd') 
      {
        char ddd[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
        PIDKd = 0.0;
        PIDKd = atol(ddd);
        msgBuffer += " Setting D = ";
        msgBuffer += String(PIDKd);
      }
      
      heaterPID.SetTunings(PIDKp,PIDKi,PIDKd);
    }
  } 
  else if(incomingSerial[0] == 'S' && incomingSerial[1] == 't' )
  {
    for(int i = 0; i < historyArraysSize; i++) 
    {
      Serial.print(glassTemperatureHistory[i]);
      Serial.print(", "); 
    }
    Serial.print(", index = ");
    Serial.println(historyArraysIndex);
  } 

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
    for(int i = 0; i < historyArraysSize; i++) 
    {
      Serial.print(airTemperatureHistory[i]);
      Serial.print(", "); 
    }
    Serial.print(", index = ");
    Serial.println(historyArraysIndex);
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
    Serial.println("\tT(enclosure):\tT(lid):\tT(encl.slope):\tT(lid.slope):\tT(mse):\tsetPt(lid):\tsetPt(enclosure):\tPWMOut:\tV(in):\tPID:\tError:\tMsg:";); 
  } 
  else 
  {
    Serial.println(" Command not recognized. Press 'h' for index of commands");
  }
}

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
  thisVoltage = (thisVoltage * boardVoltageOut) / 1024.0;   
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
void ErrorCheck(int &thisError)
{
  double lid.deviation = lid.temperature- lid.setpoint; // This was initially deltaFromSetPt and was never declared or set so I did that here

  // Assume all is well
  int newError = 0;
  
  // Check if thermistor is not connected. Temp will read as -273C
  if (glassTemperature < 0) 
  {
    newError = 1;
    // Assume PID will be full throttle. Override to 0.
    PWMOutput = 0;
    if (!isErrorBuffered)
    {
      isErrorBuffered = true;
      errorBuffer += newError;
      errorBuffer += " Glass thermistor disconected. Output shut off.";
    }
  } 

  
  else if (glassTemperature >= maxGlassTemperature) 
  {
    newError = 2;
    // Gently throttle back the power output to reduce temperature
    // While too high output will halve recursively - eventually to 0
    PWMOutput = PWMOutputIfError;
    if (!isErrorBuffered)
    {
      isErrorBuffered = true;
      errorBuffer += newError;
      errorBuffer += " Glass >= max temp. Throttling output to ";
      errorBuffer += String(PWMOutputIfError, 0);
    }
  }

  // Thermal run away. Getting hotter than setpoint and beyond minor overshoot      
  // Do not trigger error if the program is just starting back up after a reset, as the glass will have cooled a bit.
  else if (lid.deviation > 4.0 && (millis() - startUpTime) > errorGraceTime) 
  {
    if (!isErrorBuffered)
    {
      newError = 3;
      errorCodePrevious = newError;
      errorBuffer += newError;
      errorBuffer += " Thermal runaway detected. Throttling output to ";			
      errorBuffer += String(PWMOutputIfError, 0);
    }
  }
  
  else if (isHistoryArraysFilled) 
  {

    // Slope (over 50 samples) when ramping up can be >11 C/min 
    // When stable, always within +/-0.7C/min
    // When off, falling from 35C in ambient temp, goes to ~ -1C/min
//%%% Issue to address. How to avoid this error being thrown if system shuts off, slope<-1, then power comes on and slope transits through 0. 
    // If temp not reaching desired setpoint and slope is relatively flat, assume power is insufficient to reach setpoint
    if (lid.deviation < -1.0 && glassTemperatureSlope < 0.5 && glassTemperatureSlope > -1) 
    {
      if (!isErrorBuffered)
      {
        newError = 4;
        errorBuffer += newError;
  	    errorBuffer += " Under-powered. Increase max output.";
      }
    }

    // Heater failure: temperature falling unexpectedly. Assume power not getting to the heating elements
    else if (lid.deviation < -0.5 && glassTemperatureSlope <= -1 && (millis() - lastGlassSetpointUpdate) >= glassSetpointInterval/2) 
  //%%%need some additional metric to indicate that glass temp is not simply descreasing to a new set point, then some delay to account for stabilization
    {
      if (!isErrorBuffered)
      {
        newError = 5;
        PWMOutput = 0;
  	    errorBuffer += newError;
  	    errorBuffer += " Heater failure. Output - off.";
      }
    }
    else 
    {
      errorBuffer += newError;
    }
  }
	else 
  {
      errorBuffer += newError;
  }

  thisError = newError;
}

void AppendToHistory(double gValue, double aValue)
{
  lid.history[historyArraysIndex] = gValue;
  enclosure.history[historyArraysIndex] = aValue;
  mseGlassTemperatureHistory[historyArraysIndex] = sqrt(sq(gValue-lid.setpoint));
  historyArraysIndex++;
  
  if (historyArraysIndex >= historyArraysSize) 
  {
    historyArraysIndex = 0;
    lid.historyArraysFilled = true;
    enclosure.historyArraysFilled = true;
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

// Print the buck converter voltage to the serial, pass in the buck converter voltage
void PrintParametersToSerial()
{
  // Getting set up to us buffer to merge data with error codes and event logs
  // Need to recall/check syntax for float values with sprintf
  // char logString[];
  // sprintf(logString,"V(in):24.54  T(enclosure):33.36  T(glass):66.11  T(slope):0.04 T(mse):0.047  PWMOut:0.00 PIDKp:1.00  PIDKi:96.00 PIDKd:21.00 PIDmode:1 setPt(glass):66.00"

  if (nLogged == 0) {
    Serial.println(" ");
    logBuffer += "\tT(enclosure):\tT(lid):\tT(encl.slope):\tT(lid.slope):\tT(mse):\tsetPt(lid):\tsetPt(enclosure):\tPWMOut:\tV(in):\tPID:\tError:\tMsg:";
    Serial.println(logBuffer);
    logBuffer = "";
  }

  logBuffer = "";
  logBuffer += "\t";
  logBuffer += String(enclosure.temperature, 2);
  logBuffer += "\t";
  logBuffer += String(lid.temperature, 2);
  logBuffer += "\t";
  logBuffer += String(enclosure.slope, 2);
  logBuffer += "\t";
  logBuffer += String(lid.slope, 2);
  logBuffer += "\t";
  logBuffer += String(mseGlassTemperature, 3);
  logBuffer += "\t";
  logBuffer += String(lid.setpoint, 2);
  logBuffer += "\t";
  logBuffer += String(enclosure.setpoint, 1);
  logBuffer += "\t";
  logBuffer += String(lidHeater.output);
  logBuffer += "\t";
  logBuffer += String(buckConverterVoltage, 2);
  logBuffer += "\t";
  logBuffer += String(lidHeater.P);
  logBuffer += "/";
  logBuffer += String(lidHeater.I);
  logBuffer += "/";
  logBuffer += String(lidHeater.D);
  logBuffer += "\t";
  logBuffer += errorBuffer;
  logBuffer += "\t";
  logBuffer += msgBuffer;
  logBuffer += "\n";
  Serial.print(logBuffer);
  logBuffer = "";
  msgBuffer = " ";
  errorBuffer = " ";
  isErrorBuffered = false;

  nLogged++;
}

void CheckGlassSetpoint()
{
  // Current time in milliseconds since the program has been running
  int currentTimeMilliseconds = millis();

  enclosure.deviation = enclosure.temperature - enclosure.setpoint;
  //airTemperatureDeviation = airTemperature - airTemperatureSetpoint;
  lid.autoSetpointChange = 0;

  // If it has been long enough since the last glass setpoint update and the history array has enough data, check if the glass setpoint needs to be updated
  if ((currentTimeMilliseconds - lastGlassSetpointUpdate) >= glassSetpointInterval && lid.historyFilled == true)
  {
    // Calculate gap from air temperature setpoint

    // If the glass temperature has been stable and the air temperature is significantly higher than the air temperature setpoint, update the glass setpoint
    {

      //If air temp is too high, reduce glass temp, if too low, increase glass temp. This toggles direction of change.
      float deviationDirection = 0.0;
      if (enclosure.deviation>=0) deviationDirection = -1.0;
      if (enclosure.deviation<0) deviationDirection = 1.0;

      // Very basic proportional control of glass temperature relative to air temp
      
      if( abs(enclosure.deviation) > 0.4){
        // Reduce the glass setpoint
        lid.autoSetpointChange = deviationDirection * 2.0;
      }
      else if( abs(enclosure.deviation) > 0.2){
        // Reduce the glass setpoint
        lid.autoSetpointChange = deviationDirection * 1.5;
      }
      else if( abs(enclosure.deviation) > 0.15){
        // Reduce the glass setpoint
        lid.autoSetpointChange = deviationDirection * 0.75;
      }
      else if( abs(enclosure.deviation) > 0.10){
        // Reduce the glass setpoint
        lid.autoSetpointChange = deviationDirection * 0.25;
      }  
      else {
        // Reduce the glass setpoint
        lid.autoSetpointChange = 0.0;
      }  

      //increment the deviation of glassSetpoint from the most recent user-defined glassSetpoint
      lid.cummSetpointChange += lid.autoSetpointChange;
      
      // if the delta is positive, limit the total possible increase to 5C. The user can reset this by manually increasing the set point.
      //The should limit overshoot in the automatic increases in glass set point
      if (lid.autoSetpointChangea>0 && lid.cummSetpointChange >= lid.maxAutoIncrease) {
        lid.cummSetpointChange = lid.maxAutoIncrease;
        lid.autoSetpointChange = 0;
      }

      glassSetpoint += lid.autoSetpointChange;

      // Update the last glass setpoint update time to the current time
      lastGlassSetpointUpdate = currentTimeMilliseconds;

      // Send a message to the serial
      msgBuffer += "Glass Setpoint updated to ";
      msgBuffer += String(glassSetpoint);

    }
  }
}

void RemoveErroneousTemperatureReadings()
{
  // Calculate difference between current glass temperature and previous glass temperature reading
  //double glassTemperatureDataDifference = abs(lid.temperature - lid.prevTtemperature);

  //Since glassTemperatureHistory is a circular buffer, we access the values 1 and 2 indices before the current historyArraysIndex
  if ( (lid.historyFilled == true) && (lid.history[(historyArraysIndex-1)%historyArraysSize] != lid.history[(historyArraysIndex-2)%historyArraysSize]) )
  {
    if ( (abs(lid.temperature - lid.history[(historyArraysIndex-1)%historyArraysSize])>0.25) && (abs(lid.temperature - lid.history[(historyArraysIndex-2)%historyArraysSize])>0.25) ) 
    {
     lid.temperature = lid.history[(historyArraysIndex-2)%historyArraysSize];
      // Send a message to the serial
      msgBuffer += "Erroneous glass temperature data point reassigned";
    }
  }


  /*
    if ( (isHistoryArraysFilled == true) && (airTemperatureHistory[(historyArraysIndex-1)%historyArraysSize] != airTemperatureHistory[(historyArraysIndex-2)%historyArraysSize]) )
  {
    if ( (abs(airTemperature - airTemperatureHistory[(historyArraysIndex-1)%historyArraysSize])>0.4) && (abs(airTemperature - airTemperatureHistory[(historyArraysIndex-2)%historyArraysSize])>0.4) ) 
    {
      //airTemperature = (airTemperatureHistory[(historyArraysIndex-2)%historyArraysSize]+airTemperatureHistory[(historyArraysIndex-1)%historyArraysSize])/2;
      airTemperature = airTemperatureHistory[(historyArraysIndex-3)%historyArraysSize];
      // Send a message to the serial
      msgBuffer += "Erroneous enclosure temperature data point reassigned";
    }
  }
  */
  
  
  // Calculate difference between current air temperature and previous air temperature reading
  //double previousAirTemperatureDataDifference = abs(enclosure.temperature - enclosure.prevTemperature);

  // Calculate difference between current air temperature and oldest air temperature reading
  //double oldestAirTemperatureDataDifference = abs(enclosure.temperature - enclosure.prev2Temperature);

  // If the differences are significant and a previousAirTemperature and oldestAirTemperature value exists, reassign the reading a new value
  if ( (abs(enclosure.temperature - enclosure.prevTemperature)) > 0.25 && (abs(enclosure.temperature - enclosure.prev2Temperature)) > 0.25 && enclosure.prevTemperature != 0 && enclosure.prevwTemperature != 0)
  {
    // Reassign this inaccurate data point to the value of the previous reading
    // Send a message to the serial
    msgBuffer += "Erroneous air temperature (";
    msgBuffer += enclosure.temperature; 
    msgBuffer += ") reassigned, ";
    msgBuffer += " 1 & 2 previous readings were ";
    msgBuffer += enclosure.history[(historyArraysIndex-1)%historyArraysSize];
    msgBuffer += " and ";
    msgBuffer += enclosure.history[(historyArraysIndex-2)%historyArraysSize];
    enclosure.temperature = enclosure.prev2Temperature;
    
  }
  
  
}
