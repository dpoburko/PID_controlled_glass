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

// *************************************************************************************************************************************
// PID CONTROLLER VARIABLES
// *************************************************************************************************************************************

// Set glass temperature goal (in degrees Celsius)
double glassSetpoint = 45.0;

// PID proportional, integral, and derivative constants (these values (2, 96, 21) are working well for the 5 mm thick glass)
double PIDKp = 2, PIDKi = 96, PIDKd = 21;

// Define setpoint, input, and output variables (from PID library)
double PIDSetpoint, PIDInput, PIDOutput;

// Enumerate PID modes
enum enumPIDMode {
  manualPIDMode,
  automaticPIDMode
};

enumPIDMode PIDMode;

// Check if output has changed since last loop
bool isNewPID = false;

// Interval for updating glass setpoint (10 minutes or 600000 milliseconds)
int glassSetpointInterval = 150000;

// Variable to store when the last glass setpoint update happened
int lastGlassSetpointUpdate = 0;

// *************************************************************************************************************************************
// SAFETY VARIABLES
// *************************************************************************************************************************************

// Set maximum glass temperature allowed (in degrees Celsius)
const int maxGlassTemperature = 65;

// Distance from the setpoint where PID should start computing
const double gapFromSetpointWherePIDStarts = 1.5;

// Set limits on output to pulse-width modulator (on an 8-bit scale) to limit total current based on limits of the PSU or Buck converter
// Aggressive limit used when the glass temperature is outside gapFromSetpointWherePIDStarts
double maximumAggressivePIDOutput = 45.0;

// Conservative limit used when the glass temperature is inside gapFromSetpointWherePIDStarts
double maximumConservativePIDOutput = 40.0;

// Set current maximum output to pulse-width modulator to the conservative limit initially
double currentMaximumPIDOutput = maximumConservativePIDOutput;

//%%% to ErrorCheck library
// Error variables
int errorCode = 0; // Izzy's edit: errorCode was initially a long but I changed it to an int since ErrorCheck() requires a parameter of type int
int errorCodePrevious = 0; // keep track of previous error code
double timeErrorAcknowledged =0; //for keeping track of when error was acknowledged 
bool isErrorAcknowledged = true;
bool isErrorBuffered = false;
bool isUserOverride = false;
double errorGraceTime = 180000; //in ms
long startUpTime;

bool errorCodesActive[] = {false,false,false,false,false,false,false,false,false,false}; // keep track of previous error code
double timesErrorsAcknowledged[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; //for keeping track of when error was acknowledged 
bool areErrorsAcknowledged[] = {true,true,true,true,true,true,true,true,true,true};

String errorBuffer,  msgBuffer, logBuffer;

// *************************************************************************************************************************************
// STEINHART EQUATION VARIABLES
// *************************************************************************************************************************************

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

// Pulse-width modulator output value needed to change glass temperature
double PWMOutput = 0.0;

// Last pulse-width modulator output value
double PWMOutputLast = 0.0;

//%%% to ErrorCheck library?
// Value used to hold glass temperature warm but safe
double PWMOutputIfError = 10.0;

// *************************************************************************************************************************************
// GLASS/THERMISTOR VARIABLES
// *************************************************************************************************************************************

// Variable to store the voltage of the glass read by the thermistor
double thermistorVoltage = 0.0;

// Set how often glass voltage will be read (in milliseconds)
int glassVoltageReadingInterval = 2000;

// Variable to store glass temperature (in degrees Celsius)
double glassTemperature = 0.0;

// Variable to store previous glass temperature (in degrees Celsius)
double previousGlassTemperature = 0.0;

// Variable to store the slope of the glass temperature change
double glassTemperatureSlope = 0.0;

// Variable to store air temperature
double airTemperature = 20.0;

// Variable to store previous air temperature
double previousAirTemperature = 0.0;

// Air temperature goal
double airTemperatureSetpoint = 37.0;

// Allowable difference betwee Air temperature and airTemperatureSetpoint
double airTemperatureTolerance = 0.1;

// Boolean to check if the air temperature setpoint has been reached
bool isAirTemperatureSetpointReached = false;

// yes if Air temperature < airTemperatureSetpoint and reassessed at start-up and changes of airTemperatureSetpoint
bool isAirTemperatureClimbing = true;

// Size of arrays used for logging temperature and temperature error
const int historyArraysSize = 60;

// Variable to store a counter for the index of the temperature and temperature error arrays
int historyArraysIndex = 0;

// Variable to store when the index of the temperature and temperature error arrays ends
int endHistoryArraysIndex = 0;

// Variable to store whether or not the temperature and temperature error arrays have been filled
bool isHistoryArraysFilled = false;

// Array to store glass temperature
double glassTemperatureHistory[historyArraysSize];

// Array to store glass temperature
double airTemperatureHistory[historyArraysSize];

// Array to store mean square error of glass temperature
double mseGlassTemperatureHistory[historyArraysSize];

// Variable to store the mean square error of the glass temperature
double mseGlassTemperature = 0.0;

// Boolean to check whether or not there is a new glass temperature
bool isNewGlassTemperature = false;

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
STEINHART thermistor1(THERMISTOR1PIN, &glassTemperature, nominalResistance, nominalTemperature, bCoefficient, seriesResistance);
STEINHART thermistor2(THERMISTOR2PIN, &airTemperature, nominalResistance, nominalTemperature, bCoefficient, seriesResistance);

// Create PID controller for the glass, including relevant information for the PID such as input, output, setpoint, and constants
PID heaterPID(&glassTemperature, &PIDOutput, &glassSetpoint, PIDKp, PIDKi, PIDKd, DIRECT);

// *************************************************************************************************************************************
// SETUP
// *************************************************************************************************************************************

void setup()
{
  // Open serial port, set data rate to 57600 bps
  Serial.begin(57600);	
  // Delay helps fix serial glitch
  delay(1000);

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
  PIDSetpoint = glassSetpoint;

  // Set up PID output limits  
  heaterPID.SetOutputLimits(0, maximumAggressivePIDOutput); 
  heaterPID.SetSampleTime(glassVoltageReadingInterval); 

  // Set frequency of voltage readings for thermistor 1
  thermistor1.setSampleTime(glassVoltageReadingInterval);

  // Read the temperature from thermistor 1
  thermistor1.read();

  // Repeat for thermistor 2
  thermistor2.setSampleTime(glassVoltageReadingInterval);
  thermistor2.read();

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
    // Read temperature from thermistor 2
    thermistor2.read();

    // Get the voltage from the tunable buck converter (what do we do with this?)
    GetBuckConverterVoltage(buckConverterVoltage);

    // Check to make sure the glass temperature data point makes sense, and reassign the value if necessary
    RemoveErroneousGlassTemperatureReadings();

    // Check to make sure the air temperature data point makes sense, and reassign the value if necessary
    RemoveErroneousAirTemperatureReadings();
    
    // Log the glass temperature in an array
    AppendToGlassHistory(glassTemperature);

    //%%% - Can be removed. No longer needed with Izzy's method of erroneous reading check
    AppendToAirHistory(airTemperature);
    
    // If the arrays are not yet full, calculate the slope of the glass temperature change
    if (isHistoryArraysFilled == false) 
    {
      // If this is the first value in the array, the slope cannot yet be calculated, so set it to zero
      if (historyArraysIndex == 0) 
      {
        glassTemperatureSlope = 0.0;
      } 
      else 
      {
        // In degrees/minute
        glassTemperatureSlope = (glassTemperature - glassTemperatureHistory[0]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);
      }
    }
    else 
      {
        // In degrees/minute
        glassTemperatureSlope = (glassTemperature - glassTemperatureHistory[endHistoryArraysIndex]) / (historyArraysSize * glassVoltageReadingInterval/60000);
      }
    
    // Compute the average of the arrays
    GetArrayAverage(mseGlassTemperatureHistory, mseGlassTemperature);

    // If glass temperature is far from the setpoint, set the maximum PID output to a pre-determined aggressive value
    if (glassTemperature < (glassSetpoint - gapFromSetpointWherePIDStarts))
    {
      // If the current maximum PID output is not already set to aggressive, set it to aggressive and print a message to the serial
      if (currentMaximumPIDOutput != maximumAggressivePIDOutput) 
      {
        // Set the current PID output to aggressive
        currentMaximumPIDOutput = maximumAggressivePIDOutput;
        heaterPID.SetOutputLimits(0, currentMaximumPIDOutput);

        // Print a message to the serial
        msgBuffer += " Switching to maximumAggressivePIDOutput_";
	msgBuffer += String(maximumAggressivePIDOutput);
      } 
    }
    // If the glass temperature is close to the setpoint, set the maximum PID output to a pre-determined conservative value 
    else 
    {
      // If the current maximum PID output is not already set to conservative, set it to conservative and print a message to the serial
      if (currentMaximumPIDOutput != maximumConservativePIDOutput) 
      {
        // Set the current PID output to conservative
        currentMaximumPIDOutput = maximumConservativePIDOutput;
        heaterPID.SetOutputLimits(0, currentMaximumPIDOutput);

        // Print a message to the serial
        msgBuffer += " Switch to maximumConservativePIDOutput_";
	msgBuffer += String(maximumConservativePIDOutput);
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
	PWMOutput = PIDOutput;
	// Perform a series of tests on thermistor reading and temperature vs. setpoint and over time
        ErrorCheck(errorCode);
    } 
  }

   //%%% @Izzy, I have just noticed that we have no errorChecking when in manual mode. This seems prone to troubles. Let's rethink this.
	
	
  // Check if the air temperature is sufficiently heated
  if (isAirTemperatureClimbing == true)
  {
    //if air temp needs to increase to reach set point
    if (airTemperature >= airTemperatureSetpoint)
    {
      isAirTemperatureSetpointReached = true;
      if (isAirTemperatureSetpointNoted == false) {
        msgBuffer += " air temperature set point reached"; 
        isAirTemperatureSetpointNoted = true;
      }
    }
  } else {
    //assume air temp needs to fall to reach set point
    if (airTemperature <= airTemperatureSetpoint)
    {
      isAirTemperatureSetpointReached = true;
      if (isAirTemperatureSetpointNoted == false) {
        msgBuffer += " air temperature set point reached"; 
        isAirTemperatureSetpointNoted = true;
      }
    }
  }
  
  // Check to see if the glass setpoint needs to be updated, only if airTemperatureSetpointReached is true
  if (isAirTemperatureSetpointReached)
  {
    CheckGlassSetpoint();
  }
	
  // If the output from the PID is different from the previous output, adjust the pulse-width modulator duty cycle
  if (PWMOutput!= PWMOutputLast) 
  {
    // Adjust the duty cycle of the PWM pin connected to the MOSFET fron the PID output if the value has changed
    analogWrite(PWMPINOUT, PWMOutput);

    // Log the new output value as the last output value received
    PWMOutputLast = PWMOutput;
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
      if (incomingSerial[0] == 'P' || incomingSerial[0] == 'T'|| incomingSerial[0] == 'E') 
      {
        // Go to parse function to read the next message inputted
        ParsePIDCmd();

        // User override
        isUserOverride = true;  
      }
      // If user inputted OK, indicate that the error acknowledgement has been received
      else if ((incomingSerial[0] == 'O') & (incomingSerial[1] == 'K')) 
      {
        // Print acknowledgement to serial
        Serial.println("OK. Error acknowledgement received");

        // Set error acknowledged boolean to true
        isErrorAcknowledged = true; 
        
      }
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
        Serial.println("   Ponnn. - manually set PWMOutput (only usual in manualPIDMode mode");
        Serial.println("   Plnnn. - call heaterPID.SetOutputLimits(0, nnn)");
        Serial.println("   Phnnn. - updated holding PWM");
        Serial.println("   Ptpnnn. - heaterPID.SetTunings(nnn,iii,ddd)");
        Serial.println("   Ptinnn. - heaterPID.SetTunings(ppp,nnn,ddd)");
        Serial.println("   Ptdnnn. - heaterPID.SetTunings(ppp,iii,nnn)");
        Serial.println("   Pdnnnn. - call heaterPID.SetSampleTime(nnnn) & STEINHART::setSampleTime(nnnn)");
        Serial.println("   Tp - print temperature history");
        Serial.println("   Ep - print mse history");
        Serial.println("   >>>>>>>>>>>>>><<<<<<<<<<<<<<<<");
      }
      // If another message is inputted, print to the serial that the message was not recognized
      else 
      {
        Serial.println("   Serial command not recognized. Press 'h' for help");
      }
    }
  }

  // Set the previousGlassTemperature value to the current glass temperature
  previousGlassTemperature = glassTemperature;

  // Set the previousAirTemperature value to the current air temperature
  previousAirTemperature = airTemperature;

  // Set the oldestAirTemperature value to the previousAirTemperature value
  oldestAirTemperature = previousAirTemperature;

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
      
      double newAirTemperatureSetpoint = atof(newSetpoint)/10.0;

      airTemperatureSetpoint = newAirTemperatureSetpoint;

      if (airTemperature < airTemperatureSetpoint) {
        isAirTemperatureClimbing  = true;
      } else {
        isAirTemperatureClimbing  = false;
      }
      //glassSetpoint = glassSetpoint; 
      msgBuffer += " airTemperatureSetpoint now ";
      msgBuffer += String(airTemperatureSetpoint,1); 
      //isAirTemperatureSetpointReached = false;
      isAirTemperatureSetpointNoted = false;
      
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
  else if(incomingSerial[0] == 'T' && incomingSerial[1] == 'p' )
  {
    for(int i = 0; i < historyArraysSize; i++) 
    {
      Serial.print(glassTemperatureHistory[i]);
      Serial.print(", "); 
    }
    Serial.println(" ");
  } 
  else if(incomingSerial[0] == 'E' && incomingSerial[1] == 'p' )
  {
    for(int i = 0; i < historyArraysSize; i++) 
    {
      Serial.print(mseGlassTemperatureHistory[i]);
      Serial.print(", "); 
    }
    Serial.println(" ");
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
  double glassTemperatureGapFromSetpoint = glassTemperature - glassSetpoint; // This was initially deltaFromSetPt and was never declared or set so I did that here

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
  else if (glassTemperatureGapFromSetpoint > 4.0 && (millis() - startUpTime) > errorGraceTime) 
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
    if (glassTemperatureGapFromSetpoint < -0.5 && glassTemperatureSlope < 0.5 && glassTemperatureSlope > -1) 
    {
      if (!isErrorBuffered)
      {
        newError = 4;
        errorBuffer += newError;
  	errorBuffer += " Under-powered. Increase max output.";
      }
    }

    // Heater failure: temperature falling unexpectedly. Assume power not getting to the heating elements
//%%% - need an exception for case where system shuts off, slope<-1, then system restarts an needs to turn on.
    else if (glassTemperatureGapFromSetpoint < -0.5 && glassTemperatureSlope <= -1) 
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

void AppendToGlassHistory(double value)
{
  glassTemperatureHistory[historyArraysIndex] = value;
  mseGlassTemperatureHistory[historyArraysIndex] = sqrt(sq(value-glassSetpoint));
  historyArraysIndex++;
  
  if (historyArraysIndex >= historyArraysSize) 
  {
    historyArraysIndex = 0;
    isHistoryArraysFilled = true;
  }
  endHistoryArraysIndex = historyArraysIndex + 1;

  if (historyArraysIndex == historyArraysSize - 1) 
  {
    endHistoryArraysIndex = 0;
  }
}

//%%% Deprecated? 
void AppendToAirHistory(double value)
{
  //indexing is goverend by AppendToGlassHistory()
  airTemperatureHistory[historyArraysIndex] = value;
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
    logBuffer += "T(enclosure):\tT(glass):\tT(slope):\tT(mse):\tsetPt(glass):\tsetPt(air):\tPWMOut:\tV(in):\tPID:\tError:\tMsg:";
    Serial.println(logBuffer);
    logBuffer = "";
  }

  logBuffer = "";
  logBuffer += "\t";
  logBuffer += String(airTemperature, 2);
  logBuffer += "\t";
  logBuffer += String(glassTemperature, 2);
  logBuffer += "\t";
  logBuffer += String(glassTemperatureSlope, 2);
  logBuffer += "\t";
  logBuffer += String(mseGlassTemperature, 3);
  logBuffer += "\t";
  logBuffer += String(glassSetpoint, 2);
  logBuffer += "\t";
  logBuffer += String(airTemperatureSetpoint, 1);
  logBuffer += "\t";
  logBuffer += String(PWMOutput);
  logBuffer += "\t";
  logBuffer += String(buckConverterVoltage, 2);
  logBuffer += "\t";
  logBuffer += String(PIDKp);
  logBuffer += "/";
  logBuffer += String(PIDKi);
  logBuffer += "/";
  logBuffer += String(PIDKd);
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

  // If it has been long enough since the last glass setpoint update and the history array has enough data, check if the glass setpoint needs to be updated
  if ((currentTimeMilliseconds - lastGlassSetpointUpdate) >= glassSetpointInterval && isHistoryArraysFilled == true)
  {
    // Calculate gap from air temperature setpoint

    // If the glass temperature has been stable and the air temperature is significantly higher than the air temperature setpoint, update the glass setpoint
    {

      float deviationDirection = 0.0;
      if (airTemperatureDeviation>=0) deviationDirection = -1.0;
      if (airTemperatureDeviation<0) deviationDirection = 1.0;
      
      if( abs(airTemperatureDeviation) > 0.2){
        // Reduce the glass setpoint
        glassSetpoint = glassSetpoint + (deviationDirection * 1.5);
      }
      else if( abs(airTemperatureDeviation) > 0.15){
        // Reduce the glass setpoint
        glassSetpoint = glassSetpoint + (deviationDirection * 0.75);
      }
      else if( abs(airTemperatureDeviation) > 0.0){
        // Reduce the glass setpoint
        glassSetpoint = glassSetpoint + (deviationDirection * 0.25);
      }  

      // Update the last glass setpoint update time to the current time
      lastGlassSetpointUpdate = currentTimeMilliseconds;

      // Send a message to the serial
      msgBuffer += "Glass Setpoint updated to ";
      msgBuffer += String(glassSetpoint);
      lastGlassSetpointUpdate = currentTimeMilliseconds;
    }
  }
}

void RemoveErroneousGlassTemperatureReadings()
{
  // Calculate difference between current glass temperature and previous glass temperature reading
  double glassTemperatureDataDifference = abs(glassTemperature - previousGlassTemperature);

  //Since glassTemperatureHistory is a circular buffer, we access the values 1 and 2 indices before the current historyArraysIndex
  if ( (isHistoryArraysFilled == true) && (glassTemperatureHistory[(historyArraysIndex-1)%historyArraysSize] != glassTemperatureHistory[(historyArraysIndex-2)%historyArraysSize]) )
  {
    if ( (abs(glassTemperature - glassTemperatureHistory[(historyArraysIndex-1)%historyArraysSize])>0.25) && (abs(glassTemperature - glassTemperatureHistory[(historyArraysIndex-2)%historyArraysSize])>0.25) ) 
    {
      glassTemperature = glassTemperatureHistory[(historyArraysIndex-1)%historyArraysSize];
    }
    // Send a message to the serial
    msgBuffer += "Erroneous glass temperature data point reassigned";
  }
}

void RemoveErroneousAirTemperatureReadings()
{
  // Calculate difference between current air temperature and previous air temperature reading
  double previousAirTemperatureDataDifference = abs(airTemperature - previousAirTemperature);

  // Calculate difference between current air temperature and oldest air temperature reading
  double oldestAirTemperatureDataDifference = abs(airTemperature - oldestAirTemperatureDataDifference);

  // If the differences are significant and a previousAirTemperature and oldestAirTemperature value exists, reassign the reading a new value
  if (previousAirTemperatureDataDifference > 0.25 && oldestAirTemperatureDataDifference > 0.25 && previousAirTemperature != 0 && oldestAirTemperature != 0)
  {
    // Reassign this inaccurate data point to the value of the previous reading
    airTemperature = previousAirTemperature;

    // Send a message to the serial
    msgBuffer += "Erroneous air temperature data point reassigned";
  }
}
