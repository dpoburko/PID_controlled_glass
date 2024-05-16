// *************************************************************************************************************************************
// *************************************************************************************************************************************
// INCUBATOR PID TEMPERATURE CONTROL
// *************************************************************************************************************************************
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
#include <STEINHART.h>

// *************************************************************************************************************************************
// PINS
// *************************************************************************************************************************************

// Analog input for measuring the voltage delivered to the glass heater via a 1/10 voltage divider
#define VOLTMETERPIN A2
// PWM PID output pin to MOSFET (adjust as needed)
#define PWMPINOUT 9
// Thermistor pin
#define THERMISTORPIN A0

// *************************************************************************************************************************************
// PID CONTROLLER VARIABLES
// *************************************************************************************************************************************

// Set glass temperature goal (in degrees Celsius)
double glassSetpoint = 45.0;

// PID proportional, integral, and derivative constants (these values (2, 24, 21) are working well for the 5 mm thick glass)
double PIDKp = 2, PIDKi = 24, PIDKd = 21;

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

// *************************************************************************************************************************************
// SAFETY VARIABLES
// *************************************************************************************************************************************

// Set maximum glass temperature allowed (in degrees Celsius)
const int maxGlassTemperature = 60;

// Distance from the setpoint where PID should start computing
const double gapFromSetpointWherePIDStarts = 1.5;

// Set limits on output to pulse-width modulator (on an 8-bit scale) to limit total current based on limits of the PSU or Buck converter
// Aggressive limit used when the glass temperature is outside gapFromSetpointWherePIDStarts
double maximumAggressivePIDOutput = 25.0;
// Conservative limit used when the glass temperature is inside gapFromSetpointWherePIDStarts
double maximumConservativePIDOutput = 22.0;
// Set current maximum output to pulse-width modulator to the conservative limit initially
double currentMaximumPIDOutput = maximumConservativePIDOutput;

// WHAT ARE THESE
long errorCode = 0; //catch all for errors
bool isErrorAcknowledged = false;
bool isUserOverride = false;

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

// *************************************************************************************************************************************
// GLASS/THERMISTOR VARIABLES
// *************************************************************************************************************************************

// Variable to store the voltage of the glass read by the thermistor
double thermistorVoltage = 0.0;

// Set how often glass voltage will be read (in milliseconds)
int glassVoltageReadingInterval = 2000;

// Variable to store glass temperature (in degrees Celsius)
double glassTemperature = 0.0;

// Variable to store the slope of the glass temperature change
double glassTemperatureSlope = 0.0;

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
const double boardVoltageOut = 3.3;

// Variable to store buck converter voltage
double buckConverterVoltage = 0;

// Set number of samples to take in order to get an average of the voltage data (needs to be a double for average to have decimals)
double numberOfSampleVoltageReadings = 11.0;

// *************************************************************************************************************************************
// CONSTRUCTORS
// *************************************************************************************************************************************

// Create thermistor object, including relevant information about the thermistor and the required constants for the Steinhart equation
STEINHART thermistor1(THERMISTORPIN, &glassTemperature, nominalResistance, nominalTemperature, bCoefficient, seriesResistance);

// Create PID controller for the glass, including relevant information for the PID such as input, output, setpoint, and constants
PID heaterPID(&glassTemperature, &PIDOutput, &glassSetpoint, PIDKp, PIDKi, PIDKd, DIRECT);

// *************************************************************************************************************************************
// SETUP
// *************************************************************************************************************************************

void setup()
{
  // Open serial port, set data rate to 9600 bps
  Serial.begin(9600);
  // Delay helps fix serial glitch
  delay(1000);

  // Initialize communication with I2C devices
  Wire.begin();
 
  // Initialize PID (automaticPIDMode mode)
  PIDMode = automaticPIDMode;
  heaterPID.SetMode(automaticPIDMode);

  // Link PID setpoint with glass setpoint
  PIDSetpoint = glassSetpoint;

  // Set up PID output limits  
  heaterPID.SetOutputLimits(0, maximumAggressivePIDOutput); 

  // Set frequency of voltage readings
  thermistor1.setSampleTime(glassVoltageReadingInterval);

  // Read the temperature from the thermistor
  thermistor1.read();

  // Wait the specified amount of time before reading the temperature again
  delay(glassVoltageReadingInterval);
  thermistor1.read();

  // Print initialization message and temperature to serial
  Serial.print("   ===== PID controlled glass heater =====================");
  Serial.println("   Stage-Top Incubator component"); 
  Serial.print("   Initial Temperature:");
  Serial.println(glassTemperature, 2);
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
    // Get the voltage from the tunable buck converter (what do we do with this?)
    GetBuckConverterVoltage(buckConverterVoltage);

    // Log the glass temperature in an array
    AppendToGlassHistory(glassTemperature);

    // If the arrays are not yet full, calculate the slope of the glass temperature change
    if (isHistoryArraysFilled == false) 
    {
      // If this is the first value in the array, the slope cannot yet be calculated, so set it to zero
      if (historyArraysIndex == 0) 
      {
        glassTemperatureSlope = 0.0;
      } 
      // Why are there two else statements? What do these do?
      else 
      {
        glassTemperatureSlope = (glassTemperature - glassTemperatureHistory[0]) / ((long)(historyArraysIndex+1) * (long)glassVoltageReadingInterval/60000);  //degrees/minute
      }
      //else 
      //{
        //glassTemperatureSlope = (glassTemperature - glassTemperatureHistory[endHistoryArraysIndex]) / (historyArraysSize * glassVoltageReadingInterval/60000);  //degrees/minute
      //}
    }
    
    // Compute the average of the arrays (what do we do with this?)
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
        Serial.print(" setting currentMaximumPIDOutput to ");
        Serial.print(maximumAggressivePIDOutput);
        Serial.println(" (maximumAggressivePIDOutput) for rapid heating");
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
        Serial.print(" setting currentMaximumPIDOutput to ");
        Serial.print(maximumConservativePIDOutput);
        Serial.println(" (maximumConservativePIDOutput) for setpoint maintenance");
      }
    }
  }
  
  // If the PID is in automaticPIDMode mode, compute output
  if (heaterPID.GetMode() == 1) 
  {
    // Link PID input with glass temperature
    PIDInput = glassTemperature;

    // Make sure it is time to recalculate
    isNewPID = heaterPID.Compute();

    // Define output of PID as the pulse-width modulator output (will be at 0 the first time around as the PID needs two data points for computation)
    if (isNewPID == true) 
    {
      PWMOutput = PIDOutput;
    } 
  }
   
  //****** perform a series of tests on thermistor reading and temperature vs setpoint and over time
  //-- override PWMOutput if indicated
  //errorCheck(errorCode);
  //need to concatenate a message to the serial output line
	
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
      else if ((incomingSerial[0] == 'O') & (incomingSerial[0] == 'K')) 
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

  // Delay determines how often loop repeats
  delay(20); 
}

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
        Serial.println(" PID set to automaticPIDMode"); 
      }

      if (incomingSerial[2] == 'm') 
      {
        PIDMode = manualPIDMode;
        heaterPID.SetMode(PIDMode); 
        Serial.println(" PID set to manualPIDMode"); 
      } 
      
      if (incomingSerial[2] == 'g') 
      {
        PIDMode = manualPIDMode;
        heaterPID.SetMode(PIDMode); 
        Serial.print(" PID is in mode: manualPIDMode");  
      }         
    } 
    else if (incomingSerial[1] == 'o') 
    {
      char newOutput[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      PWMOutput = atof(newOutput);
      Serial.print(" PWMOutput: "); 
      Serial.println(PWMOutput);  
    }
    else if (incomingSerial[1] == 's') 
    {
      char newSetpoint[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      glassSetpoint = atof(newSetpoint)/10.0;
      glassSetpoint = glassSetpoint; 
      Serial.print(" glassSetpoint: "); 
      Serial.println(glassSetpoint);  
    }
    else if (incomingSerial[1] == 'l') 
    {
      char upper[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      maximumAggressivePIDOutput = atof(upper);
      heaterPID.SetOutputLimits(0,maximumAggressivePIDOutput);
      Serial.print("   heaterPID.SetOutputLimits(0,"); 
      Serial.print(maximumAggressivePIDOutput);  
      Serial.println(")");     
    }
    else if (incomingSerial[1] == 'h') 
    {
      char holding[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
      maximumConservativePIDOutput = atof(holding);
      Serial.print("   maximumConservativePIDOutput now "); 
      Serial.print(maximumConservativePIDOutput,2);  
      Serial.println(" ");     
    }
    else if (incomingSerial[1] == 'd') 
    {
      char newDelay[4] = {incomingSerial[2],incomingSerial[3],incomingSerial[4],incomingSerial[5]};
      int nd = atol(newDelay);
      heaterPID.SetSampleTime(nd);
      Serial.print(" PID interval now ");
      Serial.println(newDelay);
    }
    else if (incomingSerial[1] == 't') 
    {
      if (incomingSerial[2] == 'p') 
      {
        char ppp[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
        PIDKp = 0.0;
        PIDKp = atol(ppp);
        Serial.print(" Setting P = ");
        Serial.println(PIDKp);
      }
      else if (incomingSerial[2] == 'i') 
      {
        char iii[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
        PIDKi = 0.0;
        PIDKi = atol(iii);
        Serial.print(" Setting I = ");
        Serial.println(PIDKi);
      }
      else if (incomingSerial[2] == 'd') 
      {
        char ddd[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
        PIDKd = 0.0;
        PIDKd = atol(ddd);
        Serial.print(" Setting D = ");
        Serial.println(PIDKd);

      }
      
      heaterPID.SetTunings(PIDKp,PIDKi,PIDKd);
    }
  } 
  else if(incomingSerial[0] == 'T' && incomingSerial[1] == 'p' )
  {
    for(int i = 0; i< historyArraysSize; i++) 
    {
      Serial.print(glassTemperatureHistory[i]);
      Serial.print(", "); 
    }
    
    Serial.println(" ");
  } 
  else if(incomingSerial[0] == 'E' && incomingSerial[1] == 'p' )
  {
    for(int i = 0; i< historyArraysSize; i++) 
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
  if (thisVoltage < 0.1)  {  thisVoltage = 0.0;   }

  newVoltage = thisVoltage;
  
}

void errorCheck(int &thisError)
{
  int newError = 0; //assume all is well
  
  // Check if thermistor is not connected. Temp will read as -273C
  if ( glassTemperature < 0) 
  {
    newError = 1;
    //Assume PID will be full throttle. Overrie to 0.
    PWMOutput = 0;
  } 

  
  if (glassTemperature >= maxGlassTemperature) 
  {
    newError = 1;
    //gently throttle back the power output to reduce temperature
    //while too high output will half recursively - eventually to 0.
    //does
    PWMOutput /= 2;

    Serial.println("Error Code: 1 - max glass temp reached. Throttling back output");
  }
  
  if (isHistoryArraysFilled) 
  {

    // calculate: delta over history, slope in C/min, check if at max temp
    double deltaOverTime = glassTemperature - glassTemperatureHistory[endHistoryArraysIndex];
    double deltaFromSetPt = glassTemperature - glassSetpoint;
    //slope (over 50 samples) when ramping up can be >3 C/min, 
    //when stable, always within +/-0.7C/min
    //when off, falling from 35C in ambient temp, goes to ~ -1C/min
     
    //Thermal run away. Getting hotter than setpoint and beyond minor overshoot      
    if (deltaFromSetPt>2.0) 
    {
      newError = 2;
      Serial.println("Error Code: 2 - Thermal Run away detected. Throttling back"); 
      //Throttle back to give a chance to self-correct
      PWMOutput /= 2;  
    }

    //If temp not reaching desired setpoint and slope is relatively flat, 
    // assume power is insufficient to reach setpoint
    if (deltaFromSetPt < -0.5 && glassTemperatureSlope < 0.5 && glassTemperatureSlope > -1) 
    {
      newError = 3;
      Serial.println("Error Code: 3 - Under-powered. Consider increasing max output or optimizing P constant"); 
    }

    // Heater failure: temperature falling unexpectedly when in auto mode
    // assume power is insufficient to reach setpoint
    if (deltaFromSetPt < -0.5 && glassTemperatureSlope <= -1  ) 
    {
      newError = 4;
      Serial.println("Error Code: 4 - Heater appears to have failed. "); 
    }
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

void GetArrayAverage(double thisArray[], double &average)
{
  double arrAvg = 0.0;
  for (int i = 0; i<historyArraysSize; i++) 
  {
    arrAvg += thisArray[i];
  }
  if (isHistoryArraysFilled) 
  {
    arrAvg /= historyArraysSize;   
  } 
  else 
  {
    arrAvg /= long(historyArraysIndex+1);
  }

  average = arrAvg;
}

// Print the buck converter voltage to the serial, pass in the buck converter voltage
void PrintParametersToSerial()
{
  Serial.print("V(in):");    
  Serial.print(buckConverterVoltage,2);
  Serial.print("\tT(glass):");
  Serial.print(glassTemperature,2);
  Serial.print("\tT(slope):");
  Serial.print(glassTemperatureSlope,2);
  Serial.print("\tT(mse):");
  Serial.print(mseGlassTemperature,3);
  Serial.print("\tPWMOut:");
  Serial.print(PWMOutput);
  Serial.print("\tPIDKp:");
  Serial.print(PIDKp);
  Serial.print("\tPIDKi:");
  Serial.print(PIDKi);
  Serial.print("\tPIDKd:");
  Serial.print(PIDKd);
  Serial.print("\tPIDMode:");
  if (PIDMode == manualPIDMode)
  {
    Serial.print("manualPIDMode");
  }
  else
  {
    Serial.print("automaticPIDMode");
  }
  Serial.print("\tsetPt(glass):");
  Serial.print(glassSetpoint);
  Serial.print("\tError:");
  Serial.println(errorCode);
}
