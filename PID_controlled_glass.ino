// For wiring details of voltmeter see:
// https://www.digikey.ca/en/maker/projects/how-to-make-a-simple-digital-voltmeter-with-an-arduino/082dff9d725549aea8bf84a7e302c1b2
// For thermistor details see:
// https://learn.adafruit.com/thermistor/using-a-thermistor

// PID control library (must be downloaded)
#include <PID_v1.h>
// Wire library for I2C communication (must be downloaded)
#include <Wire.h>
#include "STEINHART.h"

//indicate the voltage of the dev board digital outputs.
const double boardVout = 5.0;

// Analog input for measuring the voltage delivered to the glass heater via a 1/10 voltage divider
#define VOLTMETERPIN A2
// PWM PID output PIN to MOSFET
#define pwmPinOut 9 //adjust as needed

// Set glass temperature goal (in degrees Celsius) and max temp allowed
const double glassSetpoint = 35.0;
const double maxGlassTemperature = 60.0;
double PIDStartDelta = 6.0;

// Set heater limits (bits), these are arbitrary values for now
const double outPutMax = 25.0; // Currently set to limit total current to the limits of the PSU or Buck converter. 
//const double aggressiveUpperHeaterLimit = outPutMax;
//const double conservativeUpperHeaterLimit = outPutMax;

// Variable to store current PWMoutput limit
double currentUpperHeaterLimit = outPutMax;

// Set PID constants (aggressive and conservative), these are arbitrary values for now
//const double aggressivePIDKp = 25, aggressivePIDKi = 10, aggressivePIDKd = 0;
//const double conservativePIDKp = 25, conservativePIDKi = 10, conservativePIDKd = 0;
const double PIDKp = 25, PIDKi = 10, PIDKd = 0;
//double currentPIDKp = conservativePIDKp;
//double currentPIDKi = conservativePIDKi;
//double currentPIDKd = conservativePIDKd;

// Define PID variables (from PID library)
double PIDSetpoint, PIDInput, PIDOutput;
double PWMoutput = 0.0;
double PWMoutputLast = 0.0;

bool PIDmode = 0;
bool safeMode = false;
bool newPID = false;

// Create PID object (start with conservative tuning constants to be safe)
PID heaterPID(&PIDInput, &PIDOutput, &PIDSetpoint, PIDKp, PIDKi, PIDKd, DIRECT);

// Resistors r1 and r2 used for ~1:10 voltage divider to detection input voltage
const double r1Coefficient = 9810.0;
const double r2Coefficient = 983.0;
double buckConverterVoltage = 0;

// Set number of samples to take in order to get an average of the voltage and temperature data (more samples takes longer but 
// is more smooth)
const int nSampleReadings = 10;

// Create a counter for the amount of reads taken
int readCounter = 0;

// Arrays to log tunable buck converter voltage data and thermistor voltage data
//uint16_t thermistorVoltagesArray[nSampleReadings];

//=================================================================================
//Variables for the thermistor attached to the glass lid

int thermistorPin = A0; // // Thermistor pin
double glassTemperature = 0.0; // Initialize glass temperature (in degrees Celsius) and PWMoutput value needed to change glass temperature
int glassInterval = 2000;
const int historySize = 50;
int historyIndex =0;
bool historyFilled = false;
double glassTempHistory[historySize];// Create glass temperature array (of size 50 for now)

double thermistorVoltage = 0.0;
long Rnominal = 10000;  // Thermistor resistance at 25 C 
long Tnominal = 25; // Temperature for nominal resistance (almost always 25 C)
long bCoeff = 3435; // B coefficient for Steinhart equation 
long Rseries = 9985; // measured R for whatever seriers resistor (~10 kOhms) is used

//Only the glass temp will change and is usefull to have a pointer
STEINHART thermistor1(thermistorPin, &glassTemperature, Rnominal, Tnominal, bCoeff, Rseries);
//=================================================================================

int displayInterval = 2000;
int displayLast = 0;

void setup()
{
  // Open serial port, set data rate to 9600 bps
  Serial.begin(9600);

  // Delay helps fix serial glitch
  delay(1000);

  // Initialize desired glass temperature
  PIDSetpoint = glassSetpoint;

  // Initialize communication with I2C devices
  Wire.begin();

  // Initialize PID
  heaterPID.SetMode(AUTOMATIC);
  PIDmode =1;     
  heaterPID.SetOutputLimits(0, outPutMax); //set to limits of 

  thermistor1.setSampleTime(glassInterval);
  thermistor1.read();
  // Start at conservative upper heater limit to be safe
  //currentUpperHeaterLimit = conservativeUpperHeaterLimit;
}

void loop() 
{

  // Get the voltage from the thermistor
  //double thermistorVoltage = GetThermistorVoltage(); //moved to library

  // Calculate the glass temperature using the Steinhart equation
  //glassTemperature = GetTemperatureSteinhartEquation(thermistorVoltage);

  //this is incorrect. Need to shift toward end of array. Newest value should be 0th and have a max array length
  //***** Consider using circular buffer library.
  // **** to check if a value x observations earlier use index of i%buffer size. 
  
  //call STEINHART library, check if value is updated  
  bool newTemp = thermistor1.read();
  
  //add the new temp reading to the history array if new value available
  if (newTemp) {
    
    // Get the voltage from the tunable buck converter - burried here to match temp check interval
    buckConverterVoltage = GetBuckConverterVoltage();

     appendToGlassHistory(glassTemperature);
     // Define input of the PID as the glass temperature
     PIDInput = glassTemperature;
  }

  //When initially warming, if glass is too cold, go full throttle
  if ( glassTemperature < (glassSetpoint-PIDStartDelta)) {
      heaterPID.SetMode(MANUAL);
      PWMoutput = outPutMax;
      PIDmode = 0;
  
  //If thermistor is disconnected, the temp will read as -273
  } else if ( glassTemperature < 0) {
      heaterPID.SetMode(MANUAL);
      PWMoutput = 0;
      PIDmode = 0;
      Serial.println("Error: thermistor unplugged");
      //create a user input to continue and exit safet mode
      safeMode = true;
      
  //Otherwise, check if it needs to be set to Auto and compute  
  } else {
    // Compute PID loop (this is a function from the PID library)
    if (heaterPID.GetMode()==0) {
      heaterPID.SetMode(AUTOMATIC); 
      PIDmode =1;
      PWMoutput = 0;     
    } 
    
    newPID = heaterPID.Compute();
    // Define output of PID as the PWMoutput (will be at 0 the first time around as the PID needs two data points 
    // for computation)
    if (newPID ==true) PWMoutput = PIDOutput;    
  }

  if (PWMoutput!= PWMoutputLast) {
    // Adjust the duty cycle of the PWM pin connected to the MOSFET fron the PID output if the value has changed
    analogWrite(pwmPinOut, PWMoutput);
    PWMoutputLast = PWMoutput;
  }

  

/*
  // Add glass temperature check for safety reasons
  if (!IsGlassTemperatureSafe())
  {
    // Glass temperature limit exceeded - shut down the program
    SafetyShutDown("Glass too hot");
  }

  // Check for significant temperature drop (may indicate thermistor has detached), need at least two reads
  if (readCounter > 0)
  {
    if (checkGlassTemp())
    {
      // Significant glass temperature drop - shut down the program
      SafetyShutDown("Temp drop");
    }
  }

  // Check for significant temperature increase (may indicate safety issue), need at least two reads
  if (readCounter > 0)
  {
    if (HasGlassTemperatureIncreased())
    {
      // Significant glass temperature increase - shut down the program
      SafetyShutDown("Temp increase");
    }
  }
*/

  // Decide whether to use aggressive or conservative tuning constants based on the distance from the setpoint before computing PID
  // DP deprecated (scheduled for removal). Changing the tuning is probably not optimal. Better to consider changing max temp or max output
  //SetPIDTuningFromSetpointGap();

  // Adjust PWMoutput if it is outside of the desired limits
  //AdjustPWMoutputWhenOutsideLimits();

  // Print buck converter voltage, glass temperature, and PWMoutput to serial
  int tNow = millis();
  if ( (tNow - displayLast) >= displayInterval) {
    displayLast = tNow;
    printParametersToSerial();
    
  }
  

  // Delay determines how often loop repeats
  delay(20); 
}

// Get the average voltage from the tunable buck converter, return tunable buck converter voltage
double GetBuckConverterVoltage()
{
  // Read voltage from voltmeter 
  double buckConverterVoltage;
	
  // Average nSampleReadings from heater voltage source with a slight delay
  for (int i = 0; i < nSampleReadings; i++) 
  {
    buckConverterVoltage += analogRead(VOLTMETERPIN)/ nSampleReadings;
    delay(5);
  }
 
  // Convert the 0-1024 analog input to a voltage
  buckConverterVoltage = (buckConverterVoltage * boardVout) / 1024.0;   
  //Correct for the voltage-divide upstream of the measured voltage
  buckConverterVoltage = buckConverterVoltage / (r2Coefficient / (r1Coefficient + r2Coefficient));

  // If voltage is very small, consider it negligible and set it to 0        
  if (buckConverterVoltage < 0.1)
  {     
    buckConverterVoltage = 0.0;    
  }

  return buckConverterVoltage;
}

// Get the average voltage from the thermistor, return the average thermistor voltage
/*double GetThermistorVoltage()
{
  // Take a predetermined number of thermistor voltage samples in a row, with a slight delay
  for (int i = 0; i < nSampleReadings; i++) 
  {
    thermistorVoltagesArray[i] = analogRead(THERMISTORPIN);
    delay(5);
  }

  double sumOfThermistorVoltageSamples = 0;

  // Sum all the logged thermistor voltage data
  for (int i = 0; i < nSampleReadings; i++) 
  {
    sumOfThermistorVoltageSamples += thermistorVoltagesArray[i];
  }

  // Take the average of the thermistor voltages by dividing by the number of samples
  double averageThermistorVoltage = 0;
  averageThermistorVoltage = sumOfThermistorVoltageSamples / nSampleReadings;

  // Do something?
  averageThermistorVoltage = 1023 / averageThermistorVoltage - 1;
  
  return averageThermistorVoltage;
}


// Calculate glass temperature using the Steinhart equation, pass in thermistor voltage, return glass temperature
double GetTemperatureSteinhartEquation(double thermistorVoltage)
{
  // Convert the thermistor voltage to resistance
  double resistance = SERIESRESISTOR / thermistorVoltage;
  // R / Ro
  double steinhartTemporaryVariable = resistance / THERMISTORNOMINAL;
  // ln(R / Ro)
  steinhartTemporaryVariable = log(steinhartTemporaryVariable);
  // 1 / B * ln(R / Ro)
  steinhartTemporaryVariable /= BSTEINHARTCOEFFICIENT;
  // + (1 / To), convert To to Celsius from Kelvin
  steinhartTemporaryVariable += 1.0 / (TEMPERATURENOMINAL + 273.150);
  // Invert
  steinhartTemporaryVariable = 1.0 / steinhartTemporaryVariable;
  // Convert from Kelvin to Celsius
  steinhartTemporaryVariable -= 273.150;

  return steinhartTemporaryVariable;
}
*/
/*
void SafetyShutDown(String errorMessage)
{
  // Turn off heater (stop any input) - need to actually communicate this to heater
  PWMoutput = 0;

  // Print alert on serial
  Serial.println(errorMessage);
  Serial.println("System shut down");

  // Delay gives serial time to print the message before shutting down the system
  delay(100);

  // Exit main program loop
  //exit(0);
}

// Check for a significant glass temperature drop
// We can expect temp changes on the glass to be slow. 
*/
bool checkGlassTemp()
{
  bool isGlassTemperatureSafe = false;
  bool hasGlassTemperatureDropped = true;
  bool hasGlassTemperatureIncreased = true;
  //double glassTemperatureGap = glassTemperaturesArray[readCounter] - glassTemperaturesArray[readCounter - 1];

  //check if temp falling, suggesting a heater failure/short 
  if (historyFilled) {
    //check if temp falling despite heat being applied
      
      double deltaT = glassTemperature - getGlassHistory( (historyIndex - historySize)%historySize);
      if (deltaT>1.5) {
        Serial.print("Potential Thermal Run away"); 
        safeMode = true;
        //create a user input to continue and exit safet mode
        hasGlassTemperatureDropped = true;
      }
      if (deltaT<1.5) {
        Serial.print("Glass temp increasing"); 

        //create a user input to continue and exit safet mode
        hasGlassTemperatureIncreased = true;
      }

  }
  
  // Make sure glass temperature is equal to or below the maximum allowed temperature
  if (glassTemperature <= maxGlassTemperature)
  {
    isGlassTemperatureSafe = true;
  }

  // Return safety status
  return isGlassTemperatureSafe;
  return hasGlassTemperatureDropped;
  return hasGlassTemperatureIncreased;
  
}



// Based on how far away we are from the setpoint, use different PID constants and different heater limits
/*
void SetPIDTuningFromSetpointGap()
{

  heaterPID.SetTunings(currentPIDKp, currentPIDKi, currentPIDKd);

  // Distance away from setpoint
  double setpointGap = abs(glassSetpoint-glassTemperature);

  if (setpointGap < 2.0)
  {
    // Close to setpoint, use conservative tuning parameters
    currentPIDKp = conservativePIDKp;
    currentPIDKi = conservativePIDKi;
    currentPIDKd = conservativePIDKd;
    heaterPID.SetTunings(currentPIDKp, currentPIDKi, currentPIDKd);
    //Serial.print("Conservative constants being used, upper heater limit: ");

    // Use conservative upper heater limit
    currentUpperHeaterLimit = conservativeUpperHeaterLimit;
    //Serial.println(currentUpperHeaterLimit);
  } 

  else  
  {
    // Far from setpoint, use aggressive tuning parameters
    currentPIDKp = aggressivePIDKp;
    currentPIDKi = aggressivePIDKi;
    currentPIDKd = aggressivePIDKd;
    heaterPID.SetTunings(currentPIDKp, currentPIDKi, currentPIDKd);


    //Serial.print("Aggressive constants being used, upper heater limit: ");

    // Use aggressive upper heater limit
    currentUpperHeaterLimit = aggressiveUpperHeaterLimit;
    //Serial.println(currentUpperHeaterLimit);
  }
  
}
*/
/*
//DP I am not sure this is necessary. The PID library should take care of this when we set the max output
// Ensure that PWMoutput is within desired heater limits, change it if not
void AdjustPWMoutputWhenOutsideLimits()
{
  // Change heater temperature to current upper limit if the PID suggested PWMoutput exceeds it
  if (PWMoutput > currentUpperHeaterLimit)
  {
    PWMoutput = currentUpperHeaterLimit;
  }
}
*/

// Print the buck converter voltage to the serial, pass in the buck converter voltage
void printParametersToSerial()
{
  Serial.print("V(in):");    
  Serial.print(buckConverterVoltage,2);
  Serial.print("\tT(glass):");
  Serial.print(glassTemperature,2);
  Serial.print("\tPWMOut:");
  Serial.print(PWMoutput);
  Serial.print("\tPIDKp:");
  Serial.print(PIDKp);
  Serial.print("\tPIDKi:");
  Serial.print(PIDKi);
  Serial.print("\tPIDKd:");
  Serial.print(PIDKd);
  Serial.print("\tPIDmode:");
  Serial.print(PIDmode);
  Serial.print("\tsetPt(glass):");
  Serial.println(glassSetpoint);
 
}


void appendToGlassHistory(int value)
{
  glassTempHistory[historyIndex] = value;
  historyIndex++;
  if (historyIndex >= historySize) {
    historyIndex = 0;
    historyFilled = true;
  }
}

double getGlassHistory(int index)
{
  double out = glassTempHistory[index];
  return out;
}


/*
void PrintBuckConverterVoltageToSerial(double buckConverterVoltage)
{
  Serial.print("Buck converter voltage: ");    
  Serial.print(buckConverterVoltage);
  Serial.println(" V");
}

// Print the glass temperature to the serial
void PrintGlassTemperatureToSerial()
{
  Serial.print("Current glass temperature: ");
  Serial.print(glassTemperature);
  Serial.println(" C");
}

// Print the PWMoutput to the serial
void PrintPWMoutputToSerial()
{
  Serial.print("PWMoutput: ");    
  Serial.println(PWMoutput);    
}
*/
