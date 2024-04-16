// For wiring details of voltmeter see:
// https://www.digikey.ca/en/maker/projects/how-to-make-a-simple-digital-voltmeter-with-an-arduino/082dff9d725549aea8bf84a7e302c1b2
// For thermistor details see:
// https://learn.adafruit.com/thermistor/using-a-thermistor

// PID control library (must be downloaded)
#include <PID_v1.h>
// Wire library for I2C communication (must be downloaded)
#include <Wire.h>

// Thermistor pin
#define THERMISTORPIN A0
// Voltmeter pin
#define VOLTMETERPIN A2
// PWM PID output PIN to MOSFET
#define pwmPinOut 9 //adjust as needed
// Thermistor resistance at 25 C 
#define THERMISTORNOMINAL 10000
// Temperature for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// B coefficient for Steinhart equation (is dependent on the probe black Vishay from Digikey - NTCLE413E2103F102L = 3435)
#define BSTEINHARTCOEFFICIENT 3435
// Value of the other resistor (230501 - measured with AstroAI DM6000AR multimeter - difference between 5 C (with series resistor) 
//and A0 line at 21 C and 0 C - same value)
#define SERIESRESISTOR 9985

// Set glass temperature goal (in degrees Celsius)
const float glassSetpoint = 35.0;
float PIDStartDelta = 30.0;

// Set heater limits (bits), these are arbitrary values for now
const float outPutMax = 25.0; 
const float aggressiveUpperHeaterLimit = outPutMax;
const float conservativeUpperHeaterLimit = outPutMax;

// Variable to store current PWMoutput limit
float currentUpperHeaterLimit = outPutMax;

// Variable to store maximum allowed glass temperature (in degrees Celsius)
float maxGlassTemperature = 60.0;

// Set PID constants (aggressive and conservative), these are arbitrary values for now
const float aggressivePIDKp = 25, aggressivePIDKi = 10, aggressivePIDKd = 0;
const float conservativePIDKp = 25, conservativePIDKi = 10, conservativePIDKd = 0;
float currentPIDKp = conservativePIDKp;
float currentPIDKi = conservativePIDKi;
float currentPIDKd = conservativePIDKd;

bool PIDmode = 0;

// R coefficients
const float r1Coefficient = 9810.0;
const float r2Coefficient = 983.0;

float buckConverterVoltage =0;

// Set number of samples to take in order to get an average of the voltage and temperature data (more samples takes longer but 
// is more smooth)
const int numberOfSampleReadings = 15;

// Initialize glass temperature (in degrees Celsius) and PWMoutput value needed to change glass temperature
float glassTemperature = 0.0;
float PWMoutput = 0.0;

// Define PID variables (from PID library)
double PIDSetpoint, PIDInput, PIDOutput;

// Create a counter for the amount of reads taken
int readCounter = 0;

// Arrays to log tunable buck converter voltage data and thermistor voltage data
uint16_t thermistorVoltagesArray[numberOfSampleReadings];
uint16_t buckConverterVoltagesArray[numberOfSampleReadings];

// Create glass temperature array (of size 50 for now)
double glassTemperaturesArray[50];

// Create PID object (start with conservative tuning constants to be safe)

PID heaterPID(&PIDInput, &PIDOutput, &PIDSetpoint, currentPIDKp, currentPIDKi, currentPIDKd, DIRECT);

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

  // Start at conservative upper heater limit to be safe
  currentUpperHeaterLimit = conservativeUpperHeaterLimit;
}

void loop() 
{
  // Get the voltage from the tunable buck converter
  buckConverterVoltage = GetBuckConverterVoltage();

  // Get the voltage from the thermistor
  float thermistorVoltage = GetThermistorVoltage();

  // Calculate the glass temperature using the Steinhart equation
  glassTemperature = GetTemperatureSteinhartEquation(thermistorVoltage);

  //When initially warming, if glass is too cold, go full throttle
  if (0 > glassTemperature || glassTemperature < (glassSetpoint-PIDStartDelta)) {
      heaterPID.SetMode(MANUAL);
      PWMoutput = outPutMax;
      PIDmode = 0;
  //Otherwise, check if it needs to be set to Auto and compute
  } else {
    // Compute PID loop (this is a function from the PID library)
    if (heaterPID.GetMode()==0) {
      heaterPID.SetMode(AUTOMATIC); 
      PIDmode =1;
      PWMoutput = 0;     
    } 
    
    heaterPID.Compute();
    // Define output of PID as the PWMoutput (will be at 0 the first time around as the PID needs two data points 
    // for computation)
    PWMoutput = PIDOutput;    
  }
 

  // Log the temperature in an array and increase the read count
  glassTemperaturesArray[readCounter] = glassTemperature;
  readCounter ++;
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
    if (HasGlassTemperatureDropped())
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
  // Define input of the PID as the glass temperature
  PIDInput = glassTemperature;

  // Decide whether to use aggressive or conservative tuning constants based on the distance from the setpoint before computing PID
  SetPIDTuningFromSetpointGap();


  // Adjust PWMoutput if it is outside of the desired limits
  AdjustPWMoutputWhenOutsideLimits();

  // Adjust the duty cycle of the PWM pin connected to the MOSFET fron the PID output
  analogWrite(pwmPinOut, PWMoutput);


  // Print buck converter voltage, glass temperature, and PWMoutput to serial
  //PrintBuckConverterVoltageToSerial(buckConverterVoltage);
  //PrintGlassTemperatureToSerial();
  //PrintPWMoutputToSerial();
  printParametersToSerial();

  // Delay determines how often loop repeats
  delay(2000); 
}

// Get the average voltage from the tunable buck converter, return tunable buck converter voltage
float GetBuckConverterVoltage()
{
  // Read voltage from voltmeter
  float buckConverterVoltage = analogRead(VOLTMETERPIN);

	//Damon's note - rather than storing measures in an array,then reading from that array, it would make for more compact code to simeple =+ the value

  // Take a predetermined number of voltage readings from the tunable buck converter in a row, with a slight delay
  for (int i = 0; i < numberOfSampleReadings; i++) 
  {
    buckConverterVoltagesArray[i] = buckConverterVoltage;
    delay(5);
  }

  float sumOfVoltageSamples = 0;

  // Sum all the logged buck converter data
  for (int i = 0; i < numberOfSampleReadings; i++) 
  {
    sumOfVoltageSamples += buckConverterVoltagesArray[i];
  }

  // Take the average of the buck converter voltages by dividing by the number of samples
  float averageBuckConverterVoltage = 0;
  averageBuckConverterVoltage = sumOfVoltageSamples / numberOfSampleReadings;

  // Do something?
  float buckConverterTemporaryVariable = (averageBuckConverterVoltage * 5.0) / 1024.0;   
  buckConverterVoltage = buckConverterTemporaryVariable / (r2Coefficient / (r1Coefficient + r2Coefficient));

  // If voltage is very small, consider it negligible and set it to 0        
  if (buckConverterVoltage < 0.1)
  {     
    buckConverterVoltage = 0.0;    
  }

  return buckConverterVoltage;
}

// Get the average voltage from the thermistor, return the average thermistor voltage
float GetThermistorVoltage()
{
  // Take a predetermined number of thermistor voltage samples in a row, with a slight delay
  for (int i = 0; i < numberOfSampleReadings; i++) 
  {
    thermistorVoltagesArray[i] = analogRead(THERMISTORPIN);
    delay(5);
  }

  float sumOfThermistorVoltageSamples = 0;

  // Sum all the logged thermistor voltage data
  for (int i = 0; i < numberOfSampleReadings; i++) 
  {
    sumOfThermistorVoltageSamples += thermistorVoltagesArray[i];
  }

  // Take the average of the thermistor voltages by dividing by the number of samples
  float averageThermistorVoltage = 0;
  averageThermistorVoltage = sumOfThermistorVoltageSamples / numberOfSampleReadings;

  // Do something?
  averageThermistorVoltage = 1023 / averageThermistorVoltage - 1;
  
  return averageThermistorVoltage;
}

// Calculate glass temperature using the Steinhart equation, pass in thermistor voltage, return glass temperature
float GetTemperatureSteinhartEquation(float thermistorVoltage)
{
  // Convert the thermistor voltage to resistance
  float resistance = SERIESRESISTOR / thermistorVoltage;
  // R / Ro
  float steinhartTemporaryVariable = resistance / THERMISTORNOMINAL;
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

bool IsGlassTemperatureSafe()
{
  bool isGlassTemperatureSafe = false;

  // Make sure glass temperature is equal to or below the maximum allowed temperature
  if (glassTemperature <= maxGlassTemperature)
  {
    isGlassTemperatureSafe = true;
  }

  // Return safety status
  return isGlassTemperatureSafe;
}

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
bool HasGlassTemperatureDropped()
{
  bool hasGlassTemperatureDropped = true;

  //float glassTemperatureGap = glassTemperaturesArray[readCounter] - glassTemperaturesArray[readCounter - 1];

  if (glassTemperaturesArray[readCounter - 1] < 0)
  {
    hasGlassTemperatureDropped = false;
  }

  return hasGlassTemperatureDropped;
}

// Check for a significant glass temperature increase
bool HasGlassTemperatureIncreased()
{
  bool hasGlassTemperatureIncreased = true;

  float glassTemperatureGap = glassTemperaturesArray[readCounter] - glassTemperaturesArray[readCounter - 1];

  if (glassTemperatureGap < 5)
  {
    hasGlassTemperatureIncreased = false;
  }

  return hasGlassTemperatureIncreased;
}

// Based on how far away we are from the setpoint, use different PID constants and different heater limits
void SetPIDTuningFromSetpointGap()
{

  heaterPID.SetTunings(currentPIDKp, currentPIDKi, currentPIDKd);
  /*
  // Distance away from setpoint
  float setpointGap = abs(glassSetpoint-glassTemperature);

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
  */
}

// Ensure that PWMoutput is within desired heater limits, change it if not
void AdjustPWMoutputWhenOutsideLimits()
{
  // Change heater temperature to current upper limit if the PID suggested PWMoutput exceeds it
  if (PWMoutput > currentUpperHeaterLimit)
  {
    PWMoutput = currentUpperHeaterLimit;
  }
}


//Damon's note - Izzy, here you have the data printing to separate lines of the serial monitor. For easy of handling of graphing software, I present to 
//print all values of interest to a single line with tabs beteen. 

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
  Serial.print(currentPIDKp);
  Serial.print("\tPIDKi:");
  Serial.print(currentPIDKi);
  Serial.print("\tPIDKd:");
  Serial.print(currentPIDKd);
  Serial.print("\tPIDmode:");
  Serial.print(PIDmode);
  Serial.print("\tsetPt(glass):");
  Serial.println(glassSetpoint);
 
}
/*
void PrintBuckConverterVoltageToSerial(float buckConverterVoltage)
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
