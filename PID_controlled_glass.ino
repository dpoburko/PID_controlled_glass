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
#include "functions.h"
#include "errorCheck.h"
#include "parseSerial.h"

// *************************************************************************************************************************************
// PINS definitions are in structures.h
// *************************************************************************************************************************************
  
/* General approach to temperature control:
 *  1. temperature readings are stored in generalSensor structures - named as objectTemperature
 *  2. thermistor hardware and cummulitive deviations values are stored in thermistor structures - named as objectThermistor
 *  3. calculation of thermistor temperatures with the STEINHART library is assigned to objects named steinhart#
 */

// *************************************************************************************************************************************
// PID CONTROLLER VARIABLES
// *************************************************************************************************************************************
//generalSensor thisSensor(int arraySize, "name", value, setpoint, slopeInterval, slopeUnits(), upperLimit, lowerLimit) : 
  generalSensor lidTemperature(35, "lid temperature",22.0,48.0,10,1000,65.0,20.0);
  generalSensor enclosureTemperature(35, "enclosure temperature", 22.0, 37.0, 20, 1000, 40.0, 20.0);

//PIDextra(double aP, double aI, double aD, double aSetpoint, double amaxOutputNormal,double amaxOutputHigh, double errorOutput int aMode)
  PIDextras heaterValues(2.0, 96.0, 21.0, lidTemperature.setpoint, 40.0,45.0,10.0,1);

String errorBuffer;
String msgBuffer;
String logBuffer;

long startUpTime = millis();

//thermistor type should mostly refer to the hardware aspects of a thermistor rather than it's measurements.
thermistor lidThermistor("lid thermistor", THERMISTOR1PIN);
thermistor enclosureThermistor("enclosure thermistor",THERMISTOR2PIN);

// *************************************************************************************************************************************
// VOLTAGE DIVIDER VARIABLES
// *************************************************************************************************************************************
// Coefficient for resistors 1 and 2 used for 1/10 voltage divider to detect input voltage
const double resistor1Coefficient = 9810;
const double resistor2Coefficient = 983;

// Set how often glass voltage will be read (in milliseconds)
int glassVoltageReadingInterval = 2000;

// *************************************************************************************************************************************
// SERIAL VARIABLES
// *************************************************************************************************************************************

//Initialize serialMsg
serialMsg serialMain(23); //hold buffer and variables related to serial monitor messages

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

// Variable to store buck converter voltage for input to lid heater
double buckConverterVoltage = 0;

// Data logging
uint16_t nLogged = 0;

// *************************************************************************************************************************************
// CONSTRUCTORS
// *************************************************************************************************************************************

// Create thermistor object, including relevant information about the thermistor and the required constants for the Steinhart equation
STEINHART steinhardt1(lidThermistor.pin, &lidTemperature.value, lidThermistor.rNominal, lidThermistor.tNominal, lidThermistor.bCoefficient, lidThermistor.rSeries);
STEINHART steinhardt2(enclosureThermistor.pin, &enclosureTemperature.value, enclosureThermistor.rNominal, enclosureThermistor.tNominal, enclosureThermistor.bCoefficient, enclosureThermistor.rSeries);

// Create PID controller for the glass, including relevant information for the PID such as input, output, setpoint, and constants
PID heaterPID(&lidTemperature.value, &heaterValues.outputFromPID, &lidTemperature.setpoint, heaterValues.P, heaterValues.I, heaterValues.D, DIRECT);

//instantiate the errorCheck library with references to needed variables. Note that errorCodes is a global variable, so does not need to be transfered.
errorCheck errorCheck(msgBuffer,errorBuffer, lidTemperature, heaterValues, startUpTime);

parseSerial parseSerial(serialMain, heaterPID, heaterValues, msgBuffer, lidTemperature, enclosureTemperature);

//optional looping through a range of PID values
timers pidTimer("pid cycler",600000); // (name, duration in ms)
int pidIndex = 0;
int nPidIndices = 0;
double Ps[] = {2.0,2.0,2.0,2.0,2.0,2.0,1.5,1.5,1.5,1.5,1.5,1.5};
double Is[] = {96.0,96.0,96.0,96.0,96.0,96.0,96.0,96.0,96.0,96.0,96.0,96.0};
double Ds[] = {21.0,10.0,50.0,100.0,150.0,200.0,21.0,10.0,50.0,100.0,150.0,200.0};

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
 
  // Initialize PID (automaticPIDMode mode)
  heaterPID.SetMode(AUTOMATIC); //note that AUTOMATIC is inherited as #define from the PID library.

  // Set up PID output limits  
  heaterPID.SetOutputLimits(0, heaterValues.maxOutputHigh); 
  heaterPID.SetSampleTime(glassVoltageReadingInterval); 

  // Set frequency of voltage readings for thermistor 1
  steinhardt1.setSampleTime(glassVoltageReadingInterval);

  // Read the temperature from thermistor 1
  steinhardt1.read();

  // Repeat for thermistor 2
  steinhardt2.setSampleTime(glassVoltageReadingInterval);
  enclosureTemperature.prevValue = enclosureTemperature.value;

  // Print initialization message and temperature to serial
  Serial.print("   ===== PID controlled glass heater =====================");
  Serial.println("   Stage-Top Incubator component"); 
  Serial.print("   Initial Temperature:");
  Serial.println(lidTemperature.value, 2);

   // Variable to store when the last glass setpoint update happened
  lidTemperature.setpointInterval = 30000;
  lidTemperature.setpointLastUpdate = 0;

  heaterValues.maxInput = buckConverterVoltage;

  startUpTime = millis();

  pidTimer.active = true;
  pidTimer.start = millis();
  
}

// *************************************************************************************************************************************
// MAIN LOOP
// *************************************************************************************************************************************

void loop() 
{

  // Call STEINHART library, check if value is updated  
  if (steinhardt1.read()==true) // this should both take a reading and service as the logical test.
  {
    // Read temperature from thermistor 2 - air temperature
    steinhardt2.read();

    // Get the voltage from the tunable buck converter (what do we do with this?)
    GetBuckConverterVoltage(heaterValues.maxInput,resistor1Coefficient ,resistor2Coefficient, boardVoltageOut); //by storing the current voltage in this structure, it become accessible to errorCheck

    // Check to make sure the temperature data points makes sense, and reassign the value if necessary
    RemoveErroneousSensorReadings(lidTemperature,0.25,msgBuffer);
    RemoveErroneousSensorReadings(enclosureTemperature,0.25,msgBuffer);

    // Log the temperatures and calculate slopes and MSE
    sensorUpdate(lidTemperature);
    sensorUpdate(enclosureTemperature);
    
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
    // Define output of PID as the pulse-width modulated output (will be at 0 the first time around as the PID needs two data points for computation)
    if (heaterPID.Compute() == true) 
    {
    	heaterValues.outputToDevice = heaterValues.outputFromPID;
    	// handoff the PID's intended output to the heater structure
    	// Perform a series of tests on thermistor reading and temperature vs. setpoint and over time
  
      unsigned long thisMillis = millis();
      if(pidTimer.active && (thisMillis - pidTimer.start) > pidTimer.duration) {
          heaterValues.P = Ps[pidIndex];
          heaterValues.I = Is[pidIndex];
          heaterValues.D = Ds[pidIndex];
          heaterPID.SetTunings(Ps[pidIndex],Is[pidIndex],Ds[pidIndex]);
          pidIndex++;
          if (pidIndex>11) pidIndex=0;
          pidTimer.start = thisMillis;
      } 

     //error check will likely neeed a bunch of structures passed to it: buffers, lidTemperature, enclosureTemperature, CO2 eventually, heaterValues
     errorCheck.update();
    } 
  }

   //%%% @Izzy, I have just noticed that we have no errorChecking when in manual mode. This seems prone to troubles. Let's rethink this.
  // Check if the air temperature is rising or fall

  if ( enclosureTemperature.slope[enclosureTemperature.index-1] > 0.2  )
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
    CheckGlassSetpoint(enclosureTemperature, lidTemperature, lidThermistor,msgBuffer);

	
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
    PrintParametersToSerial(enclosureTemperature, lidTemperature, heaterValues, msgBuffer, errorBuffer, logBuffer, nLogged);

    // Log the current time in milliseconds as the most recent time in which the parameters were printed to the serial
    lastSerialDisplay = currentTimeMilliseconds;
  }

  // If the user has typed a message to the serial monitor, read the message
  if (Serial.available() > 0) 
  {
    parseSerial.parse();
  }

  //Glass and air temp are logged in history array, so most references to historical values will use sensor.history[] 
  lidTemperature.prevValue = lidTemperature.value;
  enclosureTemperature.prevValue = enclosureTemperature.value;

  // Delay determines how often loop repeats
  delay(20);   //need to get rid of this!!!
}
