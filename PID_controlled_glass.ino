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
// PINS
// *************************************************************************************************************************************

// ~~~~~ moved to structures.h for global scoping ~~~~~
// // Analog input for measuring the voltage delivered to the glass heater via a 1/10 voltage divider
// #define VOLTMETERPIN A2
// // PWM PID output pin to MOSFET (adjust as needed)
// #define PWMPINOUT 9
// // Thermistor 1 pin
// #define THERMISTOR1PIN A0
// // Thermistor 2 pin
// #define THERMISTOR2PIN A4
  
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

// #define AUTOMATIC  1 & #define MANUAL  0 are defined in the PID library, meaning that they are visible to the sketch. No need to replace them. 

String errorBuffer;
String msgBuffer;
String logBuffer;

long startUpTime = millis();

// *************************************************************************************************************************************
// STEINHART EQUATION VARIABLES - now all held in a structure for each thermistor. This allows for simple expansion to have additional thermistors
// *************************************************************************************************************************************

//thermistor type should mostly refer to the hardware aspects of a thermistor rather than it's measurements.
thermistor lidThermistor("lid thermistor", THERMISTOR1PIN);
thermistor enclosureThermistor("enclosure thermistor",THERMISTOR2PIN);

// *************************************************************************************************************************************
// VOLTAGE DIVIDER VARIABLES
// *************************************************************************************************************************************

// Coefficient for resistors 1 and 2 used for 1/10 voltage divider to detect input voltage
const double resistor1Coefficient = 9810;
const double resistor2Coefficient = 983;

// *************************************************************************************************************************************
// PULSE-WIDTH MODULATION VARIABLES
// *************************************************************************************************************************************

// *************************************************************************************************************************************
// GLASS/THERMISTOR VARIABLES
// *************************************************************************************************************************************

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

// Set number of samples to take in order to get an average of the voltage data (needs to be a double for average to have decimals)
//double numberOfSampleVoltageReadings = 11.0;

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

//!!!!!! About controlling the glass temp. In principle, there's no reason that we couldn't control the enclosure temp, where the output is the glass temp if we use
// a sufficiently slow update cycle (every 60 - 90 seconds?) and share glass max time with the enclosure PID.
// looks pretty easy! Just need time to tune the PID values. 
//PID enclosurePID(&enclosureTemperature.value, &lidTemperature.setpoint,&enclosureTemperature.setpoint, enclosureValues.P, enclosureValues.I, enclosureValues.D, DIRECT);

//instantiate the errorCheck library with references to needed variables. Note that errorCodes is a global variable, so does not need to be transfered.
//250224 - I am not convinced that errorCheck and parseSerial instantiation don't need to be references
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
    //GetBuckConverterVoltage(buckConverterVoltage); 
    GetBuckConverterVoltage(heaterValues.maxInput,resistor1Coefficient ,resistor2Coefficient, boardVoltageOut); //by storing the current voltage in this structure, it become accessible to errorCheck

    // Check to make sure the glass temperature data point makes sense, and reassign the value if necessary
    //250123 - DP revised this function to work for any generalSensor structure

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
    // Make sure it is time to recalculate
    //heaterValues->newValue = heaterPID.Compute();

    // Define output of PID as the pulse-width modulated output (will be at 0 the first time around as the PID needs two data points for computation)
    //if (heaterValues->newValue == true) 
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
  //if (enclosureTemperature.setpointReached)
  //{
    //CheckGlassSetpoint();
    CheckGlassSetpoint(enclosureTemperature, lidTemperature, lidThermistor,msgBuffer);
  //}
	
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

// *************************************************************************************************************************************
// FUNCTIONS
// *************************************************************************************************************************************

// Read serial message, consider moving this to a library

// // Get the average voltage from the tunable buck converter, return tunable buck converter voltage
// void GetBuckConverterVoltage(double &newVoltage)
// {
//   // Read voltage from voltmeter 
//   double thisVoltage = 0.0;
	
//   // Average GetBuck from heater voltage source with a slight delay
//   for (int i = 0; i < numberOfSampleVoltageReadings; i++) 
//   {
//     thisVoltage += analogRead(VOLTMETERPIN)/ numberOfSampleVoltageReadings;
//     delay(1);
//   }
 
//   // Convert the 0-1024 analog input to a voltage
//   thisVoltage = (thisVoltage * boardVoltageOut) / 1023.0;   
//   //Correct for the voltage-divide upstream of the measured voltage
//   thisVoltage = thisVoltage / (resistor2Coefficient / (resistor1Coefficient + resistor2Coefficient));

//   // If voltage is very small, consider it negligible and set it to 0        
//   if (thisVoltage < 0.1)  
//   {  
//     thisVoltage = 0.0;   
//   }
//   newVoltage = thisVoltage; 
// }

// // Print all relevant values to serial
// // Eventually this will need to merge with logToSD.cpp in IncuStage
// void PrintParametersToSerial()
// {

//   if (nLogged == 0) {
//     Serial.println(" ");
//     logBuffer += "\tT(enclosure):\tT(lid):\tT(encl.slope):\tT(lidTemperature.slope):\tT(mse):\tsetPt(lid):\tsetPt(enclosure):\tPWMOut:\tV(in):\tPID:\tError:\tMsg:";
//     Serial.println(logBuffer);
//     logBuffer = "";
//   }

//   //Note that sensor.Index is advanced at the end of updateSensor, so the current index is 1 ahead of the most recently stored values
//   int enclosureIndexA = (enclosureTemperature.index -1  + enclosureTemperature.historySize) % enclosureTemperature.historySize;
//   int enclosureIndexB = (enclosureTemperature.index -1 - enclosureTemperature.slopeInterval + enclosureTemperature.historySize) % enclosureTemperature.historySize;
//   int lidIndexA = (lidTemperature.index  -1 + lidTemperature.historySize) % lidTemperature.historySize;
//   int lidIndexB = (lidTemperature.index  -1 - lidTemperature.slopeInterval + lidTemperature.historySize) % lidTemperature.historySize;

//   logBuffer = "";
//   logBuffer += "\t";
//   logBuffer += String(enclosureTemperature.value, 2);
//   logBuffer += "\t";
//   logBuffer += String(lidTemperature.value, 2);
//   logBuffer += "\t";
//   logBuffer += String(enclosureTemperature.slope[enclosureIndexA], 3);
//   logBuffer += "\t";
//   logBuffer += String(lidTemperature.slope[lidIndexA], 3);
//   logBuffer += "\t";
//   logBuffer += String(lidTemperature.meanSquareError, 3);
//   logBuffer += "\t";
//   logBuffer += String(lidTemperature.setpoint, 2);
//   logBuffer += "\t";
//   logBuffer += String(enclosureTemperature.setpoint, 1);
//   logBuffer += "\t";
//   logBuffer += String(heaterValues.outputToDevice);
//   logBuffer += "\t";
//   //logBuffer += String(buckConverterVoltage, 2);
//   logBuffer += String(heaterValues.maxInput,2);
//   logBuffer += "\t";
//   logBuffer += String(heaterValues.P);
//   logBuffer += "/";
//   logBuffer += String(heaterValues.I);
//   logBuffer += "/";
//   logBuffer += String(heaterValues.D);
//   logBuffer += "\t";
//   logBuffer += errorBuffer;
//   logBuffer += "\t";
//   logBuffer += msgBuffer;
//   logBuffer += "\n";
//   Serial.print(logBuffer);
//   logBuffer = "";
//   msgBuffer = " ";
//   errorBuffer = " ";
//   nLogged++;
// }

// void CheckGlassSetpoint()

// {
//   // Current time in milliseconds since the program has been running
//   int currentTimeMilliseconds = millis();

//   // Calculate gap from air temperature setpoint
//   enclosureTemperature.deviation = enclosureTemperature.value - enclosureTemperature.setpoint;

//   lidThermistor.autoSetpointChange = 0;

//   // If it has been long enough since the last glass setpoint update and the history array has enough data, check if the glass setpoint needs to be updated
//   if ((currentTimeMilliseconds - lidTemperature.setpointLastUpdate) >= lidTemperature.setpointInterval && lidTemperature.historyFilled == true)
//   {

//     // If the glass temperature has been stable and the air temperature is significantly higher than the air temperature setpoint, update the glass setpoint
//     {

//       //If air temp is too high, reduce glass temp, if too low, increase glass temp. This toggles direction of change.
//       float deviationDirection = 0.0;
//       if (enclosureTemperature.deviation>=0) deviationDirection = -1.0;
//       if (enclosureTemperature.deviation<0) deviationDirection = 1.0;

//       double deviations[] = {1.0,0.7, 0.4 ,0.2 ,0.15,0.1,0};
//       double corrections[] = {2,1.75, 1.5 ,1.25,0.50,0.25,0};
//       int nDeviations = sizeof(deviations) / sizeof(deviations[0]);
      
//       for (int d=0;d<nDeviations;d++) {
//         if( abs(enclosureTemperature.deviation) > deviations[d]){
//           lidThermistor.autoSetpointChange = deviationDirection * corrections[d];
//           d = nDeviations;
//         }
        
//       }

//       //increment the deviation of glassSetpoint from the most recent user-defined glassSetpoint
//       lidThermistor.cummSetpointChange += lidThermistor.autoSetpointChange;
      
//       // if the delta is positive, limit the total possible increase to 5C. The user can reset this by manually increasing the set point.
//       //The should limit overshoot in the automatic increases in glass set point
//       if (lidThermistor.autoSetpointChange>0 && lidThermistor.cummSetpointChange >= lidThermistor.maxAutoIncrease) {
//         lidThermistor.cummSetpointChange = lidThermistor.maxAutoIncrease;
//         lidThermistor.autoSetpointChange = 0;
//       }

//       lidTemperature.setpoint += lidThermistor.autoSetpointChange;

//       // Update the last glass setpoint update time to the current time
//       lidTemperature.setpointLastUpdate = currentTimeMilliseconds;

//       if (lidThermistor.autoSetpointChange != 0.0) {
//         // Send a message to the serial
//         msgBuffer += "Glass Setpoint updated to ";
//         msgBuffer += String(lidTemperature.setpoint);
//       }
//     }
//   }
// }


// //Note, with change to generalSensor for temperatures and use of sensorUpdate():
// // 1. the erroneous check needs to come before sensorUpdate and the values are logged - Done
// // 2. the erroneous check should work on a single general sensor, i.e. lidTemperature or enclosureTemperature
// // 3. it should include a double Tolerance to identify erroneous readings
// //%%%%% Need to add increased tolerance depending on current slope. When heating, it needs to be more generous.

// void RemoveErroneousSensorReadings(generalSensor &sensor, double tolerance)
// {
//   // Calculate difference between current glass temperature and previous glass temperature reading
//   //Since glassTemperatureHistory is a circular buffer, we access the values 1 and 2 indices before the current historyArraysIndex
//   if ( (sensor.historyFilled == true) && (sensor.history[(sensor.index - 1 + sensor.historySize)%sensor.historySize] != sensor.history[(sensor.index - 2 + sensor.historySize)%sensor.historySize]) )
//   {
//     double prev1Value = sensor.history[(sensor.index - 1 + sensor.historySize)%sensor.historySize]; 
//     double prev2Value = sensor.history[(sensor.index - 2 + sensor.historySize)%sensor.historySize]; 
//     if ( (abs(sensor.value - prev1Value) > tolerance) && (abs(sensor.value - prev2Value) > tolerance) ) 
//     {
//       // Send a message to the serial
//       msgBuffer += "Erroneous ";
//       msgBuffer += sensor.name;
//       msgBuffer += " reading of ";
//       msgBuffer += sensor.value;
//       msgBuffer += " vs ";
//       msgBuffer += String(prev1Value,2);
//       msgBuffer += " & ";
//       msgBuffer += String(prev2Value,2);
//       msgBuffer += ". Reassigned to ";
//       sensor.value = (prev1Value + prev2Value)/2;
//       sensor.history[(sensor.index - 1 + sensor.historySize)%sensor.historySize] = sensor.value; 
//       msgBuffer += sensor.value;
      
//     }
//   }
 
// }

// void sensorUpdate(generalSensor& sensor) {

//   //add the value to the array and note the time that value is added to the history in millis
//   sensor.history[sensor.index] = sensor.value;
//   sensor.deviation = sensor.value - sensor.setpoint;
//   sensor.time[sensor.index] = millis();
//   if (sensor.value < sensor.minimum) sensor.minimum = sensor.value;
//   if (sensor.value > sensor.maximum) sensor.maximum = sensor.value;

//   //determine slope and append to slope history
  
//   if (sensor.historyFilled) {
    
//     //determine slope
//     sensor.slope[sensor.index] = (sensor.value - sensor.history[(sensor.index - sensor.slopeInterval + sensor.historySize) % sensor.historySize]) / 
//                                  ( (sensor.time[sensor.index] - sensor.time[(sensor.index - sensor.slopeInterval + sensor.historySize) % sensor.historySize]) / sensor.slopeUnits);
 
//     if (abs(sensor.slope[sensor.index])>10000) {
//       sensor.slope[sensor.index] = 0;
//     }

//     //calculate deviation squared for mse
//     sensor.errorHistory[sensor.index] = (sensor.deviation)*(sensor.deviation);

//     //calculate average and mean squared error over the history
//     sensor.average = 0;
//     sensor.meanSquareError = 0;
    
//     for (int i = 0; i < sensor.historySize; i++) {
      
//       sensor.average += sensor.history[i];
//       sensor.meanSquareError += sensor.errorHistory[i];
      
//     }
    
//     sensor.average /= sensor.historySize;
//     sensor.meanSquareError /= sensor.historySize;
//     sensor.meanSquareError = sqrt(sensor.meanSquareError);
	  
//   } else {
//     //if history is not yet filled ....
//       if (sensor.index<=1) {
//         sensor.slope[sensor.index] = (sensor.value - sensor.history[0]) / ( (sensor.time[sensor.index] - sensor.time[0]) / sensor.slopeUnits);
//       } else if (sensor.index < sensor.slopeInterval+2) {
//         sensor.slope[sensor.index] = (sensor.value - sensor.history[1]) / ( (sensor.time[sensor.index] - sensor.time[1]) / sensor.slopeUnits);
//       } else {
// 	      sensor.slope[sensor.index] = (sensor.value - sensor.history[(sensor.index - sensor.slopeInterval + sensor.historySize) % sensor.historySize]) / 
//                                  ( (sensor.time[sensor.index] - sensor.time[(sensor.index - sensor.slopeInterval + sensor.historySize) % sensor.historySize]) / sensor.slopeUnits);
//       }   
      
//       sensor.meanSquareError = 0;
//       sensor.average = 0;
      
//       if (sensor.index==0 ) {
//         sensor.errorHistory[sensor.index] = 0;
//       } else {
//         sensor.errorHistory[sensor.index] = (sensor.value-sensor.setpoint)*(sensor.value-sensor.setpoint);
//       }
      
//       for (int i = 0; i < sensor.index; i++) {
//         sensor.average += sensor.history[i];
//         sensor.meanSquareError += sensor.errorHistory[i];
//       }
//       sensor.average /= (sensor.index+1);
//       sensor.meanSquareError /= (sensor.index+1);
//       sensor.meanSquareError = sqrt(sensor.meanSquareError);
//   }

//   sensor.index++;

//   //this is essentially used like a circular buffer
//   if (sensor.index == sensor.historySize) {
//     sensor.historyFilled = true;
//     sensor.index = 0;  // reset
//   }
// }
