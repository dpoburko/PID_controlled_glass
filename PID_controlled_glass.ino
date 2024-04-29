// For wiring details of voltmeter see:
// https://www.digikey.ca/en/maker/projects/how-to-make-a-simple-digital-voltmeter-with-an-arduino/082dff9d725549aea8bf84a7e302c1b2
// For thermistor details see:
// https://learn.adafruit.com/thermistor/using-a-thermistor

// PID control library (must be downloaded)
#include <PID_v1.h>
// Wire library for I2C communication (must be downloaded)
#include <Wire.h>
#include "STEINHART.h"

// Analog input for measuring the voltage delivered to the glass heater via a 1/10 voltage divider
#define VOLTMETERPIN A2
// PWM PID output PIN to MOSFET
#define pwmPinOut 9 //adjust as needed

//indicate the voltage of the dev board digital outputs.
const double boardVout = 5.0;

// Set glass temperature goal (in degrees Celsius) and max temp allowed
const double glassSetpoint = 35.0;
const double maxGlassTemperature = 60.0;
double PIDStartDelta = 6.0;

// ******* Set heater limits (bits), these are arbitrary values for now ***************************************
const double outPutMax = 25.0; // Currently set to limit total current to the limits of the PSU or Buck converter. 

// Set number of samples to take in order to get an average of the voltage and temperature data 
int nSampleReadings = 5;


//===== PID controller variables ===============================================================================
// Set PID constants (aggressive and conservative), these are arbitrary values for now
const double PIDKp = 25, PIDKi = 10, PIDKd = 0;

// Define PID variables (from PID library)
double PIDSetpoint, PIDInput, PIDOutput;
double PWMoutput = 0.0;
double PWMoutputLast = 0.0;
bool PIDmode = 0; 
bool safeMode = false; //catch all for errors
bool newPID = false; //check if output has changed since last loop

// Create PID object (start with conservative tuning constants to be safe)
PID heaterPID(&PIDInput, &PIDOutput, &PIDSetpoint, PIDKp, PIDKi, PIDKd, DIRECT);

//===== Resistors r1 and r2 used for ~1:10 voltage divider to detection input voltage ============================
const double r1Coefficient = 9810.0;
const double r2Coefficient = 983.0;
double buckConverterVoltage = 0;

//===== Variables for the thermistor attached to the glass lid ==================================================================
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

//===== Initialize the Steinhart temp calculation. Only glass temp needs to be a reference ===========================
STEINHART thermistor1(thermistorPin, &glassTemperature, Rnominal, Tnominal, bCoeff, Rseries);
//=================================================================================

//====== Timing and communications ===============================================================================
int displayInterval = 2000;
int displayLast = 0;

//Variables for serial message
const int serialMML = 23; //max message length
char endMessage = '.'; // Character which signals end of message from PC
char incomingSerial[serialMML];
int lengthOfMessage = 0;


void setup()
{
  // Open serial port, set data rate to 9600 bps
  Serial.begin(9600);

  // Delay helps fix serial glitch
  delay(1000);

  Serial.print("===== PID controlled glass heater =====================");
  Serial.println("Stage-Top Incubator component"); 
  
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

}

void loop() 
{
 
  //call STEINHART library, check if value is updated  
  bool newTemp = thermistor1.read();
  
  //add the new temp reading to the history array if new value available
  if (newTemp) {
    
    // Get the voltage from the tunable buck converter - burried here to match temp check interval
    buckConverterVoltage = GetBuckConverterVoltage();

     appendToGlassHistory(glassTemperature);
     // Define input of the PID as the glass temperature
     PIDInput = glassTemperature; //no reason these can't be hard linked. 
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

 
  //check the glass temp to intervene with PID controller output if needed 
  bool glassCheck = checkGlassTemp();
	
  if (PWMoutput!= PWMoutputLast) {
    // Adjust the duty cycle of the PWM pin connected to the MOSFET fron the PID output if the value has changed
    analogWrite(pwmPinOut, PWMoutput);
    PWMoutputLast = PWMoutput;
  }

  // Print buck converter voltage, glass temperature, and PWMoutput to serial
  int tNow = millis();
  if ( (tNow - displayLast) >= displayInterval) {
    displayLast = tNow;
    printParametersToSerial();
    
  }

  if (Serial.available() > 0) {

  /*
   * What do we need to be able to change
   * h - help. list options
   * OK - acknowledge problem. Continue programs
   * 
   * P - PID
   *  m = mode
   *   1 = auto, 0 = manual, g = getMode
   *  o - output 
   *   8-bit value < maxoutput
   *  t 
   *   pppiiiddd - 9-digit value , 3 for each of p i and d
   *  s - update setpoint 
   *   nnn - 3-digit value representing tens, ones, decimal 00.0
   *  D - update PID/STEINHART update delay
   *   
   */

      // Read incomingMX300 until end character is reached or maxMessageLength characters are read.
    lengthOfMessage = Serial.readBytesUntil(endMessage,incomingSerial,serialMML);
    if (lengthOfMessage <= serialMML) {
      if (incomingSerial[0] == 'P') {
        //parsePIDCmd();  
      }
      if ((incomingSerial[0] == 'O') & (incomingSerial[0] == 'K')) {
          Serial.println("OK. Error acknoledgement received");
          safeMode = false;
          
      }
      if ( incomingSerial[0] == 'h' || incomingSerial[0] == 'H' || incomingSerial[0] == '?') {
        Serial.println("\n>>>> PID Serial Commands <<<<\n" );
        Serial.println("End commands with '.' for faster responses" );
        Serial.println("Interacting with the PID controller" );
        Serial.println("   Pm1 - call  heaterPID.SetMode(AUTOMATIC)" );
        Serial.println("   Pm0 - call  heaterPID.SetMode(Manual)" );
        Serial.println("   Pmg - call  heaterPID.GetMode()" );
        Serial.println("   Ponnn. - manually set PWMoutput (only usual in MANUAL mode" );
        Serial.println("   Plnnn. - call heaterPID.SetOutputLimits(0, nnn)" );
        Serial.println("   Ptpppiiiddd. - heaterPID.SetTunings(ppp,iii,ddd)" );
        Serial.println("   Dnnnn. - call heaterPID.SetSampleTime(nnnn) & STEINHART::setSampleTime(nnnn)" );
        
      }
    }

   }// if (Serial.available() > 0)

  // Delay determines how often loop repeats
  delay(20); 
}

// Get the average voltage from the tunable buck converter, return tunable buck converter voltage
double GetBuckConverterVoltage()
{
  // Read voltage from voltmeter 
  buckConverterVoltage = 0.0;
	
  // Average nSampleReadings from heater voltage source with a slight delay
  for (int i = 0; i < nSampleReadings; i++) 
  {
    buckConverterVoltage += analogRead(VOLTMETERPIN)/ nSampleReadings;
    delay(3);
  }
 
  // Convert the 0-1024 analog input to a voltage
  buckConverterVoltage = (buckConverterVoltage * boardVout) / 1024.0;   
  //Correct for the voltage-divide upstream of the measured voltage
  buckConverterVoltage = buckConverterVoltage / (r2Coefficient / (r1Coefficient + r2Coefficient));

  // If voltage is very small, consider it negligible and set it to 0        
  if (buckConverterVoltage < 0.1)  {  buckConverterVoltage = 0.0;   }

  return buckConverterVoltage;
}

bool checkGlassTemp()
{
  //check if temp falling, suggesting a heater failure/short 
  if (historyFilled) {
    //check if temp falling despite heat being applied
      
      double deltaOverTime = glassTemperature - glassTempHistory[ (historyIndex - historySize)%historySize];
      double deltaFromSetPt = glassTemperature - glassSetpoint;
      if (deltaOverTime>1.5) {
        if (deltaFromSetPt>2.0) {
	      	Serial.print("Error: Thermal Run away detected. Switching to safemode"); 
        	safeMode = true;
		      heaterPID.SetMode(MANUAL);
      		PWMoutput = outPutMax;
      		PIDmode = 0;
        	//create a user input to continue and exit safet mode
        	
        }
      }
      if (deltaOverTime<-1.5) {
        Serial.print("Glass temp falling"); 
        if (glassTemperature >= maxGlassTemperature) {
  	      Serial.print("Error: GLASS AT SAFETY LIMIT!! Throlling back pwmout"); 
  	      //gently throttle back the power output
  	      PWMoutput /= 2;
        }
      }
  }
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
