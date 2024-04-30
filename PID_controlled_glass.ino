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
double glassSetpoint = 35.0;
const double maxGlassTemperature = 60.0;
double PIDStartDelta = 6.0;

// ******* Set heater limits (bits), these are arbitrary values for now ***************************************
double outPutMax = 25.0; // Currently set to limit total current to the limits of the PSU or Buck converter. 

// Set number of samples to take in order to get an average of the voltage and temperature data 
int nSampleReadings = 5;

//===== PID controller variables ===============================================================================
// Set PID constants (aggressive and conservative), these are arbitrary values for now
double PIDKp = 25, PIDKi = 10, PIDKd = 0;

// Define PID variables (from PID library)
double PIDSetpoint, PIDInput, PIDOutput;
double PWMoutput = 0.0;
double PWMoutputLast = 0.0;
bool PIDmode = 0; 
int errorCode = 0; //catch all for errors
bool errorAcknowledged = false;
bool userOverRide = false;
bool newPID = false; //check if output has changed since last loop
// Create PID object (start with conservative tuning constants to be safe)


//===== Resistors r1 and r2 used for ~1:10 voltage divider to detection input voltage ============================
const double r1Coefficient = 9810.0;
const double r2Coefficient = 983.0;
double buckConverterVoltage = 0;

//===== Variables for the thermistor attached to the glass lid ==================================================================
int thermistorPin = A0; // // Thermistor pin
double glassTemperature = 0.0; // Initialize glass temperature (in degrees Celsius) and PWMoutput value needed to change glass temperature
int glassInterval = 2000;
const int historySize = 60;
int historyIndex =0;
bool historyFilled = false;
double glassTempHistory[historySize];// Create glass temperature array (of size 50 for now)
bool newTemp = false;

double thermistorVoltage = 0.0;
long Rnominal = 10000;  // Thermistor resistance at 25 C 
long Tnominal = 25; // Temperature for nominal resistance (almost always 25 C)
long bCoeff = 3435; // B coefficient for Steinhart equation 
long Rseries = 9985; // measured R for whatever seriers resistor (~10 kOhms) is used

//===== Initialize the Steinhart temp calculation. Only glass temp needs to be a reference ===========================
STEINHART thermistor1(thermistorPin, &glassTemperature, Rnominal, Tnominal, bCoeff, Rseries);
//=================================================================================

//===== Initialize the PID controller for the glass lid  ===========================
// test replacing PIDInput with glassTemperature
PID heaterPID(&PIDInput, &PIDOutput, &PIDSetpoint, PIDKp, PIDKi, PIDKd, DIRECT);
//==================================================================================

//====== Timing and communications ===============================================================================
int displayInterval = 2000;
int displayLast = 0;

//Variables for serial message
const int serialMML = 12; //max message length
char endMessage = '.'; // Character which signals end of message from PC
char incomingSerial[serialMML];
int lengthOfMessage = 0;


void setup()
{
  
  Serial.begin(9600); // Open serial port, set data rate to 9600 bps
  delay(1000);   // Delay helps fix serial glitch

  Wire.begin(); // Initialize communication with I2C devices

//%% - this feels redundant. Just need either PIDSetpoint or glassSetpoint
  // Initialize desired glass temperature
  PIDSetpoint = glassSetpoint;
 
  // Initialize PID
  heaterPID.SetMode(AUTOMATIC);
  PIDmode =1;     
  heaterPID.SetOutputLimits(0, outPutMax); //set to limits of 

  thermistor1.setSampleTime(glassInterval);
  thermistor1.read();

  Serial.print("===== PID controlled glass heater =====================");
  Serial.println("Stage-Top Incubator component"); 

}

void loop() 
{
 
  //call STEINHART library, check if value is updated  
  newTemp = thermistor1.read();
  
  //add the new temp reading to the history array if new value available
  if (newTemp) {
    
    // Get the voltage from the tunable buck converter - burried here to match temp check interval
    getBuckConverterVoltage(buckConverterVoltage);

    appendToGlassHistory(glassTemperature);
     // Define input of the PID as the glass temperature
     PIDInput = glassTemperature; //no reason these can't be hard linked. 
  }
  
  //if in automatic mode, compute output
  if (heaterPID.GetMode()==1) {
      newPID = heaterPID.Compute(); //returns true if it was time to recalculate
      // Define output of PID as the PWMoutput (will be at 0 the first time around as the PID needs two data points for computation)
      if (newPID ==true) PWMoutput = PIDOutput;  
  }
   
  //****** perform a series of tests on thermistor reading and temperature vs setpoint and over time
  //-- override PWMoutput if indicated
  //errorCheck(errorCode);
  //need to concatenate a message to the serial output line
	
  if (PWMoutput!= PWMoutputLast) {
    // Adjust the duty cycle of the PWM pin connected to the MOSFET fron the PID output if the value has changed
    analogWrite(pwmPinOut, PWMoutput);
    PWMoutputLast = PWMoutput;
  }

  // Print buck converter voltage, glass temperature, and PWMoutput to serial
  int tNow = millis();
  if ( (tNow - displayLast) >= displayInterval) {
    printParametersToSerial();
    displayLast = tNow;
  }

  if (Serial.available() > 0) {

      // Read incomingMX300 until end character is reached or maxMessageLength characters are read.
     lengthOfMessage = Serial.readBytesUntil(endMessage,incomingSerial,serialMML);
    if (lengthOfMessage <= serialMML) {
      if (incomingSerial[0] == 'P' || incomingSerial[0] == 'T') {
        parsePIDCmd();
        userOverRide = true;  
      }
      if ((incomingSerial[0] == 'O') & (incomingSerial[0] == 'K')) {
          Serial.println("OK. Error acknoledgement received");
          errorAcknowledged = true;
          
      }
      if ( incomingSerial[0] == 'h' || incomingSerial[0] == 'H' || incomingSerial[0] == '?') {
        Serial.println("\n >>>> PID Serial Commands <<<<\n" );
        Serial.println(" End commands with '.' for faster responses" );
        Serial.println(" Interacting with the PID controller" );
        Serial.println("   Pma - call  heaterPID.SetMode(AUTOMATIC)" );
        Serial.println("   Pmm - call  heaterPID.SetMode(Manual)" );
        Serial.println("   Pmg - call  heaterPID.GetMode()" );
        Serial.println("   Ponnn. - manually set PWMoutput (only usual in MANUAL mode" );
        Serial.println("   Plnnn. - call heaterPID.SetOutputLimits(0, nnn)" );
        Serial.println("   Ptpppiiiddd. - heaterPID.SetTunings(ppp,iii,ddd)" );
        Serial.println("   Pdnnnn. - call heaterPID.SetSampleTime(nnnn) & STEINHART::setSampleTime(nnnn)" );
        Serial.println("   Tp - print temperature history" );
      }
    }
   }// if (Serial.available() > 0)

  // Delay determines how often loop repeats
  delay(20); 
}

void parsePIDCmd(){
      
       if (incomingSerial[0] == 'P') {
        if (incomingSerial[1] == 'm') {
          if (incomingSerial[2] == 'a') {
            heaterPID.SetMode(AUTOMATIC); 
            Serial.println(" PID set to ATUOMATIC"); 
            //PIDmode = 1;
             PIDmode = heaterPID.GetMode();
          } 
          if (incomingSerial[2] == 'm') {
            heaterPID.SetMode(MANUAL); 
            //PIDmode = 0;
            PIDmode = heaterPID.GetMode();
            Serial.println(" PID set to MANUAL"); 
          } 
          if (incomingSerial[2] == 'g') {
            heaterPID.SetMode(MANUAL); 
            Serial.print(" PID is in mode: "); 
            Serial.println(heaterPID.GetMode()); 
          }         
        } 
        else if (incomingSerial[1] == 'o') {
          char newOutput[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
          PWMoutput = atof(newOutput);
          Serial.print(" PWMoutput: "); 
          Serial.println(PWMoutput); 
         
        }
        else if (incomingSerial[1] == 'l') {
          char upper[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
          outPutMax = atof(upper);
          heaterPID.SetOutputLimits(0,outPutMax);
          Serial.print("heaterPID.SetOutputLimits(0,outPutMax)"); 
          Serial.println(outPutMax); 
          Serial.println(")"); 
          
               
         }
        else if (incomingSerial[1] == 'd') {
          char newDelay[4] = {incomingSerial[2],incomingSerial[3],incomingSerial[4],incomingSerial[5]};
          heaterPID.SetSampleTime(newDelay);
          Serial.print(" PID interval now ");
          Serial.println(newDelay);
         }
        else if (incomingSerial[1] == 't') {
          
          char ppp[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
          char iii[3] = {incomingSerial[5],incomingSerial[6],incomingSerial[7]};
          char ddd[3] = {incomingSerial[8],incomingSerial[9],incomingSerial[10]};
          PIDKp = 0.0;
          PIDKi = 0.0;
          PIDKd = 0.0;
          PIDKp = atol(ppp);
          PIDKd = atol(ddd);
          PIDKi = atol(iii);
          PIDKi = (PIDKi-PIDKp)/1000; 
          Serial.print(" Setting P = ");
          Serial.print(PIDKp);
          Serial.print(", I = ");
          Serial.print(PIDKi);
          Serial.print(", D = ");
          Serial.println(PIDKd);
          heaterPID.SetTunings(PIDKp,PIDKi,PIDKd);

         }
       } else if(incomingSerial[0] == "T" && incomingSerial[1] == "p" ){
          if (incomingSerial[2] == 'p') {
            for( int i =0; i< historySize; i++) {
              Serial.print(glassTempHistory[i]);
              Serial.println(", "); 
            }
          }
       } else {
        Serial.println(" Command not recognized. Press 'h' for index of commands");
       }
}

// Get the average voltage from the tunable buck converter, return tunable buck converter voltage
void getBuckConverterVoltage(double &newVoltage)
{
  // Read voltage from voltmeter 
  double thisVoltage = 0.0;
	
  // Average nSampleReadings from heater voltage source with a slight delay
  for (int i = 0; i < nSampleReadings; i++) 
  {
    thisVoltage += analogRead(VOLTMETERPIN)/ nSampleReadings;
    delay(3);
  }
 
  // Convert the 0-1024 analog input to a voltage
  thisVoltage = (thisVoltage * boardVout) / 1024.0;   
  //Correct for the voltage-divide upstream of the measured voltage
  thisVoltage = thisVoltage / (r2Coefficient / (r1Coefficient + r2Coefficient));

  // If voltage is very small, consider it negligible and set it to 0        
  if (thisVoltage < 0.1)  {  thisVoltage = 0.0;   }

  newVoltage = thisVoltage;
  
}

void errorCheck(int &thisError)
{
  int newError =0; //assume all is well
  
  // Check if thermistor is not connected. Temp will read as -273C
  if ( glassTemperature < 0) {
      newError = 1;
      //Assume PID will be full throttle. Overrie to 0.
      PWMoutput = 0;
  } 

  
  if (glassTemperature >= maxGlassTemperature) {
      newError = 1;
      //gently throttle back the power output to reduce temperature
      //while too high output will half recursively - eventually to 0.
      //does
      PWMoutput /= 2;

      Serial.println("Error Code: 1 - max glass temp reached. Throttling back output");
  }
  
  if (historyFilled) {

    // calculate: delta over history, slope in C/min, check if at max temp
      double deltaOverTime = glassTemperature - glassTempHistory[ (historyIndex - historySize)%historySize];
      double deltaFromSetPt = glassTemperature - glassSetpoint;
      double slope = deltaOverTime / (glassInterval*60/60000);  //degrees/minute
      //slope (over 50 samples) when ramping up can be >3 C/min, 
      //when stable, always within +/-0.7C/min
      //when off, falling from 35C in ambient temp, goes to ~ -1C/min
     
      //Thermal run away. Getting hotter than setpoint and beyond minor overshoot      
      if (deltaFromSetPt>2.0) {
      	newError = 2;
      	Serial.println("Error Code: 2 - Thermal Run away detected. Throttling back"); 
        //Throttle back to give a chance to self-correct
        PWMoutput /= 2;  
      }

      //If temp not reaching desired setpoint and slope is relatively flat, 
      // assume power is insufficient to reach setpoint
      if (deltaFromSetPt < -0.5 && slope < 0.5 && slope > -1  ) {
        newError = 3;
        Serial.println("Error Code: 3 - Under-powered. Consider increasing max output or optimizing P constant"); 
      }

      // Heater failure: temperature falling unexpectedly when in auto mode
      // assume power is insufficient to reach setpoint
      if (deltaFromSetPt < -0.5 && slope <= -1  ) {
        newError = 4;
        Serial.println("Error Code: 4 - Heater appears to have failed. "); 
      }

  }

  thisError = newError;
  
}

void appendToGlassHistory(double value)
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
  Serial.print(glassSetpoint);
  Serial.print("\tError:");
  Serial.println(errorCode);
 
}
