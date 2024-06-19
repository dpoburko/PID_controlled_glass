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
double glassSetpoint = 45.0;
const int maxGlassTemperature = 68;
double PIDStartDelta = 1.5;

// ******* Set heater limits (bits), these are arbitrary values for now ***************************************
double outPutMax = 45.0; // Currently set to limit total current to the limits of the PSU or Buck converter. 
double outPutHolding = 40.0; // Currently set to limit total current to the limits of the PSU or Buck converter. 
double outPutUpperCurrent = outPutHolding;

// Set number of samples to take in order to get an average of the voltage and temperature data 
double nSampleReadings = 11.0; //needs to be double for average to have decimals

//===== PID controller variables ===============================================================================
// These values (2/24/21) are working well for the 5 mm thick glass
double PIDKp = 2, PIDKi = 96, PIDKd = 21;

// Define PID variables (from PID library)
double PIDSetpoint, PIDInput, PIDOutput;
double PWMoutput = 0.0;
double PWMoutputLast = 0.0;
double PWMoutputIfError = 10.0; //value used to hold glass temp warm but safe
String errorBuffer;
String msgBuffer;
String logBuffer;
bool PIDmode = 0; 
long errorCode = 0; //catch all for errors

bool errorAcknowledged = false;
bool userOverRide = false;
bool newPID = false; //check if output has changed since last loop
// Create PID object (start with conservative tuning constants to be safe)

//===== Resistors r1 and r2 used for ~1:10 voltage divider to detection input voltage ============================
const double r1Coefficient = 9810;
const double r2Coefficient = 983;
double buckConverterVoltage = 0;

//===== Variables for the thermistor attached to the glass lid ==================================================================
int thermistor1Pin = A0; // // Thermistor pin
int thermistor2Pin = A4; // // Thermistor pin

double glassTemperature = 0.0; // Initialize glass temperature (in degrees Celsius) and PWMoutput value needed to change glass temperature
double glassTempSlope = 0.0;
double airTemperature = 20.0;

int glassInterval = 2000;
const int historySize = 60;
int historyIndex = 0;
int endIndex = 0;
bool historyFilled = false;
double glassTempHistory[historySize];// Create glass temperature array (of size 50 for now)
double mseHistory[historySize];// Create glass temperature array (of size 50 for now)
double mseTemp = 0.0;
bool newTemp = false;

double thermistorVoltage = 0.0;
long Rnominal = 10000;  // Thermistor resistance at 25 C 
long Tnominal = 25; // Temperature for nominal resistance (almost always 25 C)
long bCoeff = 3435; // B coefficient for Steinhart equation 
long Rseries = 9985; // measured R for whatever seriers resistor (~10 kOhms) is used


//===== Initialize the Steinhart temp calculation. Only glass temp needs to be a reference ===========================
STEINHART thermistor1(thermistor1Pin, &glassTemperature, Rnominal, Tnominal, bCoeff, Rseries);
STEINHART thermistor2(thermistor2Pin, &airTemperature, Rnominal, Tnominal, bCoeff, Rseries);
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

  errorBuffer.reserve(256);
  msgBuffer.reserve(128);
  dataBuffer.reserve(512);
	
//%% - this feels redundant. Just need either PIDSetpoint or glassSetpoint
  // Initialize desired glass temperature
  PIDSetpoint = glassSetpoint;
 
  // Initialize PID
  heaterPID.SetMode(AUTOMATIC);
  PIDmode = heaterPID.GetMode();     
  heaterPID.SetOutputLimits(0, outPutMax); //set to limits of 

  thermistor1.setSampleTime(glassInterval);
  thermistor1.read();
  thermistor2.setSampleTime(glassInterval);
  thermistor2.read();

  Serial.print("   ===== PID controlled glass heater =====================");
  Serial.println("   Stage-Top Incubator component"); 
  Serial.print("   Initial Temperature:");
  Serial.println(glassTemperature,2);

}

void loop() 
{
 
  //call STEINHART library, check if value is updated  
  newTemp = thermistor1.read();
  
  //add the new temp reading to the history array if new value available
  if (newTemp) {
    thermistor2.read();
	  
    // Get the voltage from the tunable buck converter - burried here to match temp check interval
    getBuckConverterVoltage(buckConverterVoltage);
    appendToGlassHistory(glassTemperature);

    if (historyFilled == false) {
      if (historyIndex == 0) {
        glassTempSlope = 0.0 ;
      } else {
        glassTempSlope = (glassTemperature - glassTempHistory[0]) / ( (long)(historyIndex+1)*(long)glassInterval/60000);  //degrees/minute
      }
    } else {
      glassTempSlope = (glassTemperature - glassTempHistory[endIndex]) / (historySize*glassInterval/60000);  //degrees/minute
    }
    
    //mseTemp += abs(glassTemperature-glassSetpoint)/historySize;
    //mseTemp -= mseHistory[endIndex];
    arrayAverage(mseHistory,mseTemp);
     // Define input of the PID as the glass temperature
     PIDInput = glassTemperature; //no reason these can't be hard linked. 

     if (glassTemperature < (glassSetpoint - PIDStartDelta) ) {
      if (outPutUpperCurrent != outPutMax) {
        outPutUpperCurrent = outPutMax;
        heaterPID.SetOutputLimits(0,outPutUpperCurrent);
        Serial.print(" setting outPutUpperCurrent to ");
        Serial.print(outPutMax);
        Serial.println(" (outputMax) for rapid heating");
      }
     } else {
      if (outPutUpperCurrent != outPutHolding) {
        outPutUpperCurrent = outPutHolding;
        heaterPID.SetOutputLimits(0,outPutUpperCurrent);
        Serial.print(" setting outPutUpperCurrent to ");
        Serial.print(outPutHolding);
        Serial.println(" (outPutHolding) for setPoint maintenance");
      }
     }
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
      if (incomingSerial[0] == 'P' || incomingSerial[0] == 'T'|| incomingSerial[0] == 'E') {
        parsePIDCmd();
        userOverRide = true;  
      }
      else if ((incomingSerial[0] == 'O') & (incomingSerial[0] == 'K')) {
          Serial.println("OK. Error acknoledgement received");
          errorAcknowledged = true;
          
      }
      else if ( incomingSerial[0] == 'h' || incomingSerial[0] == 'H' || incomingSerial[0] == '?') {
        Serial.println("\n >>>> PID Serial Commands <<<<\n" );
        Serial.println(" End commands with '.' for faster responses" );
        Serial.println(" Interacting with the PID controller" );
        Serial.println("   Pma - call  heaterPID.SetMode(AUTOMATIC)" );
        Serial.println("   Pmm - call  heaterPID.SetMode(Manual)" );
        Serial.println("   Pmg - call  heaterPID.GetMode()" );
        Serial.println("   Psnnn - update glassSetpoint as nn.n degrees" );
        
        Serial.println("   Ponnn. - manually set PWMoutput (only usual in MANUAL mode" );
        Serial.println("   Plnnn. - call heaterPID.SetOutputLimits(0, nnn)" );
        Serial.println("   Phnnn. - updated holding PWM" );
        Serial.println("   Ptpnnn. - heaterPID.SetTunings(nn.n,iii,ddd)" );
        Serial.println("   Ptinnn. - heaterPID.SetTunings(ppp,nnn,ddd)" );
        Serial.println("   Ptdnnn. - heaterPID.SetTunings(ppp,iii,nnn)" );
        
        Serial.println("   Pdnnnn. - call heaterPID.SetSampleTime(nnnn) & STEINHART::setSampleTime(nnnn)" );
        Serial.println("   Tp - print temperature history" );
        Serial.println("   Ep - print mse history" );
        Serial.println("   >>>>>>>>>>>>>><<<<<<<<<<<<<<<<" );
      }
      else {
        Serial.println("   Serial command not recognized. Press 'h' for help");
      }
    }
   }// if (Serial.available() > 0)

  // Delay determines how often loop repeats
  delay(20); 
}

//consider moving this to a library
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
          else if (incomingSerial[1] == 's') {
            char newSetpoint[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
            glassSetpoint = atof(newSetpoint)/10.0;
            PIDSetpoint = glassSetpoint; 
            Serial.print(" glassSetpoint: "); 
            Serial.println(glassSetpoint); 
           
          }
          else if (incomingSerial[1] == 'l') {
            char upper[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
            outPutMax = atof(upper);
            heaterPID.SetOutputLimits(0,outPutMax);
            Serial.print("   heaterPID.SetOutputLimits(0,"); 
            Serial.print(outPutMax);  
            Serial.println(")");     
           }
          else if (incomingSerial[1] == 'h') {
            char holding[3] = {incomingSerial[2],incomingSerial[3],incomingSerial[4]};
            outPutHolding = atof(holding);
            Serial.print("   outPutHolding now "); 
            Serial.print(outPutHolding,2);  
            Serial.println(" ");     
           }
          else if (incomingSerial[1] == 'd') {
            char newDelay[4] = {incomingSerial[2],incomingSerial[3],incomingSerial[4],incomingSerial[5]};
            int nd = atol(newDelay);
            heaterPID.SetSampleTime(nd);
            Serial.print(" PID interval now ");
            Serial.println(newDelay);
           }
          else if (incomingSerial[1] == 't') {
             if (incomingSerial[2] == 'p') {
              char ppp[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
              PIDKp = 0.0;
              PIDKp = double(atol(ppp))/10.0;
              Serial.print(" Setting P = ");
              Serial.println(PIDKp,1);
            }
             else if (incomingSerial[2] == 'i') {
              char iii[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
              PIDKi = 0.0;
              PIDKi = atol(iii);
              Serial.print(" Setting I = ");
              Serial.println(PIDKi);
            }
             else if (incomingSerial[2] == 'd') {
              char ddd[3] = {incomingSerial[3],incomingSerial[4],incomingSerial[5]};
              PIDKd = 0.0;
              PIDKd = atol(ddd);
              Serial.print(" Setting D = ");
              Serial.println(PIDKd);

            }
            heaterPID.SetTunings(PIDKp,PIDKi,PIDKd);
         }

       } else if(incomingSerial[0] == 'T' && incomingSerial[1] == 'p' ){
            for( int i =0; i< historySize; i++) {
              Serial.print(glassTempHistory[i]);
              Serial.print(", "); 
            }
              Serial.println(" ");
       } else if(incomingSerial[0] == 'E' && incomingSerial[1] == 'p' ){
            for( int i =0; i< historySize; i++) {
              Serial.print(mseHistory[i]);
              Serial.print(", "); 
            }
            Serial.println(" ");
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
    delay(1);
  }
 
  // Convert the 0-1024 analog input to a voltage
  thisVoltage = (thisVoltage * boardVout) / 1024.0;   
	
  //Correct for the voltage-divide upstream of the measured voltage
  thisVoltage = thisVoltage / (r2Coefficient / (r1Coefficient + r2Coefficient));

  // If voltage is very small, consider it negligible and set it to 0        
  if (thisVoltage < 0.1)  {  thisVoltage = 0.0;   }

  newVoltage = thisVoltage;
  
}


//DP 240610
/*
 * Error logging - We need to create a globabl variable like String - errorBuffer that gets information added to it, and appended to a logBuffer that prints out all the usefull data
 * Then also have an eventBuffer to log any system changes to a tab delimited section. 
 * Would also be helpful to log a headers line. 
 */

void errorCheck(int &thisError)
{
  int newError = 0; //assume all is well
	errorBuffer ="Error:";
  // Check if thermistor is not connected. Temp will read as -273C
  if ( glassTemperature < 0) {
      newError = 1;
      //Assume PID will be full throttle. Overrie to 0.
      PWMoutput = 0;
      errorBuffer += String(newError,0);
      errorBuffer += " The glass thermistor appears to be disconected. Output shut off.";
  } 
  
  else if (glassTemperature >= maxGlassTemperature) {
      newError = 1;
      //gently throttle back the power output to reduce temperature
      //while too high output will half recursively - eventually to 0.
      PWMoutput = PWMoutputIfError;
      errorBuffer += String(newError,0);
      errorBuffer += " Max glass temp reached. Throttling output to ";
      errorBuffer += String(PWMoutputIfError,0);
  }

	//Thermal run away. Getting hotter than setpoint and beyond minor overshoot      
	else if (deltaFromSetPt>2.0) {
			newError = 2;
			errorBuffer += String(newError,0);
			errorBuffer += " Thermal runaway detected. Throttling output to ";
			errorBuffer += String(PWMoutputIfError,0);
	}
	  
  else if (historyFilled) {

      //slope (over 50 samples) when ramping up can be >3 C/min, 
      //when stable, always within +/-0.7C/min
      //when off, falling from 35C in ambient temp, goes to ~ -1C/min

      //If temp not reaching desired setpoint and slope is relatively flat, assume power is insufficient to reach setpoint
      if (deltaFromSetPt < -0.5 && glassTempSlope < 0.5 && glassTempSlope > -1  ) {
        newError = 3;
        Serial.println("Error Code: 3 - Under-powered. Consider increasing outPutMax. Enter h on serial monitor for instructions."); 
				errorBuffer += String(newError,0);
				errorBuffer += " Under-powered. Consider increasing outPutMax. Enter h on serial monitor for instructions.";
      }

      // Heater failure: temperature falling unexpectedly. Assume power not getting to the heating elements
      else if (deltaFromSetPt < -0.5 && glassTempSlope <= -1  ) {
        newError = 4;
        PWMoutput = 0;
				errorBuffer += String(newError,0);
				errorBuffer += " Heater failure detected. Shutting off output.";
      }

  }
	else {
		errorBuffer += String(newError,0);
	}

  thisError = newError;

}

void appendToGlassHistory(double value)
{
  glassTempHistory[historyIndex] = value;
  mseHistory[historyIndex] = sqrt(sq(value-glassSetpoint));
  historyIndex++;
  
  if (historyIndex >= historySize) {
    historyIndex = 0;
    historyFilled = true;
  }
  endIndex = historyIndex+1;
  if (historyIndex == historySize-1) {
    endIndex = 0;
  }
}

void arrayAverage(double thisArray[],double &average){

  double arrAvg =0.0;
  for (int i = 0;i<historySize;i++) {
    arrAvg += thisArray[i];
  }
  if (historyFilled) {
    arrAvg /= historySize;   
  } else {
    arrAvg /= long(historyIndex+1);
  }
  average = arrAvg;
}



// Print the buck converter voltage to the serial, pass in the buck converter voltage
void printParametersToSerial()
{
  //getting set up to us buffer to merge data with error codes and event logs
  //need to recall/check syntax for float values with sprintf
  //char logString[];
  //sprintf(logString,"V(in):24.54  T(enclosure):33.36  T(glass):66.11  T(slope):0.04 T(mse):0.047  PWMOut:0.00 PIDKp:1.00  PIDKi:96.00 PIDKd:21.00 PIDmode:1 setPt(glass):66.00"

	logBuffer ="";
	logBuffer += "V(in):"
  logBuffer += String(buckConverterVoltage,2);
  logBuffer += "\tT(enclosure):");
  logBuffer += String(airTemperature,2);
  logBuffer += "\tT(glass):");
  logBuffer += String(glassTemperature,2);
  logBuffer += "\tT(slope):");
  logBuffer += String(glassTempSlope,2);
  logBuffer += "\tT(mse):");
  logBuffer += String(mseTemp,3);
  logBuffer += "\tPWMOut:");
  logBuffer += String(PWMoutput);
  logBuffer += "\tPIDKp:");
  logBuffer += String(PIDKp);
  logBuffer += "\tPIDKi:");
  logBuffer += String(PIDKi);
  logBuffer += "\tPIDKd:");
  logBuffer += String(PIDKd);
  logBuffer += "\tPIDmode:");
  logBuffer += String(PIDmode);
  logBuffer += "\tsetPt(glass):");
  logBuffer += String(glassSetpoint,1);
  logBuffer +=  errorBuffer;
	logBuffer +=  msgBuffer;

	Serial.print(logBuffer);
	logBuffer ="";
	msgBuffer = "/t";
	errorBuffer = "/t";

			/*
  Serial.print("V(in):");    
  Serial.print(buckConverterVoltage,2);
  Serial.print("\tT(enclosure):");
  Serial.print(airTemperature,2);
  Serial.print("\tT(glass):");
  Serial.print(glassTemperature,2);
  Serial.print("\tT(slope):");
  Serial.print(glassTempSlope,2);
  Serial.print("\tT(mse):");
  Serial.print(mseTemp,3);
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
*/

	
}
