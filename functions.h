#ifndef functions_H
#define functions_H
#include "Arduino.h"
#include "structures.h"


// Note that the functions defined here should only be called from the .ino file.
/*If they need to be call from other .h or .cpp files, some gaurd rails need to be used like
1. Declare functions here in the .h as 'inline'
  inline void sensorUpdate () {}

2. Declaire functions in functions.h and define them in functions.cpp

3. Use static keyword, so each file calling sensorUpdate () gets it's own copy
*/

double numberOfSampleVoltageReadings = 11.0;

// Get the average voltage from the tunable buck converter, return tunable buck converter voltage
void GetBuckConverterVoltage(double &newVoltage,double rc1 ,double rc2, double boardVoltageOut )
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
  thisVoltage = (thisVoltage * boardVoltageOut) / 1023.0;   
  //Correct for the voltage-divide upstream of the measured voltage
  thisVoltage = thisVoltage / (rc2 / (rc1 + rc2));

  // If voltage is very small, consider it negligible and set it to 0        
  if (thisVoltage < 0.1)  
  {  
    thisVoltage = 0.0;   
  }
  newVoltage = thisVoltage; 
}



void CheckGlassSetpoint(generalSensor &enclosureTemperature, generalSensor &lidTemperature, thermistor &lidThermistor,String &msgBuffer)

{
  // Current time in milliseconds since the program has been running
  int currentTimeMilliseconds = millis();

  // Calculate gap from air temperature setpoint
  enclosureTemperature.deviation = enclosureTemperature.value - enclosureTemperature.setpoint;

  lidThermistor.autoSetpointChange = 0;

  // If it has been long enough since the last glass setpoint update and the history array has enough data, check if the glass setpoint needs to be updated
  if ((millis() - lidTemperature.setpointLastUpdate) >= lidTemperature.setpointInterval && lidTemperature.historyFilled == true)
  {

    // If the glass temperature has been stable and the air temperature is significantly higher than the air temperature setpoint, update the glass setpoint
    {

      //If air temp is too high, reduce glass temp, if too low, increase glass temp. This toggles direction of change.
      float deviationDirection = 0.0;
      if (enclosureTemperature.deviation>=0) deviationDirection = -1.0;
      if (enclosureTemperature.deviation<0) deviationDirection = 1.0;

      double deviations[] = {1.0,0.7, 0.4 ,0.2 ,0.15,0.1,0};
      double corrections[] = {2,1.75, 1.5 ,1.25,0.50,0.25,0};
      int nDeviations = sizeof(deviations) / sizeof(deviations[0]);
      
      for (int d=0;d<nDeviations;d++) {
        if( abs(enclosureTemperature.deviation) > deviations[d]){
          lidThermistor.autoSetpointChange = deviationDirection * corrections[d];
          d = nDeviations;
        }
        
      }

      //increment the deviation of glassSetpoint from the most recent user-defined glassSetpoint
      lidThermistor.cummSetpointChange += lidThermistor.autoSetpointChange;
      
      // if the delta is positive, limit the total possible increase to 5C. The user can reset this by manually increasing the set point.
      //The should limit overshoot in the automatic increases in glass set point
      if (lidThermistor.autoSetpointChange>0 && lidThermistor.cummSetpointChange >= lidThermistor.maxAutoIncrease) {
        lidThermistor.cummSetpointChange = lidThermistor.maxAutoIncrease;
        lidThermistor.autoSetpointChange = 0;
      }

      lidTemperature.setpoint += lidThermistor.autoSetpointChange;

      // Update the last glass setpoint update time to the current time
      lidTemperature.setpointLastUpdate = currentTimeMilliseconds;

      if (lidThermistor.autoSetpointChange != 0.0) {
        // Send a message to the serial
        msgBuffer += "Glass Setpoint updated to ";
        msgBuffer += String(lidTemperature.setpoint);
      }
    }
  }
}


//Note, with change to generalSensor for temperatures and use of sensorUpdate():
// 1. the erroneous check needs to come before sensorUpdate and the values are logged - Done
// 2. the erroneous check should work on a single general sensor, i.e. lidTemperature or enclosureTemperature
// 3. it should include a double Tolerance to identify erroneous readings
//%%%%% Need to add increased tolerance depending on current slope. When heating, it needs to be more generous.

void RemoveErroneousSensorReadings(generalSensor &sensor, double tolerance, String &msgBuffer)
{
  // Calculate difference between current glass temperature and previous glass temperature reading
  //Since glassTemperatureHistory is a circular buffer, we access the values 1 and 2 indices before the current historyArraysIndex
  if ( (sensor.historyFilled == true) && (sensor.history[(sensor.index - 1 + sensor.historySize)%sensor.historySize] != sensor.history[(sensor.index - 2 + sensor.historySize)%sensor.historySize]) )
  {
    double prev1Value = sensor.history[(sensor.index - 1 + sensor.historySize)%sensor.historySize]; 
    double prev2Value = sensor.history[(sensor.index - 2 + sensor.historySize)%sensor.historySize]; 
    if ( (abs(sensor.value - prev1Value) > tolerance) && (abs(sensor.value - prev2Value) > tolerance) ) 
    {
      // Send a message to the serial
      msgBuffer += "Erroneous ";
      msgBuffer += sensor.name;
      msgBuffer += " reading of ";
      msgBuffer += sensor.value;
      msgBuffer += " vs ";
      msgBuffer += String(prev1Value,2);
      msgBuffer += " & ";
      msgBuffer += String(prev2Value,2);
      msgBuffer += ". Reassigned to ";
      sensor.value = (prev1Value + prev2Value)/2;
      sensor.history[(sensor.index - 1 + sensor.historySize)%sensor.historySize] = sensor.value; 
      msgBuffer += sensor.value;
      
    }
  }
 
}

void sensorUpdate(generalSensor& sensor) {

  //add the value to the array and note the time that value is added to the history in millis
  sensor.history[sensor.index] = sensor.value;
  sensor.deviation = sensor.value - sensor.setpoint;
  sensor.time[sensor.index] = millis();
  if (sensor.value < sensor.minimum) sensor.minimum = sensor.value;
  if (sensor.value > sensor.maximum) sensor.maximum = sensor.value;

  //determine slope and append to slope history
  
  if (sensor.historyFilled) {
    
    //determine slope
    sensor.slope[sensor.index] = (sensor.value - sensor.history[(sensor.index - sensor.slopeInterval + sensor.historySize) % sensor.historySize]) / 
                                 ( (sensor.time[sensor.index] - sensor.time[(sensor.index - sensor.slopeInterval + sensor.historySize) % sensor.historySize]) / sensor.slopeUnits);
 
    if (abs(sensor.slope[sensor.index])>10000) {
      sensor.slope[sensor.index] = 0;
    }

    //calculate deviation squared for mse
    sensor.errorHistory[sensor.index] = (sensor.deviation)*(sensor.deviation);

    //calculate average and mean squared error over the history
    sensor.average = 0;
    sensor.meanSquareError = 0;
    
    for (int i = 0; i < sensor.historySize; i++) {
      
      sensor.average += sensor.history[i];
      sensor.meanSquareError += sensor.errorHistory[i];
      
    }
    
    sensor.average /= sensor.historySize;
    sensor.meanSquareError /= sensor.historySize;
    sensor.meanSquareError = sqrt(sensor.meanSquareError);
	  
  } else {
    //if history is not yet filled ....
      if (sensor.index<=1) {
        sensor.slope[sensor.index] = (sensor.value - sensor.history[0]) / ( (sensor.time[sensor.index] - sensor.time[0]) / sensor.slopeUnits);
      } else if (sensor.index < sensor.slopeInterval+2) {
        sensor.slope[sensor.index] = (sensor.value - sensor.history[1]) / ( (sensor.time[sensor.index] - sensor.time[1]) / sensor.slopeUnits);
      } else {
	      sensor.slope[sensor.index] = (sensor.value - sensor.history[(sensor.index - sensor.slopeInterval + sensor.historySize) % sensor.historySize]) / 
                                 ( (sensor.time[sensor.index] - sensor.time[(sensor.index - sensor.slopeInterval + sensor.historySize) % sensor.historySize]) / sensor.slopeUnits);
      }   
      
      sensor.meanSquareError = 0;
      sensor.average = 0;
      
      if (sensor.index==0 ) {
        sensor.errorHistory[sensor.index] = 0;
      } else {
        sensor.errorHistory[sensor.index] = (sensor.value-sensor.setpoint)*(sensor.value-sensor.setpoint);
      }
      
      for (int i = 0; i < sensor.index; i++) {
        sensor.average += sensor.history[i];
        sensor.meanSquareError += sensor.errorHistory[i];
      }
      sensor.average /= (sensor.index+1);
      sensor.meanSquareError /= (sensor.index+1);
      sensor.meanSquareError = sqrt(sensor.meanSquareError);
  }

  sensor.index++;

  //this is essentially used like a circular buffer
  if (sensor.index == sensor.historySize) {
    sensor.historyFilled = true;
    sensor.index = 0;  // reset
  }
}

// // Print all relevant values to serial
// // Eventually this will need to merge with logToSD.cpp in IncuStage
void PrintParametersToSerial(generalSensor &enclosureTemperature, generalSensor &lidTemperature, PIDextras &heaterValues, String &msgBuffer,String &errorBuffer,String &logBuffer,uint16_t &nLogged )
{

  if (nLogged == 0) {
    Serial.println(" ");
    logBuffer += "\tT(enclosure):\tT(lid):\tT(encl.slope):\tT(lidTemperature.slope):\tT(mse):\tsetPt(lid):\tsetPt(enclosure):\tPWMOut:\tV(in):\tPID:\tError:\tMsg:";
    Serial.println(logBuffer);
    logBuffer = "";
  }

  //Note that sensor.Index is advanced at the end of updateSensor, so the current index is 1 ahead of the most recently stored values
  int enclosureIndexA = (enclosureTemperature.index -1  + enclosureTemperature.historySize) % enclosureTemperature.historySize;
  int enclosureIndexB = (enclosureTemperature.index -1 - enclosureTemperature.slopeInterval + enclosureTemperature.historySize) % enclosureTemperature.historySize;
  int lidIndexA = (lidTemperature.index  -1 + lidTemperature.historySize) % lidTemperature.historySize;
  int lidIndexB = (lidTemperature.index  -1 - lidTemperature.slopeInterval + lidTemperature.historySize) % lidTemperature.historySize;

  logBuffer = "";
  logBuffer += "\t";
  logBuffer += String(enclosureTemperature.value, 2);
  logBuffer += "\t";
  logBuffer += String(lidTemperature.value, 2);
  logBuffer += "\t";
  logBuffer += String(enclosureTemperature.slope[enclosureIndexA], 3);
  logBuffer += "\t";
  logBuffer += String(lidTemperature.slope[lidIndexA], 3);
  logBuffer += "\t";
  logBuffer += String(lidTemperature.meanSquareError, 3);
  logBuffer += "\t";
  logBuffer += String(lidTemperature.setpoint, 2);
  logBuffer += "\t";
  logBuffer += String(enclosureTemperature.setpoint, 1);
  logBuffer += "\t";
  logBuffer += String(heaterValues.outputToDevice);
  logBuffer += "\t";
  //logBuffer += String(buckConverterVoltage, 2);
  logBuffer += String(heaterValues.maxInput,2);
  logBuffer += "\t";
  logBuffer += String(heaterValues.P);
  logBuffer += "/";
  logBuffer += String(heaterValues.I);
  logBuffer += "/";
  logBuffer += String(heaterValues.D);
  logBuffer += "\t";
  logBuffer += errorBuffer;
  logBuffer += "\t";
  logBuffer += msgBuffer;
  logBuffer += "\n";
  Serial.print(logBuffer);
  logBuffer = "";
  msgBuffer = " ";
  errorBuffer = " ";
  nLogged++;
}


#endif