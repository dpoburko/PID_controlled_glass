#include <Arduino.h>
#include "STEINHART.h"

//where the calculation of analog input to temp is done.

//need to think through what needs to use pointers etc
//Constructor
STEINHART::STEINHART(int analogPin, double* steinhartOut, long thermistoResistance, long thermistorNominalTemp, long bCoefficient, long seriesResistor) {
    
    myAnalogPin = analogPin;
    mySteinhartOut = steinhartOut;
    myThermistorResistance = thermistoResistance;
    myThermistorNominalTemp = thermistorNominalTemp;
    myBCoefficient = bCoefficient ;
    mySeriesResistor = seriesResistor;
    inCelcius = true;
    nSamples = 11.0; //doule to allow decimal average
    sampleTime = 500;  
    //set 
}

bool STEINHART::read() {

    //check if sufficient time has elapsed to calculate temp
    unsigned long now = millis();
    unsigned long deltaTime = (now - prevTime);
    thermistorReading = 0;
    double output;  
    thermistorVoltage = 0;       
    int readings[(int)nSamples];
    int thisSample;
    if (deltaTime>sampleTime) {
        
        //measured voltage on analogIn pin several times to average noise
        for (int i = 0; i < nSamples; i++) {
          //thermistorReading += analogRead(myAnalogPin)/ nSamples;
          thisSample = analogRead(myAnalogPin);
          //thermistorVoltage += analogRead(myAnalogPin);
          thermistorVoltage += thisSample;
          readings[i] = thisSample;
          delay(1);
        }
        
        thermistorVoltage /= nSamples;
        sortReadings(readings, (int)nSamples);
        int median = readings[(int)nSamples / 2];
        thermistorVoltage = median;
      
        float sum = 0.0;
        int count = 0;
      
        for (int i = 0; i < (int)nSamples; i++) {
          if (abs(readings[i] - median) <= 5) {
            sum += readings[i];
            count++;
          }
        }
        thermistorVoltage = sum/count;
        

      // Convert the thermistor voltage to resistance
        thermistorVoltage = 1023 / thermistorVoltage - 1;      
        thermistorVoltage = mySeriesResistor / thermistorVoltage; // R / Ro
        output = thermistorVoltage / myThermistorResistance;         // ln(R / Ro)
        output = log(output);          // 1 / B * ln(R / Ro)
        output /= myBCoefficient;         // + (1 / To), convert To to Celsius from Kelvin
        output += 1.0 / (myThermistorNominalTemp + 273.150);
        output = 1.0 / output;
        output -= 273.150;         // Convert from Kelvin to Celsius
       
        if (!inCelcius) {
            output = (9/5)*output + 32; //calculate output in celcius
        }
        *mySteinhartOut = output;
        prevTime = now;  
        return true;
    }  else {
        return false;
    }

}

void STEINHART::setSampleTime(int newSampleTime) {

    if (newSampleTime > 0){
      sampleTime = (unsigned long)newSampleTime;    
    }
    
}

void STEINHART::setNumSamples(int newNumSamples) {
  if (newNumSamples > 0 ){
    nSamples = newNumSamples;
  }
}

//set temperature output scale
void STEINHART::celciusOut(int Scale) {

    bool newScale = (Scale == CELCIUS);
    if (newScale){
       inCelcius = true;   
    } else {
       inCelcius = false; 
    }
}

// Function to sort the readings array using bubble sort
void STEINHART::sortReadings(int arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

long STEINHART::getNominalResistance() {return myThermistorResistance; }
long STEINHART::getCoefficient() { return myBCoefficient; }
long STEINHART::getNominalTemp(){ return  myThermistorNominalTemp;}
long STEINHART::getSeriesResistance(){ return mySeriesResistor;}
                           
