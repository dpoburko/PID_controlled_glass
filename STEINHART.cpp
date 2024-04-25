#include <Arduino.h>
#include "STEINHART.h"

//where the calculation of analog input to temp is done.

//need to think through what needs to use pointers etc
//Constructor
STEINHART::STEINHART(int analogPin, float* steinhartOut, long thermistoResistance, long thermistorNominalTemp, long bCoefficient, float seriesResistor) {
    
    //myAnalogPin = analogPin;
    mySteinhartOut = steinhartOut;
    //myThermistorResistance = thermistoResistance;
    //myThermistorNominalTemp = thermistorNominalTemp;
    //myBCoefficient = bCoefficient ;
    //mySeriesResistor = seriesResistor;
    inCelcius = true;

    sampleTime = 500;  
    //set 
}

bool STEINHART::read() {

    //check if sufficient time has elapsed to calculate temp
    unsigned long now = millis();
    unsigned long deltaTime = (now - prevTime);
    thermistorReading =0;
    int tempReading = 0;
    if (deltaTime>sampleTime) {

        //measured voltage on analogIn pin several times to average noise
        for (int i = 0; i < nSamples; i++) {
          tempReading += analogRead(analogPin);
          //tempReading += analogRead(A0);
          delay(5);
        }
        
        thermistorReading = tempReading / nSamples;
        thermistorVoltage = 1023 / thermistorReading - 1;      

      // Convert the thermistor voltage to resistance
        float resistance = seriesResistor / thermistorVoltage; // R / Ro
        float output;         
        output = resistance / thermistorNominalTemp;         // ln(R / Ro)
        output = log(output);          // 1 / B * ln(R / Ro)
        output /= bCoefficient;         // + (1 / To), convert To to Celsius from Kelvin
        output += 1.0 / (thermistorNominalTemp + 273.150);
        // Invert
        output = 1.0 / output;
        // Convert from Kelvin to Celsius
        output -= 273.150;
       
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

void STEINHART::setAnalogSamples(int newNumSamples) {
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

long STEINHART::getNominalResistance() {return thermistorResistance; }
long STEINHART::getCoefficient() { return bCoefficient; }
long STEINHART::getNominalTemp(){ return  thermistorNominalTemp;}
long STEINHART::getSeriesResistance(){ return seriesResistor;}
                           
