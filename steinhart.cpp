#include "Arduino.h"
#include <steinhart.h>

//where the calculation of analog input to temp is done.

//need to think through what needs to use pointers etc
//Constructor
steinhart::steinhart(int* analogPin, int* analogIn, float* steinhartOut, long* thermistoResistance, long* thermistorNominalTemp, long* bCoefficient, long* seriesResistor) {
    
    myAnalogPin = analogPin;
    myAnalogIn = analogIn;
    mySteinhartOut = steinhartOut;
    myThermistoResistance = thermistoResistance;
    myThermistorNominalTemp = thermistorNominalTemp;
    myBCoefficient =bCoefficient ;
    mySeriesResistor = seriesResistor;
    inCelcius = true;

    sampleTime = 500;  
    //set 
}

bool steinhart::Compute {

    //check if sufficient time has elapsed to calculate temp
    unsigned long now = millis()
    unsigned long deltaTime = (now - prevTime);

    if (delatTime>sampleTime) {

        //measured voltage on analogIn pin
        for (int i = 0; i < nSamples; i++) {
          thermistorReading += analogRead(myAnalogPin);
          delay(5);
        }
        thermistorReading /= nSamples;
        thermistorVoltage = 1023 / thermistorReading - 1;      

      // Convert the thermistor voltage to resistance
        float resistance = mySeriesResistor / thermistorVoltage; // R / Ro
        steinhartOut = resistance / myThermistorNominalTemp;         // ln(R / Ro)
        steinhartOut = log(steinhartOut);          // 1 / B * ln(R / Ro)
        steinhartOut /= myBCoefficient;         // + (1 / To), convert To to Celsius from Kelvin
        steinhartOut += 1.0 / (myThermistorNominalTemp + 273.150);
        // Invert
        steinhartOut = 1.0 / steinhartOut;
        // Convert from Kelvin to Celsius
        steinhartOut -= 273.150;
       
        if (!inCelcius) {
            steinhartOut = (9/5)*steinhartOut + 32; //calculate output in celcius
        }  
            
    }  

}

void setsSampleTime(int newSampleTime) {

    if (newSampleTime > 0)
      sampleTime = (unsignedLong)newSampleTime;    
    }
}

void setAnalogSamples(int newNumSamples) {
  if (newNumSamples > 0 ){
    nSamples = newNumSamples;
  }
}
//set temperature output scale
void steinhart::celciusOut(int Scale) {

    bool newScale = (Scale == CELCIUS);
    if (newScale){
       inCelcius = true;   
    } else {
       inCelcius = false; 
    }
}

long getNominalResistance() {return myThermistoResistance; }
long getCoefficient() { return myBCoefficient: }
long getNominalTemp(){ return  myThermistorNominalTemp;}
long getSeriesResistance(){ return mySeriesResistor;)
                           

