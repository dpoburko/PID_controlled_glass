
#ifndef STEINHART_H
#define STEINHART_H

#include "Arduino.h"
#include "structures.h"

#define LIBRARY_VERSION 1.0.0

class STEINHART {

  public:
  //constants used in functions
  #define CELCIUS 1
  #define FARENHEIT 0
  

    //constructor, needs analog in PIN, temperature out, nomimal thermistor resistance, nominal temp (C), B coeeficient, series resistor 
    STEINHART(int, double*, long, long, long,long);
      
    bool read(); //performs calculation from current voltage input

    void celciusOut(int); //  report temperature in celcius or farenheit, 
    
    void setSampleTime(int); //define time in milliseconds between measurements

    void setNumSamples(int); //number of times the analog in pin is averaged
      
  //Display functions:
    long getNominalResistance();
    long getCoefficient();
    long getNominalTemp();
    long getSeriesResistance();

  private:
    void Initialize();
    //pointers to the Steinhart parameters
    //int analogPin;
    int myAnalogPin;
    double *mySteinhartOut;
    long myThermistorResistance; 
    long myThermistorNominalTemp;
    long myBCoefficient;
    double  nSamples;
    double mySeriesResistor;

    unsigned long prevTime;
    unsigned long sampleTime;
    bool inCelcius;
    double thermistorReading;
    double thermistorVoltage;
    
}; //close steinhart class
#endif
