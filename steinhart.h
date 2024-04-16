
#ifndef steinhard_v1.h
#define steinhard_v1.h
#define LIBRARY_VERSION 1.0.0

class steinhart {

  public:
  //constants used in functions
    //constructor, needs analog in PIN, nomimal thermistor resistance, nominal temp (C), B coeeficient, series resistor 
    steinhart(int, long, long, long,long);
      
    bool Compute(); //performs calculation from current voltage input

    bool outputScale(); // celcius or farenheit
    
    void setSampleTime(); //define time between measurements
      
  //Display functions:
    long getNominalResistance();
    long getCoefficient();
    long getNominalTemp();
    long getSeriesResistance();

}  

