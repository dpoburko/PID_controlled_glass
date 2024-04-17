
#ifndef STEINHART.h
#define STEINHART.h
#define LIBRARY_VERSION 1.0.0

class STEINHART {

  public:
  //constants used in functions
  #define CELCIUS 1
  #define FARENHEIT 0
  

    //constructor, needs analog in PIN, temperature out, nomimal thermistor resistance, nominal temp (C), B coeeficient, series resistor 
    STEINHART(int*, int*, float*, long*, long*, long*,long*);
      
    bool Compute(); //performs calculation from current voltage input

    int celciusOut(); //  report temperature in celcius or farenheit, 
    
    void setSampleTime(int); //define time in milliseconds between measurements

    void setAnalogSamples(int); //number of times the analog in pin is averaged
      
  //Display functions:
    long getNominalResistance();
    long getCoefficient();
    long getNominalTemp();
    long getSeriesResistance();

  private:
    void Initialize();
    //pointers to the Steinhart parameters
    int *myAnalogPin;
    int *myAnalogIn;
    float *mySteinhartOut;
    long *myThermistorResistance; 
    long *myThermistorNominalTemp;
    long *myBCoefficient;
    long *mySeriesResistor;

    unsigned long prevTime;
    unsgined long sampelTime;
    bool inCelcius;
    int nSamples;
    float thermistorReading;
    float thermistorVoltage;
    
}; //close steinhart class
#endif
