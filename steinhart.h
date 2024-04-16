
#ifndef steinhard_v1.h
#define steinhard_v1.h
#define LIBRARY_VERSION 1.0.0

class steinhart {

  public:
  //constants used in functions
  #define CELCIUS 1
  #define FARENHEIT 0
  

    //constructor, needs analog in PIN, temperature out, nomimal thermistor resistance, nominal temp (C), B coeeficient, series resistor 
    steinhart(int, int, float, long, long, long,long);
      
    bool Compute(); //performs calculation from current voltage input

    int celciusOut(); // celcius or farenheit
    
    void setSampleTime(); //define time between measurements

    void setAnalogSamples(); //number of times the analog in pin is averaged
      
  //Display functions:
    long getNominalResistance();
    long getCoefficient();
    long getNominalTemp();
    long getSeriesResistance();
    

}  

  private:
    void Initialize();
    //pointers to the Steinhart parameters
    int *analogPin;
    int *analogIn;
    float *steinhartOut;
    long *thermistoResistance; 
    long *thermistorNominalTemp;
    long *bCoefficient;
    long *seriesResistor;
    inCelcius
    
    unsigned long prevTime;
    unsgined long sampelTime;
    bool inCelcius;
    int nSamples;
    float thermistorReading;
    float thermistorVoltage;
    
}; //close steinhart class
#endif
