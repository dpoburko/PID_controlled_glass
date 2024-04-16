#include "Arduino.h"
#include <steinhart.h>

//where the calculation of analog input to temp is done.

//need to think through what needs to use pointers etc
//Constructor
steinhart::steinhart() {

}

bool steinhart::Compute {


  
}

  // Convert the thermistor voltage to resistance
  float resistance = SERIESRESISTOR / thermistorVoltage;
  // R / Ro
  float steinhartTemporaryVariable = resistance / THERMISTORNOMINAL;
  // ln(R / Ro)
  steinhartTemporaryVariable = log(steinhartTemporaryVariable);
  // 1 / B * ln(R / Ro)
  steinhartTemporaryVariable /= BSTEINHARTCOEFFICIENT;
  // + (1 / To), convert To to Celsius from Kelvin
  steinhartTemporaryVariable += 1.0 / (TEMPERATURENOMINAL + 273.150);
  // Invert
  steinhartTemporaryVariable = 1.0 / steinhartTemporaryVariable;
  // Convert from Kelvin to Celsius
  steinhartTemporaryVariable -= 273.150;

  return steinhartTemporaryVariable;
