# Introduction
We have created code to control a resistively heated sheet of glass. This glass will be the lid of a stage-top incubator on a microscope stage. 
The incubator controls:
1) the temperature of the heated glass lid
2) the air temperature in the incubator, and adjusted the glass temperature to maintain the air temp setpoint
3) CO2 levels, measured as % 

The incubator also monitors:
1) relative humidity
2)     Ideally, the system would alert the user (audibly or electronically) if the RH falls below some expected level once temperature stabilizes
3) air pressure
4) time of day
    
# Approach to heating
## Employ a well-validated PID library
We are using Brett Beuaregard's PID library, which seems to be the current open access gold standard
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

## Optimize the PID controller

This stage will need to require that we assess:
### 1) A reasonable voltage to apply to the heater
   Cell MicroControls' ITO coated glass (~8 Ohms, 80 x 120 mm) did not provide sufficient heat to warm the incubator, event at ~60C
   We reverted to our 110 x 145 x 5 mm sheet of soft glass with our DIY resistive system (Permatex 09227 Rear Window Defogger Repair Ki)
   We moved from ~2 mm to 5 mm thick glass to reduce the chance of uneven heating causing cracking
   Our current "ladder" heater design has resistance of ~ 2.8 Ohms. Slightly higher (5-8 Ohsm) would be better
   Current (I, Amps) = voltage / resistance, where we have a variable Bucker Converter to manually adjust between 0-24 V
   heating Power ( P, watts) = V^2 / R
   In principle, we should be able to increase power faster than current increases (heating other components)

### 2) Optimize the PID constants of the PID controller

  Initally, we should simply aim to maintain a set temp of the sheet of glass, assuming we will need temperatures of 40-55C on the glass to ramp up to and maintain 37C air in the chamber.

  `#4169E1` Note that the changes in temperature of the glass measured by the thermistor will be relatively slow. This will need to be taken into account in the frequency of updating the PID output (PWM of the MOSFET) to avoid "thrash" or rampant changes of out put `#000000`

# Safety feautres and considerations
1) Determine the applied voltage for the glass heater that at 100% current flow (e.g. MOSFET 100% duty cycle) at maxes out <= 60C to minimize chance of burn
2) Incorporate thermal run away detection (i.e. where the heater is applying lots of power, but the temperature isn't increasing (e.g. ther thermistor fall off the glass)
3) Monitor for a failure / disconnetion of the thermistor. This will be indicated by temperature jumping to -273 C.
 
  
