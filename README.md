# Introduction
We are aiming to create the base code needed to control a resistively heated sheet of glass. This glass will be the lid of a stage-top incubator on a microscope stage. 
The incubator needs to control:
1) Monitor and set the temperature of the heated glass lid
2) Monitor the air temperature in the incubator, and aim to sustain 37C by adjusting the temperature of the lid
3) CO2 levels, measured as %
4) Monitor relative humidity, which will be a function of temparature. Ideally, the system would alert the user (audibly or electronically) if the RH falls below some expected level once temperature stabilizes
5) Integrate the temperature control into the current code that controls CO2 and data logging
    
# Approach to heating
## Find a PID library
Start with using Brett Beuaregard's PID library, which seems to be the current open access gold standard
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

## Optimize the PID controller

This stage will need to rewuire that we assess:
### 1) A reasonable voltage to apply to the heater
   The heater lid from Cell MicroControls (~8 Ohms, 80 x 120 mm) did not provide sufficient heat to warm the incubator, event at ~60C
   We needed to revert to our larger sheet of glass with our DIY resistive system
   We are currently using 5 mm thick glass to provide a more crack-resistant substrate
   Our current "ladder" heater design has resistance of ~ 2.8 Ohms. Slightly higher (5-8 Ohsm) would be better
   Current (I, Amps) = voltage / resistance, where we have a variable Bucker Converter to manually adjust between 0-24 V
   heating Power ( P, watts) = V^2 / R
   So in principles, we should be able to increase power faster than current increases (heating other components)

### 2) Optimize the PID constants of the PID controller

  Initally, we should simply aim to maintain a set temp of the sheet of glass, assuming we will need temperatures of 40-55C on the glass to ramp up to and maintain 37C air in the chamber.

  `#4169E1` Note that the changes in temperature of the glass measured by the thermistor will be relatively slow. This will need to be taken into account in the frequency of updating the PID output (PWM of the MOSFET) to avoid "thrash" or rampant changes of out put `#000000`

# Safety feautres and considerations
1) Determine the applied voltage for the glass heater that at 100% current flow (e.g. MOSFET 100% duty cycle) at maxes out <= 60C to minimize chance of burn
2) Incorporate thermal run away detection (i.e. where the heater is applying lots of power, but the temperature isn't increasing (e.g. ther thermistor fall off the glass)
3) Monitor for a failure / disconnetion of the thermistor. This will be signalling by temperature jumping to a max value
 
  
