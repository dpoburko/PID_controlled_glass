#include "Arduino.h"
#include "structures.h"
#include "errorCheck.h"
#include "parseSerial.h"
#include <PID_v1.h>


  parseSerial::parseSerial(serialMsg& aSerial, PID& aHeaterPID, PIDextras& aHeaterValues ,String& aMsgBuffer, generalSensor& aTemp, generalSensor& bTemp)
  :  serialMain(&aSerial), heaterPID(&aHeaterPID), heaterValues(&aHeaterValues), msgBuffer(aMsgBuffer),lidTemperature(&aTemp), enclosureTemperature(&bTemp) { }

  /*
   * Things that need to be passed
   * msgBuffer, PIDmode, heaterPID str, PWMoutput >> lidHeater->output, heaterValues->errorOutput >> lidHeater->errorOutput
   * need to get isAirTemperatureClimbing into a structure.... are use slope and setpoint relationship in its place.
   */
  
  //since the sketch structure is replicated in the constuctor of this library as a pointer with the same name, we don't need to pass the structure as an argugement functions.
  //However, since serialMain is now a pointer, it's values are accessed with '->' instead of '.'
  
  void parseSerial::parse() {
  
      // Read the number of characters until end character is reached or maxMessageLength character is read
      serialMain->length = Serial.readBytesUntil(serialMain->end, serialMain->incoming, serialMain->size);
  
      // If the message length is less than the maximum allowed message length, read the message
      if (serialMain->length <= serialMain->size) 
      {
        // If user inputted P, T, or E, go to parse function to read the next message inputted
        if (serialMain->incoming[0] == 'P' || serialMain->incoming[0] == 'T'|| serialMain->incoming[0] == 'S'|| serialMain->incoming[0] == 'E' || serialMain->incoming[0] == 'L'|| serialMain->incoming[0] == 'N') 
        {
          // Go to parse function to read the next message inputted
          parsePIDCmd();
  
        }

        // If user inputted h, H, or ?, print out available serial commands
        else if (serialMain->incoming[0] == 'h' || serialMain->incoming[0] == 'H' || serialMain->incoming[0] == '?') 
        {
          Serial.println("\n >>>> PID Serial Commands <<<<\n");
          Serial.println(" End commands with '.' for faster responses");
          Serial.println(" Interacting with the PID controller");
          Serial.println("   Pma - call  heaterPID->SetMode(AUTOMATIC)");
          Serial.println("   Pmm - call  heaterPID->SetMode(MANUAL)");
          Serial.println("   Pmg - call  heaterPID->GetMode()");
          Serial.println("   Pslnnn - update glassSetpoint as nn.n degrees");
          Serial.println("   Psennn - update enclosureTemperature->valueSetpoint as nn.n degrees");
          Serial.println("   Ponnn. - manually set heaterValues.outputToDevice (only usual in manualPIDMode mode");
          Serial.println("   Plmnnn. - updated agressive PWM output (high max)"); // e.g. 47 is 047
          Serial.println("   Plnnnn. - updated holding (normal max) PWM");
          Serial.println("   Ptpnnn. - heaterPID->SetTunings(nnn,iii,ddd)");
          Serial.println("   Ptinnn. - heaterPID->SetTunings(ppp,nnn,ddd)");
          Serial.println("   Ptdnnn. - heaterPID->SetTunings(ppp,iii,nnn)");
          Serial.println("   Pdnnnn. - call heaterPID->SetSampleTime(nnnn) & STEINHART::setSampleTime(nnnn)");
          Serial.println("   N:xxxxx - add brief note to message log");
          Serial.println("   L - List column headers");
          Serial.println("   St - Show temperature history in serial monitor");
          //Serial.println("   Se - Show mse history in serial monitor");
          Serial.println("   Sa - Show air temp history in serial monitor");
          Serial.println("   Eaxn - Silence/override error x and ignore for n minutes ");
          Serial.println("   Egxn - update error x gracePeriod to n minutes");
          Serial.println("   >>>>>>>>>>>>>><<<<<<<<<<<<<<<<");
        }
        // If another message is inputted, print to the serial that the message was not recognized
        else 
        {
          Serial.println("   Serial command not recognized. Press 'h' for help");
        }
      }
  }

  void parseSerial::parsePIDCmd() {

    if (serialMain->incoming[0] == 'P') 
    {
      if (serialMain->incoming[1] == 'm') 
      {
        if (serialMain->incoming[2] == 'a') 
        {
          //PIDMode = automaticPIDMode;   
          heaterPID->SetMode(AUTOMATIC);
          
          msgBuffer += "PID set to Automatic"; 
        }
  
        if (serialMain->incoming[2] == 'm') 
        {
          //PIDMode = manualPIDMode;
          heaterPID->SetMode(MANUAL); 
          msgBuffer += " PID set to MANUAL"; 
        } 
        
        if (serialMain->incoming[2] == 'g') 
        {
          //PIDMode = manualPIDMode;
          //heaterPID->SetMode(PIDMode); 
          msgBuffer += " PID mode is ";
          msgBuffer += heaterPID->GetMode();  
        }         
      } 
      else if (serialMain->incoming[1] == 'o') 
      {
        char newOutput[3] = {serialMain->incoming[2],serialMain->incoming[3],serialMain->incoming[4]};
        heaterValues->outputToDevice = atof(newOutput);
        msgBuffer += " heaterValues.outputToDevice now "; 
        msgBuffer += String(heaterValues->outputToDevice);  
      }
      else if (serialMain->incoming[1] == 'e') 
      {
        char newOutput[3] = {serialMain->incoming[2],serialMain->incoming[3],serialMain->incoming[4]};
        heaterValues->errorOutput = atof(newOutput); 
        msgBuffer += " heaterValues->errorOutput now "; 
        msgBuffer += String(heaterValues->errorOutput, 1);
             
      }
      //update temperature set points
      else if (serialMain->incoming[1] == 's') 
      {      
        if (serialMain->incoming[2] == 'l') 
        {
          char newSetpoint[3] = {serialMain->incoming[3],serialMain->incoming[4],serialMain->incoming[5]};
          lidTemperature->setpoint = atof(newSetpoint)/10.0;
          msgBuffer += " lidTemperature->setpoint now ";
          msgBuffer += String(lidTemperature->setpoint,1); 
        }
        else if (serialMain->incoming[2] == 'e') 
        {
          char newSetpoint[3] = {serialMain->incoming[3],serialMain->incoming[4],serialMain->incoming[5]};
          enclosureTemperature->setpoint = atof(newSetpoint)/10.0;
          msgBuffer += " enclosureTemperature->setpoint now ";
          msgBuffer += String(enclosureTemperature->setpoint,1); 
          
          if (enclosureTemperature->value < enclosureTemperature->setpoint) {
            //isAirTemperatureClimbing  = true;
            msgBuffer += " expecting heating to new enclosure temp";
          } else {
            //isAirTemperatureClimbing  = false;
            msgBuffer += " expect cooling to new enclosure temp";
          }
        }
        enclosureTemperature->setpointReached = false;
        enclosureTemperature->setpointNoted = false;
  
      } else if (serialMain->incoming[1] == 'l') 

      {
        if (serialMain->incoming[2] == 'm') 
        {
          char upper[3] = {serialMain->incoming[3],serialMain->incoming[4],serialMain->incoming[5]};
          heaterValues->maxOutputHigh = atof(upper);
          heaterPID->SetOutputLimits(0,heaterValues->maxOutputHigh);
          msgBuffer += " heaterPID->SetOutputLimits(0,";
          msgBuffer += String(heaterValues->maxOutputHigh);
          msgBuffer += ")";     
        }
        else if (serialMain->incoming[2] == 'n') 
        {
          char holding[3] = {serialMain->incoming[3],serialMain->incoming[4],serialMain->incoming[5]};
          float safetyCheck = atof(holding);
          if (safetyCheck<=50.0) {
            heaterValues->maxOutputNormal = atof(holding);
            msgBuffer += " heaterValues.maxOutputNormal now";
            msgBuffer += String(heaterValues->maxOutputNormal, 2);     

          } else {
            heaterValues->maxOutputNormal = 50.0;
            msgBuffer += " max firmware heaterValues->maxOutputNormal = 50.0";
          }
          
        }
      }
      else if (serialMain->incoming[1] == 'd') 
      {
        char newDelay[4] = {serialMain->incoming[2],serialMain->incoming[3],serialMain->incoming[4],serialMain->incoming[5]};
        int nd = atol(newDelay);
        heaterPID->SetSampleTime(nd);
        msgBuffer +=" PID interval now ";
        msgBuffer += String(nd, 2);
      }
      else if (serialMain->incoming[1] == 't') 
      {
        if (serialMain->incoming[2] == 'p') 
        {
          char ppp[3] = {serialMain->incoming[3],serialMain->incoming[4],serialMain->incoming[5]};
          heaterValues->P = 0.0;
          heaterValues->P = atol(ppp);
          msgBuffer += " Setting P = ";
          msgBuffer += String(heaterValues->P, 1);
        }
        else if (serialMain->incoming[2] == 'i') 
        {
          char iii[3] = {serialMain->incoming[3],serialMain->incoming[4],serialMain->incoming[5]};
          heaterValues->I = 0.0;
          heaterValues->I = atol(iii);
          msgBuffer += " Setting I = ";
          msgBuffer += String(heaterValues->I);
        }
        else if (serialMain->incoming[2] == 'd') 
        {
          char ddd[3] = {serialMain->incoming[3],serialMain->incoming[4],serialMain->incoming[5]};
          heaterValues->D = 0.0;
          heaterValues->D = atol(ddd);
          msgBuffer += " Setting D = ";
          msgBuffer += String(heaterValues->D);
        }
        
        heaterPID->SetTunings(heaterValues->P,heaterValues->I,heaterValues->D);
      }
    } 
    else if(serialMain->incoming[0] == 'S' && serialMain->incoming[1] == 't' )
    {
      for(int i = 0; i < lidTemperature->historySize; i++) 
      {
        Serial.print(lidTemperature->history[i]);
        Serial.print(", "); 
      }
      Serial.print(", index = ");
      Serial.println(lidTemperature->index);
    } 
  
  //Do we really need MSE? It now feels like a vestige of testing. 
    /*  
    else if(serialMain->incoming[0] == 'S' && serialMain->incoming[1] == 'e' )
    {
      for(int i = 0; i < lidTemperature->historySize; i++) 
      {
        Serial.print(mseGlassTemperatureHistory[i]);
        Serial.print(", "); 
      }
      Serial.print(", index = ");
      Serial.println(historyArraysIndex);
    }
    */
    else if(serialMain->incoming[0] == 'S' && serialMain->incoming[1] == 'a' )
    {
      for(int i = 0; i < enclosureTemperature->historySize; i++) 
      {
        Serial.print(enclosureTemperature->history[i]);
        Serial.print(", "); 
      }
      Serial.print(", index = ");
      Serial.println(enclosureTemperature->index);
    } 

    //Add a note to the msg buffer
    else if(serialMain->incoming[0] == 'N' && serialMain->incoming[1] == ':' )
    {
      
      msgBuffer += " Note:  ";
      // skip optional space after the colon
      int i = 2;
      if (i < serialMain->length && serialMain->incoming[i] == ' ') i++;
    
      for (; i < serialMain->length; ++i) {
        char c = serialMain->incoming[i];
        if (c == '\r' || c == '\n') break;   // stop at line ending if present
        msgBuffer += c;
      }
      
    } 
    else if(serialMain->incoming[0] == 'E' && serialMain->incoming[1] == 'a' )
    {
         int thisError = serialMain->incoming[2];
         
         errorCodes[thisError].silenceTimer = millis(); 
         errorCodes[thisError].silenced = true;
         if (serialMain->length > 3) {
          int thisMinutes = serialMain->incoming[3]; 
          errorCodes[thisError].silencePeriod = thisMinutes * 60000;
         }
         msgBuffer += " Error ";
         msgBuffer += thisError;    
         msgBuffer += " silenced for ";
         msgBuffer += String(errorCodes[thisError].silencePeriod/60000,1);          //!!!!!!!!!!!!!!!!!!! ERROR CODES VALUES need to b sorted once errorCheck is more finalized
         msgBuffer += " min.";
    } 
    else if(serialMain->incoming[0] == 'E' && serialMain->incoming[1] == 'g' )
    {
//????? small issue reading incoming[2] correctly to integer
//
      int thisError = serialMain->incoming[2];
      //int thisError = atol(cError);
      int minGraceTime = serialMain->incoming[3];
      
      //errorGraceTime = minGraceTime * 60000;
      errorCodes[thisError].gracePeriod = minGraceTime * 60000;
      msgBuffer += " Error ";
      msgBuffer += thisError;      
      msgBuffer += " grace period now ";
      msgBuffer += String(minGraceTime, 0);
      msgBuffer += " min";    
    } 
    else if(serialMain->incoming[0] == 'L')
    {
      Serial.println("\tT(enclosure):\tT(lid):\tT(encl.slope):\tT(lidThermistor.slope):\tT(mse):\tsetPt(lid):\tsetPt(enclosure):\tPWMOut:\tV(in):\tPID:\tError:\tMsg:"); 
    } 
    else 
    {
      Serial.println(" Command not recognized. Press 'h' for index of commands");
    }


  }
