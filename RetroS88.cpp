/**********************************************************************

  S88.cpp
  COPYRIGHT (c) 2013-2021 Xavier Bouillard & Philippe Chavatte
  Last update 2021/04/16
  http://lormedy.free.fr/S88.html
  http://lormedy.free.fr/DCCpp.html
  http://fadiese.hd.free.fr/cms/index.php?page=dcc

***********************************************************************

  DCCpp_S88 BASE STATION supports dual S88 retrosignalisation.
  This S88 Master simultanueously controls 2 S88 buses of 256 bits maxi each (32x8).
  These buses are named "Left bus" reading DataL and "Right bus" reading DataR.
  They are provided to ease operations with long layout used by clubs.
  This sensor data collection will be sent back as a binary or hexadecimal string.
  This feedback will occure as soon as any sensor change is detected or upon request of the control software.
  It will be necessary to initialize the S88 data collection indicating
  how many cards are connected (a card is a group of 8 detectors/sensors) using
  DATAFORMAT: Interface with CDT3x, WDD, TCOWiFi, DMC, CDM-Rail, JMRI and Rocrail softwares
					 DataFormat = 0 for binary digit answer                        ex. <y 0001001011000000........> 
					 DataFormat = 1 for hexadecimal encoded answer for CDM-Rail    ex. <y 00000004>
					 DataFormat = 2 for pure hexadecimal encoded answer            ex. <y 12C0..>
					 DataFormat = 3 for SENSOR like style use by JMRI & Rocrail    ex. <Q 31><q 31>
	any sensor change will send new data to the PC

  The 2 data collections have the same length : total Nb_S88_Modules = 2..64 (step of 2).
  They are concatened into 1 single buffer and send to the PC, DataL followed by DataR.

  - the following variation of the "Q" command:
  <Q>:  sets Nb_S88_Modules to read = 2..64 (step of 2)
									   DataFormat = 2 for SENSOR like style use by JMRI & Rocrail
		returns: <Q ID> if sensor Id is active (1), <q ID> if sensor ID is inactive (0)
				 any sensor change will send new data to the PC

  Sensor list from 1 to 512 are reserved for S88 bus. Extra sensors should use ID > 512 up to 32768.

******************************** IMPORTANT *******************************************/
/*
   CDM-Rail : Pour utiliser CDM-Rail avec le bus S88, dé-commenter la ligne 17 de S88.h ****************
   #define USE_CDMRAIL // uncomment to use S88 with CDM-Rail
*/

#include "EXComm.h"
#ifdef ENABLE_RETROS88
#include "RetroS88.h"
#include "EXCommItems.h"

uint8_t  N = 8;              // S88 number, 2 to 64   X8
uint8_t  DataFormat = 3;     // Output DataFormat    DataFormat = 0 for binary output in ASCII,      ex. <y 0001001011000000........> 
//                                                   DataFormat = 1 for hexadecimal output in ASCII  ex. <y 00000000>
//                                                   DataFormat = 2 for pure hexa,                   ex. <y 12C0..>
//                                                   DataFormat = 3 for JMRI & Rocrail output        ex. <Q 31><q 31>

uint8_t  N_size = 8;         // S88 byte size as a group of 8 sensors
uint8_t  Old_N = 0;          // S88 byte number, default = 0
uint16_t Nr = 0;             // S88 Bits quantity = N*8
uint8_t  Mode = 0;           // Output Format for extra software TBD
uint8_t  Old_DataFormat = 0; // Output DataFormat 0=binary 1=hexa
String   Old_Occ;            // S88 detector previous status
String   OccL;               // S88 detector building status
//String   OccR;               // S88 detector building status
String   S88Status;          // S88 sensor status response
uint8_t  RetroS88::S88_Cpt = 0;   // State machine position
uint8_t  dataencode = 0;
long int RetroS88::S88sampleTime = 0;
int sampleRate = 4;
String answerString = "";

//static byte SignalEnablePinProg;  // PWM : *_SIGNAL_ENABLE_PIN_PROG
//-------------------------------------------------------------------------
int S88_RESET_PIN, S88_LOAD_PS_PIN, S88_CLOCK_PIN, S88_DATAL_PIN;
RetroS88::RetroS88(int inRstPin, int inLoadPin, int inClockPin, int inDataPin) : EXCommItem("RetroS88"){
  S88_RESET_PIN = inRstPin;
  S88_LOAD_PS_PIN = inLoadPin;
  S88_CLOCK_PIN = inClockPin;
  S88_DATAL_PIN = inDataPin;
}

bool RetroS88::begin() {
  pinMode(S88_LOAD_PS_PIN, OUTPUT);        
  pinMode(S88_RESET_PIN, OUTPUT);          
  pinMode(S88_CLOCK_PIN, OUTPUT);           
  pinMode(S88_DATAL_PIN, INPUT_PULLDOWN);   
  digitalWrite(S88_LOAD_PS_PIN, LOW);
  digitalWrite(S88_CLOCK_PIN, LOW);
  digitalWrite(S88_RESET_PIN, LOW);
  return true;
}

//----------------------------------------------
bool RetroS88::loop() {
  if (millis() - S88sampleTime < sampleRate)             // no need to check S88 yet
    return (false);
  S88sampleTime = millis();                        
  RetroS88::check();
  return (true);
} 
//--------------------------------------------
void RetroS88::check() {
  switch (++S88_Cpt) {                        // S88 scan 512-bit >= 77ms / 13Hz
    case 1:                                   // LOAD and RESET
      digitalWrite(S88_CLOCK_PIN, LOW);       // Clock low
      digitalWrite(S88_RESET_PIN, LOW);       // Reset low
      digitalWrite(S88_LOAD_PS_PIN, HIGH);    // Load high 1ms min
      Nr = N * N_size;                        // total bit number to read
      OccL = "";
      //OccR = "";
      digitalWrite(S88_CLOCK_PIN, HIGH);      // Clock rising ~50µs
      delayMicroseconds(10);
      delayMicroseconds(10);
      delayMicroseconds(10);
      delayMicroseconds(10);
      delayMicroseconds(10);
      digitalWrite(S88_CLOCK_PIN, LOW);       // Clock falling
      delayMicroseconds(10);
      delayMicroseconds(10);
      digitalWrite(S88_RESET_PIN, HIGH);      // Reset high 1ms min
      delayMicroseconds(10);
      delayMicroseconds(10);
      digitalWrite(S88_LOAD_PS_PIN, LOW);     // Load low
      delay(1);
      digitalWrite(S88_RESET_PIN, LOW);       // Reset low
      break;

    case 2:                                          // READ DATA stored in the last hundred of millis
      for (byte i = 0; i < 8; i++) {                 // read 8 sensors in a row
        OccL += String(digitalRead(S88_DATAL_PIN));  // Read data, left side
      //  if (S88_DATAR_PIN != UNDEFINED_PIN)
      //    OccR += String(digitalRead(S88_DATAR_PIN)); // Read data, right side
        digitalWrite(S88_CLOCK_PIN, HIGH);           // Clock rising ~10µs
        delayMicroseconds(10);                       // Non interruptible / 10µs !
        digitalWrite(S88_CLOCK_PIN, HIGH);           // Clock rising ~20µs
        delayMicroseconds(10);
        digitalWrite(S88_CLOCK_PIN, HIGH);           // Clock rising ~30µs
        delayMicroseconds(10);
        digitalWrite(S88_CLOCK_PIN, HIGH);           // Clock rising ~40µs
        delayMicroseconds(10);
        digitalWrite(S88_CLOCK_PIN, HIGH);           // Clock rising ~50µs
        delayMicroseconds(10);
        digitalWrite(S88_CLOCK_PIN, LOW);            // Clock falling
        delayMicroseconds(10);
        digitalWrite(S88_CLOCK_PIN, LOW);            // Clock falling
        delayMicroseconds(10);
        digitalWrite(S88_CLOCK_PIN, LOW);            // Clock falling
        delayMicroseconds(10);
        digitalWrite(S88_CLOCK_PIN, LOW);            // Clock falling
        delayMicroseconds(10);
        digitalWrite(S88_CLOCK_PIN, LOW);            // Clock falling ~50µs
        delayMicroseconds(10);
        Nr--;
      }
      if (Nr != 0) {                                 // buffers filled ?
        S88_Cpt = 1;                                 // loop to case 2, ~130µs
      }
      else {                                         // S88 string is ready, need to format it.
       // if (S88_DATAR_PIN != UNDEFINED_PIN) OccL += OccR;     // concatenate 2 bus together
        if ((Old_N != N) || (Old_DataFormat != DataFormat) || (Old_Occ != OccL)) {             // on any change: send S88 data
          if (DataFormat < 3) S88Status = "<y ";     // start of feedback (CDT3x, WDD or CDM-Rail)
          if (DataFormat == 0) {                     // binary in ASCII
            S88Status += OccL;
          }
          else if ((DataFormat == 1) || (DataFormat == 2)) {  // hexa in ASCII or pure hexa
            dataencode = 4 * DataFormat;
            for (unsigned int i = 0; i < OccL.length(); i = i + dataencode) {
              String tmp = OccL.substring(i, i + dataencode);
              int tmpint = 0;
              for (int ii = 0; ii < dataencode; ii++) {
#ifdef USE_CDMRAIL
                tmpint = tmpint | (tmp[ii] - '0') << ii;                    // lsb first
#else
                tmpint = tmpint | (tmp[ii] - '0') << (dataencode - 1 - ii);  // msb first
#endif
              }
              if (tmpint < 10) S88Status += tmpint;
              else S88Status += (char)(tmpint - 10 + 'A');
            }
          }
          else if (DataFormat == 3) {                // JMRI, Rocrail or SENSOR style
            for (unsigned int s88index = 0; s88index < OccL.length(); s88index++) {
              String tmp = OccL.substring(s88index, s88index + 1);
              String Old_tmp = Old_Occ.substring(s88index, s88index + 1);
              if (tmp[0] != Old_tmp[0]) {
                answerString =  (tmp[0] == '0') ? "<q " : "<Q ";
                answerString += String(s88index + 1) + ">";
                Serial.print(answerString);
                answerString = "";
              }
            }
          }
          if (DataFormat < 3) {
            S88Status += ">";                        // end of feedback (CDT3x or CDM-Rail)
            Serial.print(S88Status);
          }
          Old_Occ = OccL;      // save data
          Old_N = N;
          Old_DataFormat = DataFormat;
        }
        S88_Cpt = 0;           // reset to case 1
      }
      break;
    default:
      S88_Cpt = 0;             // reset to case 1
      break;
  }   // end of switch (S88_Cpt)
}     // end of S88::check

#endif  // USE_S88
