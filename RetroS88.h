/**********************************************************************

  S88.h
  COPYRIGHT (c) 2013-2020 Xavier Bouillard & Philippe Chavatte
  Last update 2021/04/16

**********************************************************************/

#include "EXComm.h"

#ifdef ENABLE_RETROS88
#ifndef RetroS88_h
#define RetroS88_h

#include "Arduino.h"

#define USE_CDMRAIL               // to be uncommented if needed

#define S88_VERSION     "VERSION S88 library:   1.4.0"
#define S88_SAMPLE_PERIOD 2          // ms, to be adjusted for S88 clock


class RetroS88: public EXCommItem {
  public:
    RetroS88(int inRstPin, int inLoadPin, int inClockPin, int inDataPin);
    bool begin() override;
    bool loop() override;
    static long int S88sampleTime;
    static byte S88_Cpt;
    static boolean checkTime();
    static void check();
    static int count();
    static void load();
    static void store();

}; // S88

#endif  // S88_h 
#endif  // 
