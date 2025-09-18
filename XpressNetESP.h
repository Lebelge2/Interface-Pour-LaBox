/*
   LaBox Project
   XpressNet part

   @Author : lebelge2
   @Organization : Locoduino.org

   Dernière modif: 18-09-25
*/

#ifndef __XPressNet_h
#define __XPressNet_h

#include "DCC.h"
#include "EXComm.h"

#ifdef ENABLE_XPRESSNET

const bool DIAG_XPNET = false;  // Sans Debug
//const bool DIAG_XPNET = true;   // Avec Debug
void getfonction(uint8_t adr);

class XPressNetESP: public EXCommItem {
  public:
    XPressNetESP(int inRxTxPin, int inDirPin);
    bool begin() override;
    bool loop() override;
    static void XNetTrntInfo(uint16_t Address, uint8_t data);
};
#endif
#endif
