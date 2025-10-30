/*
   LaBox Project
   XpressNet part

   @Author : lebelge2
   @Organization : Locoduino.org
*/
//================================================ XPRESSNET MASTER V.2.1 ===================================================
// Dernière modif: 22-10-25

#ifndef __XPressNet_h
#define __XPressNet_h

#include "DCC.h"
#include "EXComm.h"

#ifdef ENABLE_XPRESSNET


#define csNormal 0x00          // Normal Operation Resumed ist eingeschaltet
#define csEmergencyStop 0x01   // Der Nothalt ist eingeschaltet
#define csTrackVoltageOff 0x02 // Die Gleisspannung ist abgeschaltet
#define csShortCircuit 0x04    // Kurzschluss
#define csServiceMode 0x08     // Der Programmiermodus ist aktiv - Service Mode

const bool DIAG_XPNET = false;
//const bool DIAG_XPNET = true;

class XPressNetESP: public EXCommItem {
  public:
    XPressNetESP(int inRxTxPin,  int inDirPin);
    bool begin() override;
    bool loop() override;
    static void Update();
    static void Decodage();
    static void Tx9(int dTx);
    static void Rx9();
    static void F0a28();
    static void Test();
    static void Marqueur(int a);
    static void XNetsend(unsigned char *dataString, byte byteCount);
    static void getXOR(unsigned char *data, byte length);
    static void getfonction(uint8_t adr);
    static void Fonction(int a, int b);
    static void notifyXNetPower(uint8_t State);
};

enum StateCV {
  Ready,            // 0
  Reading,          // 1
  Reading_OK,       // 2
  Reading_FAIL,     // 3
  Writing,          // 4
  Written_OK,       // 5
  Written_FAIL      // 6
};
static StateCV  stateCV;


#endif
#endif
