/*
   LaBox Project
   XpressNet part

   @Author : lebelge2
   @Organization : Locoduino.org

   Dernière modif: 17-07-25
*/
//================================================ XPRESSNET MASTER V.3. ===================================================

#include <Arduino.h>
#include "DCC.h"

#ifdef ENABLE_XPRESSNET
//#include "DCC.h"
#include "TrackManager.h"
#include "EXComm.h"
#include "EXCommItems.h"
#include "LaboxModes.h"
#include <XpressNetMaster.h>

#define OffsetAdress   1
#define OffsetPort     0

XpressNetMasterClass XpressNet;
uint8_t dir;
uint8_t rxtxPin, dirPin;
uint8_t F0F4;
uint8_t F5F8;
uint8_t F9F12;
uint8_t F13F20;
uint8_t F21F28;
uint8_t memF0F4;
uint8_t memF5F8;
uint8_t memF9F12;
uint8_t memF13F20;
uint8_t memF21F28;
int l;
int16_t XpressCvValue = -1;
int16_t XpressCvAddress = 0;
uint8_t port;
bool    gate;
bool    OnOff;
byte pos1;
byte pos2;
uint8_t userOp;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
bool StartOK = 0;


XPressNetESP::XPressNetESP(int inRxTxPin, int inDirPin) : EXCommItem("XPressNetESP") {
  rxtxPin = inRxTxPin;
  dirPin = inDirPin;
}
//----------------------------------------------------------------
bool XPressNetESP::begin() {
  XpressNet.setup(Loco128, rxtxPin, dirPin);    //Init XNet Serial, RX/TX-PIN, Send/Receive-PIN
  return true;
}
//--------------------------------------------------------------
bool XPressNetESP::loop() {
  currentTime = millis();
  if ((currentTime - previousTime) > 50) {
    for (int i = 0; i < 20; i++) {
      XpressNet.update();
      delay(5);
    }
  }
  previousTime =  currentTime;
  XpressNet.update();                     //call in every loop
  return true;
}
//----------------------------------------------------------
void notifyXNetPower(uint8_t State) {
  if (State == 0)
    TrackManager::setMainPower(POWERMODE::ON);      // Requête de commande de reprise des opérations
  else if (State == 1)
    TrackManager::setMainPower(POWERMODE::OFF);     // Requête de commande d’arrêt d’urgence
  else if (State == 2)
    TrackManager::setMainPower(POWERMODE::OFF);     // Requête de commande court circuit
  //else                                              // ServiceMode 0x08
  XpressNet.setPower(State);                        // Envoi l'état de la centrale aux périphériques
}
//-------------------------------------
uint8_t getPowerState() {
  Serial.println("Power State");
}
//---------------------- Vitesse 14 crans -----------------------------------
void notifyXNetLocoDrive14(uint16_t Address, uint8_t Speed) {         // 0xE4 0x10
  dir = bitRead(Speed, 7);
  Speed = Speed & 0x0F;
  if (Speed >= 15)
    Speed = 127;
  else if (Speed <= 2)
    Speed = 0;
  else
    Speed = (Speed - 1) * 9 + 1;
  DCC::setThrottle(Address, Speed & 0x7F, dir);
}
//------------------------ Vitesse 28 crans ---------------------------------------
void notifyXNetLocoDrive28(uint16_t Address, uint8_t Speed) {          // 0xE4 0x12
  dir = bitRead(Speed, 7);
  Speed = Speed & 0x1F;
  if (Speed >= 31)
    Speed = 127;
  else if (Speed <= 3)
    Speed = 0;                      // 2 and 3 are IDLE and EMGR
  else
    Speed = (Speed - 3) * 9 / 2 + 1;
  DCC::setThrottle(Address, Speed & 0x7F, dir);
}
//------------------------ Vitesse 128 crans -----------------------------------------
void notifyXNetLocoDrive128(uint16_t Address, uint8_t Speed) {        // 0xE4 0x13
  dir = bitRead(Speed, 7);
  DCC::setThrottle(Address, Speed & 0x7F, dir);
}
//--------------------------------------------------------------
void notifyXNetLocoFunc1(uint16_t Address, uint8_t Func1) {           // 0xE4 0x20
  F0F4 = (Func1 & 0b00010000) >> 4 | (Func1 & 0b00001111) << 1 ;      // F0 F3 F2 F1 F4  moveBit4to0  F4 F3 F2 F1 F0
  for (int i = 0; i < 5; i++) {
    if (bitRead(memF0F4, i) !=  bitRead(F0F4, i))
      DCC::setFn(Address, i, bitRead(F0F4, i));
  }
  memF0F4 = F0F4;
}
//--------------------------------------------------------------
void notifyXNetLocoFunc2(uint16_t Address, uint8_t Func2) {          // 0xE4 0x21    F5, F6, F7, F8
  F5F8 = Func2;
  for (int i = 0; i < 4; i++) {
    if (bitRead(memF5F8, i) !=  bitRead(F5F8, i))
      DCC::setFn(Address, i + 5, bitRead(F5F8, i));
  }
  memF5F8 = F5F8;
}
//--------------------------------------------------------------
void notifyXNetLocoFunc3(uint16_t Address, uint8_t Func3) {         // 0xE4 0x22    F9, F10, F11, F12
  F9F12 = Func3;
  for (int i = 0; i < 4; i++) {
    if (bitRead(memF9F12, i) !=  bitRead(F9F12, i))
      DCC::setFn(Address, i + 5, bitRead(F9F12, i));
  }
  memF9F12 = F9F12;
}
//--------------------------------------------------------------
void notifyXNetLocoFunc4(uint16_t Address, uint8_t Func4) {        // 0xE4 0x23    F13 --> F20
  F13F20 = Func4;
  for (int i = 0; i < 8; i++) {
    if (bitRead(memF13F20, i) !=  bitRead(F13F20, i))
      DCC::setFn(Address, i + 5, bitRead(F13F20, i));
  }
  memF13F20 = F13F20;
}
//--------------------------------------------------------------
void notifyXNetLocoFunc5(uint16_t Address, uint8_t Func5) {        // 0xE4 0x28    F21 --> F28
  F21F28 = Func5;
  for (int i = 0; i < 5; i++) {
    if (bitRead(memF21F28, i) !=  bitRead(F21F28, i))
      DCC::setFn(Address, i + 5, bitRead(F21F28, i));
  }
  memF21F28 = F21F28;
}
//--------------------------------------------------------------------------
void notifyXNetTrntInfo(uint8_t UserOps, uint16_t Address, uint8_t data) { //Demande informations sur le décodeur d'accessoires
  userOp = UserOps;
  Address = (Address >> 2) + OffsetAdress;
  XPressNetESP::XNetTrntInfo(Address, data);
}
//---------------------------------------------------------------------
void notifyXNetTrnt(uint16_t Address, uint8_t data) {   // 0x52,AAAAAAAA,1000dBBD,XOR
  Address = (Address >> 2) + OffsetAdress;              // 00AAAAAA
  port = ((data & 0x06) >> 1) + OffsetPort;             // 000000BB
  gate = bitRead(data, 0);                              // 0000000D
  OnOff =  bitRead(data, 3);                            // 0000d000
  DCC::setAccessory(Address, port, gate, OnOff);
  if (DIAG_XPNET)
    DIAG(F("Adress: %d Port: %d Gate: %d OnOff: %d"), Address, port, gate, OnOff);
  XPressNetESP::XNetTrntInfo(Address, data);
}
//---------------------------------------------------------------------
void XPressNetESP::XNetTrntInfo(uint16_t Address, uint8_t data) {
  pos1 = bitRead(data, 2);
  pos2 = data & B11;
  if (pos2 == 0) data = 0x05;
  if (pos2 > 0) data = 0x06;
  if (pos2 == 3) data = 0x0A;
  if (pos1 == 1)
    bitSet(data, 4);
  XpressNet.SetTrntStatus(userOp, Address - OffsetAdress, data);   // Feedback (0x42,MOD,DATA,...,XOR) Accessory decoder information response
}
//----------------------------------------------------------
void  notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address) {
  //Serial.println("GL");
  if (StartOK == 0) {
    StartOK = 1;
    TrackManager::setMainPower(POWERMODE::ON);
  }
}
//-----------------------------------------------------------------
void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address) {
 // Serial.println("GF");
  if (StartOK == 0) {
    StartOK = 1;
    TrackManager::setMainPower(POWERMODE::ON);
  }
}
//------------------------------------------------------------------
void notifyXNetgiveLocoMM(uint8_t UserOps, uint16_t Address) {
  //Serial.println("GM");
  if (StartOK == 0) {
    StartOK = 1;
    TrackManager::setMainPower(POWERMODE::ON);
  }
}
#endif
