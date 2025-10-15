/*
   LaBox Project
   XpressNet part

   @Author : lebelge2
   @Organization : Locoduino.org

   Dernière modif: 18-09-25
*/
//================================================ XPRESSNET MASTER V.3.1 ===================================================

#include <Arduino.h>
#include "DCC.h"

#ifdef ENABLE_XPRESSNET

#include "TrackManager.h"
#include "EXComm.h"
#include "LaboxModes.h"
#include "XpressNetESP.h"
#include <XpressNetMaster.h>
#include "hmi.h"
#define OffsetAdress   1
#define OffsetPort     0

XpressNetMasterClass XpressNet;

uint8_t rxtxPin, dirPin;
uint8_t MemF0F4;
uint8_t MemF5F8;
uint8_t MemF9F12;
uint8_t MemF13F20;
uint8_t MemF21F28;
uint8_t Gr1;         // Groupes fonctions
uint8_t Gr2;
uint8_t Gr3;
uint8_t Gr4;
uint8_t Gr5;
int16_t XpressCvValue = -1;
int16_t XpressCvAddress = 0;
uint8_t userOp;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
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
//--------------------------------------------------------------------------------------
XPressNetESP::XPressNetESP(int inRxTxPin, int inDirPin) : EXCommItem("XPressNetESP") {
  rxtxPin = inRxTxPin;
  dirPin = inDirPin;
  this->MainTrackEnabled = true;
  this->ProgTrackEnabled = true;
  this->AlwaysLoop = true;
}
//--------------------------------------------------------------
void vTaskPeriodic( void *pvParameters ){
  const char *pcTaskName = "Task periodique";
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for( ;; ) {
    XpressNet.update();
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 5 ) ); // toutes les 5 ms
  }
}
//----------------------------------------------------------------
bool XPressNetESP::begin() {
  XpressNet.setup(Loco128, rxtxPin, dirPin);                  //Init XNet Serial, RX/TX-PIN, Send/Receive-PIN
  xTaskCreate(vTaskPeriodic, "Envoi octet d'appel", 10000, NULL, 1, NULL);    // Task
  XpressNet.setPower(1);     // Signal à LaBox:  Raill OFF
   return true;
}
//---------------------------------------------------------------
bool XPressNetESP::loop() {
 //   XpressNet.update();                     //call in every loop
  return true;
}
//----------------------------------------------------------
void XpressCvValueCallback(int16_t inValue) {
  if (inValue >= 0) {
    stateCV = StateCV::Reading_OK;
  }
  else
    stateCV = StateCV::Reading_FAIL;
  Serial.print("Adr ");
  Serial.print(XpressCvAddress);
  Serial.print("  CV lu ");
  Serial.println(inValue);
  XpressNet.setCVReadValue(XpressCvAddress - 1, inValue);
}
//----------------------------------------------------------------
void XpressCvWriteValueCallback(int16_t inValue) {
  if (inValue >= 0)
    stateCV = StateCV::Written_OK;
  else
    stateCV = StateCV::Written_FAIL;
  Serial.println("Relire cv");
  // notifyXNetDirectReadCV( XpressCvAddress);
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
//---------------------- Vitesse 14 crans -----------------------------------
void notifyXNetLocoDrive14(uint16_t Address, uint8_t Speed) {         // 0xE4 0x10
  bool dir = bitRead(Speed, 7);
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
  bool dir = bitRead(Speed, 7);
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
   //Serial.println("0xE4 0x13");
  bool dir = bitRead(Speed, 7);
  DCC::setThrottle(Address, Speed & 0x7F, dir);
}
//--------------------------------------------------------------
void notifyXNetLocoFunc1(uint16_t Address, uint8_t Func1) {           // 0xE4 0x20    Fonctions groupe 1     F0 F4 F3 F2 F1
  //Serial.print("Gr1:"); Serial.println(Func1, BIN);
  uint8_t F0F4 = (Func1 & 0b00010000) >> 4 | (Func1 & 0b00001111) << 1 ;      // F0 F4 F3 F2 F1   moveBit4to0  F4 F3 F2 F1 F0
 //uint8_t F0F4 = Func1;
  Serial.print("F1:"); Serial.println(F0F4, BIN);
  for (int i = 0; i < 5; i++) {
    if (bitRead(MemF0F4, i) !=  bitRead(F0F4, i))
      DCC::setFn(Address, i, bitRead(F0F4, i));
  }
  MemF0F4 = F0F4;
}
//--------------------------------------------------------------
void notifyXNetLocoFunc2(uint16_t Address, uint8_t Func2) {          // 0xE4 0x21   Fonctions groupe 2   F5, F6, F7, F8
  Serial.print("Gr2:"); Serial.println(Func2, BIN);
  uint8_t F5F8 = Func2;
  for (int i = 0; i < 4; i++) {
    if (bitRead(MemF5F8, i) !=  bitRead(F5F8, i))
      DCC::setFn(Address, i + 5, bitRead(F5F8, i));
  }
  MemF5F8 = F5F8;
}
//--------------------------------------------------------------
void notifyXNetLocoFunc3(uint16_t Address, uint8_t Func3) {         // 0xE4 0x22   Fonctions groupe 3   F9, F10, F11, F12
  Serial.print("Gr3 ");  Serial.println(Func3, BIN);
  uint8_t F9F12 = Func3;
  for (int i = 0; i < 4; i++) {
    if (bitRead(MemF9F12, i) !=  bitRead(F9F12, i))
      DCC::setFn(Address, i + 9, bitRead(F9F12, i));
  }
  MemF9F12 = F9F12;
  
}
//--------------------------------------------------------------
void notifyXNetLocoFunc4(uint16_t Address, uint8_t Func4) {        // 0xE4 0x23   Fonctions groupe 4   F13 --> F20
  Serial.print("Gr4 "); Serial.println(Func4, BIN);
  uint8_t F13F20 = Func4;
  for (int i = 0; i < 8; i++) {
    if (bitRead(MemF13F20, i) !=  bitRead(F13F20, i))
      DCC::setFn(Address, i + 13, bitRead(F13F20, i));
  }
  MemF13F20 = F13F20;
}
//--------------------------------------------------------------
void notifyXNetLocoFunc5(uint16_t Address, uint8_t Func5) {        // 0xE4 0x28  Fonctions groupe 5   F21 --> F28
  Serial.print("Gr5 "); Serial.println(Func5, BIN);
  uint8_t F21F28 = Func5;
  for (int i = 0; i < 5; i++) {
    if (bitRead(MemF21F28, i) !=  bitRead(F21F28, i))
      DCC::setFn(Address, i + 21, bitRead(F21F28, i));
  }
  MemF21F28 = F21F28;
}
//--------------------------------------------------------------------------
void  notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address) {   // 0xE3 0x00    Requête d’information d'une locomotive
 // Serial.println("0xE3 0x00");
  uint8_t Speed = DCC::getThrottleSpeed(Address);  
  getfonction(Address);  
  XpressNet.SetLocoInfo(UserOps, Loco128, Speed, Gr1, Gr2);     // 0xE4
  //digitalWrite(15,1);
 // delayMicroseconds(100);
  //digitalWrite(15,0);
}
//---------------------------------------------------------------
void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address) {    // 0xE3 0x09 Requête d'état de fonctions  F13 à F28 anfordern
  Serial.println("0xE3 0x09"); 
   getfonction(Address); 
   XpressNet.SetFktStatus(UserOps, Gr4, Gr5);                       // 0xE3 0x52
}
//--------------------------------------------------------------------
void notifyXNetgiveLocoMM(uint8_t UserOps, uint16_t Address) {       // 0xE3 0xF0  Requête d’information d'une locomotive (Multimaus)
  Serial.println("0xE3 0xF0");
  uint8_t Speed = DCC::getThrottleSpeed(Address);
  bool dir = DCC::getThrottleDirection(Address);
  if (dir)
    Speed += 0x80;
   getfonction(Address);  
   XpressNet.SetLocoInfoMM(UserOps, Loco128, Speed, Gr1, Gr2, Gr3, Gr4); // 0xE7
}
//--------------------------------------------------------------------
void notifyXNetTrntInfo(uint8_t UserOps, uint16_t Address, uint8_t data) { // 0x42 Demande informations sur le décodeur d'accessoires
  userOp = UserOps;
  Address = (Address >> 2) + OffsetAdress;
  XPressNetESP::XNetTrntInfo(Address, data);
}
//---------------------------------------------------------------------
void notifyXNetTrnt(uint16_t Address, uint8_t data) {        // 0x52,AAAAAAAA,1000dBBD,XOR
  Address = (Address >> 2) + OffsetAdress;                   // 00AAAAAA
  uint8_t port = ((data & 0x06) >> 1) + OffsetPort;          // 000000BB
  bool gate = bitRead(data, 0);                              // 0000000D
  bool OnOff =  bitRead(data, 3);                            // 0000d000
  DCC::setAccessory(Address, port, gate, OnOff);
  if (DIAG_XPNET)
    DIAG(F("Adress: %d Port: %d Gate: %d OnOff: %d"), Address, port, gate, OnOff);
  XPressNetESP::XNetTrntInfo(Address, data);
}
//---------------------------------------------------------------------
void XPressNetESP::XNetTrntInfo(uint16_t Address, uint8_t data) {
  byte pos1 = bitRead(data, 2);
  byte pos2 = data & B11;
  if (pos2 == 0) data = 0x05;
  if (pos2 > 0) data = 0x06;
  if (pos2 == 3) data = 0x0A;
  if (pos1 == 1)
    bitSet(data, 4);
  XpressNet.SetTrntStatus(userOp, Address - 1, data);   // Feedback (0x42,MOD,DATA,...,XOR) Accessory decoder information response
}
//-------------------------- Direct ----------------------------------
void notifyXNetDirectCV(uint16_t CV, uint8_t data) {           // 0x23 Ecriture CV
  //Serial.println("notifyXNetDirectCV");
  void (*ptr)(int16_t) = &XpressCvWriteValueCallback;
  DCC::writeCVByte(CV, data, ptr);
  stateCV = Writing;
}
//--------------------------- Direct ---------------------------------
void notifyXNetDirectReadCV(uint16_t cvAdr) {                   // 0x22 Lecture CV
  cvAdr++;
  //Serial.print("Cv à lire: "); Serial.println(cvAdr);
  XpressCvAddress = cvAdr ;
  void (*ptr)(int16_t) = &XpressCvValueCallback;
  DCC::readCV(XpressCvAddress, ptr);
  stateCV = Reading;
}
//---------------------------- POM -----------------------------------
void notifyXNetPOMwriteByte (uint16_t Adr, uint16_t CV, uint8_t data) {            // 0xE6
  DIAG(F("POM write Byte Adr: %d CV: %d Data: %d OnOff: %d"), Adr, CV, data);
  DCC::writeCVByteMain(Adr, CV, data);
}
//---------------------------- POM -------------------------------------
void notifyXNetPOMwriteBit (uint16_t Adr, uint16_t CV, uint8_t data) {      // 0xE6
  DIAG(F("POM write Bit Adr: %d CV: %d Data: %d OnOff: %d"), Adr, CV, data);
  //DCC::writeCVBitMain(Adr, CV, data);
}
//--------------------------------------------------------------------
void getfonction(uint8_t adr) {
  uint32_t FonctionMap = DCC::getFunctionMap(adr);
  uint8_t moveBit0to4 = FonctionMap & 0x1F ;     // 5 bits
  Gr1 = (moveBit0to4 & 0b00000001) << 4 | (moveBit0to4 & 0b00011111) >> 1 ;   // F4 F3 F2 F1 F0    moveBit0to4  F0 F4 F3 F2 F1
  //Gr1 = (FonctionMap & 0x1F) ;                 // Grouppe 1: 0 0 0 F0 F4 F3 F2 F1
  Gr2 = (FonctionMap & 0xF00) >> 8;           // Grouppe 2: 0 0 0 0 F8 F7 F6 F5
  Gr3 = (FonctionMap & 0x1F0000) >> 16;        // Grouppe 3: 0 0 0 0 F12 F11 F10 F9
  Gr4 = (FonctionMap & 0x1FE000) >> 21;        // Grouppe 4: F20 F19 F18 F17 F16 F15 F14 F13
  Gr5 = (FonctionMap & 0x1FE000) >> 21;        // Grouppe 5: F28 F27 F26 F25 F24 F23 F22 F21
 // Serial.print("Groupe 1 ");Serial.println(Gr1,BIN);
}
#endif

