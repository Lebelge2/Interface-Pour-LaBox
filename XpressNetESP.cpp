/*
   LaBox Project
   XpressNet part

   @Author : lebelge2@yahoo.fr
   @Organization : Locoduino.org
*/
//=========================================== XpressNetESP V.3 =====================================================

// Dernière modif: 31-10-25

#include <Arduino.h>
#include <driver/uart.h>
#include "DCC.h"

#ifdef ENABLE_XPRESSNET
#include "XpressNetESP.h"
#include "TrackManager.h"
#include "DCCWaveform.h"
#include "DCCEXParser.h"
#include "hmi.h"
#include "EXComm.h"
#include "EXCommItems.h"
#include "LaboxModes.h"

int OffsetAdress = 1;
int OffsetPort = 0;

// Adresses d'appel périphériques:                p + 0x40 + 000x xxxx
uint8_t  UserOps[35] = {68, 65, 66, 195, 68, 197, 198, 71, 75, 72, 201, 202, 75, 204, 77, 78, 210, 207, 80, 209, 210, 83, 212, 85, 89, 86, 215, 216, 89, 90, 219, 92, 221, 222, 95};
// Adresses de réponses aux périphériques:        p + 0x60 + 000x xxxx
uint8_t  DirectedOps[35] = {228, 225, 226, 99, 228, 101, 102, 231, 235, 232, 105, 106, 235, 108, 237, 238, 114, 111, 240, 113, 114, 143, 116, 245, 249, 246, 119, 120, 249, 250, 123, 252, 125, 126, 255};

byte BufXpress[16];                                           // Buffer reception datas XpressNet

unsigned char NoOp[3] = {0x61, 0x82, 0xE3};
unsigned char TrackPowerOp[4] = {0x60, 0x61, 0x00, 0x61};       // Broadcast Track power OFF
unsigned char NormalOp[4] = {0x60,  0x61, 0x01, 0x60};          // Broadcast Normal operation

int Railpower;
unsigned long currentxpTime;
unsigned long previousxpTime;
int Bi;                                // Buffer Index
int Ti;
int  t;                          // for
int xpSpeed;
bool dir;
bool Bit9;
int Xor;                               // Xor
int xpAdr;
uint8_t Gr1, Gr2, Gr3, Gr4, Gr5;       // Groupes fonctions
uint8_t fonction;
unsigned long xpRegFonc;
int rxtxPin, dirPin;
int Logic = 12;
unsigned int xpTime;
volatile unsigned int xpData[40];
volatile bool xpValBit[40];
int NbrBit;
int PosBit;
unsigned int valTemp;
bool OpCode;
int16_t xpCvValue = -1;
int16_t xpCvAddress = 0;


//--------------------------------------------------------------------------------
XPressNetESP::XPressNetESP(int inrxtxPin, int inDirPin) : EXCommItem("XPressNetESP") {
  rxtxPin = inrxtxPin;
  dirPin = inDirPin;
  this->MainTrackEnabled = true;
  this->ProgTrackEnabled = true;
  this->AlwaysLoop = true;
}
//-----------------------------------------------------------------------------
void IRAM_ATTR xpValBit_ISR() {               // Interruption Rx
  //XPressNetESP::Marqueur(1);                 // Pour analyseur logique
  currentxpTime = micros();
  if (digitalRead(rxtxPin))
    xpValBit[t] = false;
  else
    xpValBit[t]  = true;
  xpTime = currentxpTime - previousxpTime;
  previousxpTime =  currentxpTime;
  xpData[t] = (xpTime + 8) / 16;                // Arrondi la division à l'unité
  t++;
}
//------------------------------------------------------------------------------------------
bool XPressNetESP::begin() {
  pinMode(dirPin, OUTPUT);
  pinMode(Logic, OUTPUT);
  attachInterrupt(rxtxPin, xpValBit_ISR, CHANGE);
  DIAG(F("[XPRESSNET]  RxTx:%d  Dir:%d"), rxtxPin, dirPin);
  Railpower = csEmergencyStop;
  return true;
}
//---------------------------------------------------------------------------------
bool XPressNetESP::loop() {
  currentxpTime = micros();
  if (currentxpTime < (previousxpTime + 3000))        // 3ms minimum
    return false;
  previousxpTime = currentxpTime;
  XPressNetESP::Update();
  return true;
}
//-------------------------------------------------------------------
void XPressNetESP::Update() {
  detachInterrupt(rxtxPin);
  Ti++;                                           // Adresse périphérique suivante
  if (Ti > 34)  Ti = 0;                           // Plage d'adresse
  OpCode = true;                                  // 8 bit sans Stop
  pinMode(rxtxPin, OUTPUT);                       // Pin RxTx en sortie
  uint8_t callByte[1] = {UserOps[Ti]};            // Envoi adresse aux périphériques (A = adresse, P = Parité.   P10A AAAA)
  XPressNetESP::XNetsend(callByte, 1);
  pinMode(rxtxPin, INPUT_PULLUP);                 // Pin RxTx en entrée
  OpCode = false;
  attachInterrupt(rxtxPin, xpValBit_ISR, CHANGE);
  t = 0;
  for (int n = 0; n < 80; n++) {                      // Le périphérique a une fenêtre de 110 µs pour répondre.
    if (digitalRead(rxtxPin) == LOW) goto A;        // Attend réponse du périphérique (Start Bit, niveau bas pin rx)
    delayMicroseconds(1);
  }
  //Marqueur(1);                                   // Pour analyseur logique
  return ;                                         // Temps dépassé, retour. (Périférique suivant)
A:                                                 // Réception en cours...
  t = 0;
  cli();
  delay(2);
  sei();
  //Marqueur(1);                                     // Pour analyseur logique
  detachInterrupt(rxtxPin);
  Xor = 0;
  Bi = 0;
  PosBit = 0;
  NbrBit = 0;
  valTemp = 0;
  for ( int nn = 0; nn < t; nn++) {                  // Lit les datas
    NbrBit += xpData[nn];                              // Additionne bits
    for (int n = 0; n < xpData[nn]; n++) {                 // Groupe de bit
      if (xpValBit[nn] == true)                        // Bit haut ?
        bitSet(valTemp, PosBit);                     // Positionne bit haut
      PosBit++;                                      // Bit suivant
    }
    if (NbrBit >= 10) {                              // Byte complet
      nn++;                                          // Saute bit Stop
      PosBit = 0;                                    // Réinit. pour Byte suivant
      NbrBit = 0;
      BufXpress[Bi] = valTemp >> 1;                  // Ignore bit Stop
      Xor ^= BufXpress[Bi++];                        // Xor des datas
      valTemp = 0;
    }
  }
  if (Xor != 0) {                                  // Si pas d'erreur, Xor = 0
    // DIAG(F("[XPRESSNET] Wrong checksum"));         // Mauvais checksum, retour
    Serial.println("Mauvais checksum");            // Mauvais checksum, retour
    return ;
  }
 
  for (int n = 0; n < 32; n += 8) {                  // Pour adressage prioritaire,
    UserOps[n] = UserOps[Ti];                    // Mémorise adresse d'appel     4x plus rapide.
    DirectedOps[n] = DirectedOps[Ti];            // Mémorise adresse de réponse
  }
  Decodage();                                    // Décodage de la trame reçue
}
//-------------------------------------------------------------------
void xpCvValueCallback(int16_t inValue) {
  if (inValue >= 0)
    stateCV = StateCV::Reading_OK;
  else
    stateCV = StateCV::Reading_FAIL;
  xpCvValue = inValue;
}
//-----------------------------------------------------------------------
void XpressCvWriteValueCallback(int16_t inValue) {
  if (inValue >= 0)
    stateCV = StateCV::Written_OK;
  else
    stateCV = StateCV::Written_FAIL;
}
//------------------------------------------------------------
void XPressNetESP::Decodage() {
   if (DIAG_XPNET) {
  Serial.print("=> ");
  for (int n = 0; n < Bi; n++) {                     // DEBUG
    if (BufXpress[n] < 10) Serial.print("0");
    Serial.print(BufXpress[n], HEX);
    Serial.print(" ");
  }
  Serial.println();
   }
  Bit9 = false;
  switch (BufXpress[0]) {
    case 0xE6:  {                                        //   POM CV write MultiMaus
        switch (BufXpress[1]) {
          case 0x30:
            uint16_t Adr = ((BufXpress[2] & 0x3F) << 8) + BufXpress[3];
            uint16_t CVAdr = ((BufXpress[4] & B11) << 8) + BufXpress[5];
            if ((BufXpress[4] & 0xFC) == 0xEC)                                //set byte
              DCC::writeCVByteMain(Adr, CVAdr, BufXpress[6]);
            if ((BufXpress[4] & 0xFC) == 0xE8)  {                             //set byte
              byte bitNum = BufXpress[6] & B00000011 << 8;
              bool value = BufXpress[6] & B00000100;
              DCC::writeCVBitMain(Adr, CVAdr, bitNum, value);
            }
            break;
        }
      } break;
    case 0xE4:
      xpAdr = word(BufXpress[2] & 0x3F, BufXpress[3]);
      dir = BufXpress[4] & 0x80;
      xpSpeed = BufXpress[4];
      fonction = BufXpress[4];
      switch (BufXpress[1]) {
        case 0x10:                                // Contrôle vitesse loco. 14 crans
        case 0x11:                                // Contrôle vitesse loco. 27 crans
        case 0x12:                                // Contrôle vitesse loco. 28 crans
        case 0x13:                                // Contrôle vitesse loco. 128 crans
          DCC::setThrottle(xpAdr, xpSpeed, dir);
          break;
        case 0x20:                                // Contrôle F0 à F4 loco
          fonction =  (fonction & 0b00010000) >> 4 | (fonction & 0b00001111) << 1 ;      // F0 F4 F3 F2 F1   moveBit4to0  F4 F3 F2 F1 F0
          XPressNetESP::Fonction(5, 0);
          break;
        case 0x21:                                // Contrôle F5 à F8 loco
          XPressNetESP::Fonction(4, 5);
          break;
        case 0x22:                                // Contrôle F9 à F12 loco
          XPressNetESP::Fonction(4, 9);
          break;
        case 0x23:                                // Contrôle F13 à F20 loco
          XPressNetESP::Fonction(8, 13);
          break;
        case 0x28:                                // Contrôle F21 à F28 loco
          XPressNetESP::Fonction(5, 21);
          break;
      }
      break;
    case 0xE3:
      switch (BufXpress[1]) {
        case 0x00: {
            xpAdr = word(BufXpress[2] & 0x3F, BufXpress[3]);
            xpSpeed = DCC::getThrottleSpeed(xpAdr);
            XPressNetESP::getfonction(xpAdr);
            uint8_t LocoInfo[] = {DirectedOps[Ti], 0xE4, 0x04, xpSpeed, Gr1, Gr2, 0x00 };   // 0xE4  0x04  (F0 à F9)
            XPressNetESP:: getXOR(LocoInfo, 7);
            XNetsend (LocoInfo, 7);
          } break;
      }
      break;
    case 0x91:                                          // Requête de demande d’arrêt de d'urgence d’une locomotive adresse courte
      DCC::setThrottle(BufXpress[1] , 1, 1);
      break;
    case 0x92:                                          // Requête de demande d’arrêt de d'urgence d’une locomotive adresse longue
      xpAdr = word( BufXpress[2] & 0x1F,  BufXpress[3]);
      DCC::setThrottle(xpAdr, 1, 1);
      break;
    case 0x80:                                          // Arrêt urgence toutes loco.
      // DCC::setThrottle(0, 1, 1);
      break;
    case 0x52: {                                         // Requête de commande à un décodeur d’accessoire.
        uint16_t Address((BufXpress[1]  << 2) | (BufXpress[2] & B110) >> 1);
        int data = BufXpress[2];
        Address = (Address >> 2) + OffsetAdress;                   // 00AAAAAA
        uint8_t port = ((data & 0x06) >> 1) + OffsetPort;          // 000000BB
        bool gate = bitRead(data, 0);                              // 0000000D
        bool OnOff =  bitRead(data, 3);                            // 0000d000
        DCC::setAccessory(Address, port, gate, OnOff);
        byte pos1 = bitRead(data, 2);
        byte pos2 = data & B11;
        if (pos2 == 0) data = 0x05;
        if (pos2 > 0) data = 0x06;
        if (pos2 == 3) data = 0x0A;
        if (pos1 == 1)
          bitSet(data, 4);
        Address -= 1;
        uint8_t TrntInfo[] = {DirectedOps[Ti], 0x42, 0x00, 0x00, 0x00 }; // Feedback (0x42,MOD,DATA,...,XOR) Accessory decoder information response
        TrntInfo[2] = Address;
        TrntInfo[3] = data;
        getXOR(TrntInfo, 5);
        XNetsend(TrntInfo, 5);
      } break;
    case 0x42: {                                          // 0x42 Demande informations sur le décodeur d'accessoires
        int Address = ((BufXpress[1] << 2) | (BufXpress[2] & B110) >> 1);
        int data = BufXpress[2];
        Address = (Address >> 2); // + OffsetAdress;
        Address -= 1;
        uint8_t TrntInfo[] = {DirectedOps[Ti], 0x42, 0x00, 0x00, 0x00 }; // Feedback (0x42,MOD,DATA,...,XOR) Accessory decoder information response
        TrntInfo[2] = Address;
        TrntInfo[3] = data;
        getXOR(TrntInfo, 5);
        XNetsend(TrntInfo, 5);
      } break;
    case 0x23:                                           // Requête d'écriture CV en Mode Direct (CV mode (CV=1..256))
      if (BufXpress[1] == 0x12) {                        // Register Mode write request (Register Mode)
        Serial.println("Non supporté");
      }
      if (BufXpress[1] == 0x16) {                        // Direct Mode CV write request (CV mode)
        xpCvAddress = BufXpress[2];
        xpCvValue = BufXpress[3];
        if (DIAG_XPNET)
          DIAG(F("[XPRESSNET] Write CV %d  value:%d."), xpCvAddress, xpCvValue);               // DEBUG
        void (*ptr)(int16_t) = &XpressCvWriteValueCallback;
        DCC::writeCVByte(xpCvAddress, xpCvValue, ptr);
        stateCV = Writing;
      }
      if (BufXpress[1] == 0x17) {                        // Paged Mode write request(Paged mode)
        Serial.println("Non supporté");
      }
      break;
    case 0x22:                                           // Requête de lecture CV
      if (BufXpress[1] == 0x11) {                        //Register Mode read request
        Serial.println("Non supporté");
      }
      if (BufXpress[1] == 0x14) {                        //Paged Mode read request
        Serial.println("Non supporté");
      }
      if (BufXpress[1] == 0x15) {                         //Direct Mode CV read request
        // Serial.println("Demande Cv");                 // DEBUG
        xpCvAddress = BufXpress[2];
        xpCvValue = -1;
        if (DIAG_XPNET)
          DIAG(F("[XPRESSNET] Read CV %d."), xpCvAddress);               // DEBUG
        void (*ptr)(int16_t) = &xpCvValueCallback;
        DCC::readCV(xpCvAddress, ptr);
        stateCV = Reading;
      }
      break;
    case 0x21:
      switch (BufXpress[1] ) {
        case 0x80:                                         // Requête de commande d’arrêt d’urgence
          TrackManager::setMainPower(POWERMODE::OFF);
          Railpower = csTrackVoltageOff;
          XNetsend( TrackPowerOp, 4 );                    // 0x60, 0x61, 0x00, 0x61     Broadcast Track power OFF
          break;
        case 0x81:                                         // Requête de commande de reprise des opérations
          TrackManager::setMainPower(POWERMODE::ON);
          Railpower = csNormal;
          XNetsend(NormalOp, 4);                         // 0x60, 0x61, 0x01, 0x60        Broadcast Normal operation
          break;
        case 0x24: {                                        // Demande de status LaBox (0x21 0x24 0x05)
            uint8_t  status = 0x01;                  //csTrackVoltageOff = B1;
            switch (Railpower) {
              case csNormal:        status = 0x00; break;
              case csEmergencyStop: status = 0x01; break;
              case csServiceMode:   status = 0x08; break;
              case csShortCircuit:  status = 0x02; break;
            }
            uint8_t   sendStatus[] = { DirectedOps[Ti], 0x62, 0x22, status, 0x00 };
            XPressNetESP::getXOR(sendStatus, 5);
            XNetsend(sendStatus, 5);
            Ti--;
          } break;
        case 0x21: {                                        // Demande Version Centrale XpressNet    (0x21 0x21 0x00)
            uint8_t  sendVersion[] = { DirectedOps[Ti], 0x63, 0x21, 0x30, 0x00, 0x00 };   //63 21 36 0 74
            XPressNetESP::getXOR(sendVersion, 6);
            XNetsend(sendVersion, 6);
            Ti--;
          } break;
        case 0x10:                                         // Lecture du CV Service mode
          if (stateCV == StateCV::Reading_OK) {
            stateCV = StateCV::Ready;
            if (DIAG_XPNET)
              DIAG(F("[XPRESSNET] Reading answer Cv."), xpCvAddress, xpCvValue);               // DEBUG
            uint8_t  sendReading_OK[] = { DirectedOps[Ti], 0x63, 0x14, xpCvAddress, xpCvValue, 0x00 };   //63 14 ou 15 ?
            XPressNetESP::getXOR(sendReading_OK, 6);
            XNetsend(sendReading_OK, 6);
          }
          else if (stateCV == StateCV::Reading) {
            DIAG(F("[XPRESSNET] Reading..."));               // DEBUG
            uint8_t  sendReading[] = { DirectedOps[Ti], 0x61, 0x1F, 0x00 };   //63 1F
            XPressNetESP::getXOR(sendReading, 4);
            XNetsend(sendReading, 4);
          }
          else {

          }
          break;
      }

    default:
      //  XNetsend(NoOp, 3);                                 // Commande non supportée. {0x61, 0x82, 0xE3}
      break;
  }
}
//-----------------------------------------------------
void  XPressNetESP::Fonction(int a, int b) {                  // 28 fonctions
  for (int i = 0; i < a; i++) {
    if (bitRead(xpRegFonc, i + b) !=  bitRead(fonction, i)) {
      DCC::setFn(xpAdr, i + b, bitRead(fonction, i));
      bitWrite( xpRegFonc, i + b, bitRead(fonction, i));
    }
  }
}
//-------------------------------------------------
void XPressNetESP::getXOR (uint8_t *data, byte length) {       // calculate the XOR
  byte XOR = 0x00;
  data++;
  for (int i = 0; i < (length - 2); i++) {
    XOR = XOR ^ *data;
    data++;
  }
  *data = XOR;
}
//--------------------------------------------------------------------------------------------
void XPressNetESP::XNetsend(unsigned char *dataString, byte byteCount) {
  pinMode(rxtxPin, OUTPUT);             // Pin RxTx en sortie
  digitalWrite (dirPin, HIGH);
  Bit9 = false;
  for (int i = 0; i < byteCount; i++)
    Tx9(*dataString++);
  digitalWrite (dirPin, LOW);
  // pinMode(rxtxPin, INPUT_PULLUP);                 // Pin RxTx en entrée
}
//--------------------------------------------------------------------------------------------
void XPressNetESP::Tx9(int dTx) {    // Envoi d'un octet 62500 Bauds   9 bits  1 stop
  digitalWrite(rxtxPin, LOW);        // Bit Start
  delayMicroseconds(15);             // Largeur 16µs
  cli();
  for (int m = 0; m < 8; m++) {          // 8 bits
    if (bitRead(dTx, m))             // Test état du bit
      digitalWrite(rxtxPin, HIGH);   // Etat haut
    else                             // Si non
      digitalWrite(rxtxPin, LOW);    // Etat bas
    delayMicroseconds(15);           // Largeur 16µs
  }                                  // bit suivant
  sei();
  if (Bit9 == false)                 // Bit 9
    digitalWrite(rxtxPin, HIGH);     // Bit 9 Haut
  else
    digitalWrite(rxtxPin, LOW);      // Bit 9 Bas
  Bit9 = true;
  if (OpCode == true)                // 8 bits sans Stop
    return;
  delayMicroseconds(15);             // 16µs
  digitalWrite(rxtxPin, HIGH);       // 1 Bit Stop
  delayMicroseconds(15);             // 16µs
}
//---------------------------------------------
void XPressNetESP::getfonction(uint8_t xpAdr) {
  uint32_t FonctionMap = DCC::getFunctionMap(xpAdr);
  uint8_t moveBit0to4 = FonctionMap & 0x1F ;     // 5 bits
  Gr1 = (moveBit0to4 & 0b00000001) << 4 | (moveBit0to4 & 0b00011111) >> 1 ;   // F4 F3 F2 F1 F0    moveBit0to4  F0 F4 F3 F2 F1
  //Gr1 = (FonctionMap & 0x1F) ;                 // Groupe 1: 0 0 0 F0 F4 F3 F2 F1              bit0 à bit4
  Gr2 = (FonctionMap & 0xF00) >> 8;              // Groupe 2: 0 0 0 0 F8 F7 F6 F5               bit8 à bit11
  Gr3 = (FonctionMap & 0xF000) >> 12;            // Groupe 3: 0 0 0 0 F12 F11 F10 F9            bit12 à bit15
  Gr4 = (FonctionMap & 0xFF0000) >> 16;          // Groupe 4: F20 F19 F18 F17 F16 F15 F14 F13   bit16 à bit23
  Gr5 = (FonctionMap & 0xFF000000) >> 24;        // Groupe 5: F28 F27 F26 F25 F24 F23 F22 F21   bit24 à bit31
}
//--------------------------------------------------------------
void XPressNetESP::Marqueur(int a) {    // Pour analyseur logique
  digitalWrite(Logic, HIGH);      // Haut __-
  //  delayMicroseconds(a);
  digitalWrite(Logic, LOW);       // Bas  -__
}
#endif

