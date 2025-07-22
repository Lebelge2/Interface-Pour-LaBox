/*
  Librairies LOCONET  ESP32 (5 fichiers)  https://github.com/tanner87661/LocoNetESP32HB

  lebelge2@yahoo.fr
  Dernière modif: 21-07-25
*/
// ============================================================= LOCONET MASTER V.1 =============================================================
#include "LocoNetESP.h"
#include "TrackManager.h"
#include "DCC.h"
#include "EXComm.h"

#ifdef ENABLE_LOCONET

#define OffsetAdressLo   1
#define OffsetPortLo     0

uint8_t cvnum = 0;
uint8_t cvvalue = 0;
uint8_t chk;
uint8_t locoNetSlots[MAX_MAIN_REGISTERS][15];
uint8_t Data[15];
uint8_t rwSlot;
uint8_t F0F4;
uint8_t F5F8;
uint8_t memoF0F4;
uint8_t memoF5F8;
int rxPin;
int txPin;
uint16_t addres;
uint8_t port;
bool gate;
uint8_t OnOff;

LocoNetESP::LocoNetESP(int inRxPin, int inTxPin) : EXCommItem("LocoNetESP") {
  rxPin = inRxPin;
  txPin = inTxPin;
}
//LocoNetESPSerial lnSerial(rxPin, txPin);  // Vas pas!
LocoNetESPSerial lnSerial(39, 13);

bool LocoNetESP::begin() {
  for (rwSlot = 0; rwSlot < MAX_MAIN_REGISTERS; rwSlot++) {
    locoNetSlots[rwSlot][0] = 0xE7;                         // opcode
    locoNetSlots[rwSlot][1] = 0x0E;                         // msg_size
    locoNetSlots[rwSlot][2] = rwSlot + 1;                   // slot
    locoNetSlots[rwSlot][3] = LOCO_FREE | DEC_MODE_128;     // stat
    locoNetSlots[rwSlot][4] = 0;                            // adr
    locoNetSlots[rwSlot][5] = 0;                            // speed
    locoNetSlots[rwSlot][6] = 0;                            // dirf
    locoNetSlots[rwSlot][7] = GTRK_POWER & GTRK_MLOK1;      // trk
    locoNetSlots[rwSlot][8] = 0;                            // ss2
    locoNetSlots[rwSlot][9] = 0;                            // adr2
    locoNetSlots[rwSlot][10] = 0;                           // snd
    locoNetSlots[rwSlot][11] = 0;                           // id1
    locoNetSlots[rwSlot][12] = 0;                           // id2
  }
  //TrackManager::setMainPower(POWERMODE::ON);               // Rails sous tension
  return true;
}
//--------------------------------------------------------------------------
bool LocoNetESP::loop() {
  lnSerial.processLoop();
  return true;
}
//-----------------------------------------------------------------------------------------------------------
void onLocoNetMessage(lnReceiveBuffer *LnPacket) {
  int i; int freeslot = MAX_MAIN_REGISTERS;
  uint16_t myAddress;
  uint8_t  mySpeed;
  bool myDir;
  unsigned char opcode = (int)LnPacket->lnData[0];

  if (DIAG_LONET) {
    chk = 0;
    Serial.print("Rx ");
    for (int i = 0; i < LnPacket->lnMsgSize; i++) {
      uint8_t val = LnPacket->lnData[i];
      chk ^= val;                       // Cheksum
      if (val < 16)  Serial.print('0');
      Serial.print(val, HEX);
      Serial.print(" ");
    }
    if (chk != 0xFF) {
      Serial.println("  Erreur cheksum");
      return;
    }
    else
      Serial.println();
  }
  switch (opcode) {                                // Opcode
    case OPC_GPON:                                 // 0x83 Global ON command
      TrackManager::setMainPower(POWERMODE::ON);
      break;

    case OPC_GPOFF:                                // 0x82 Global OFF command
      TrackManager::setMainPower(POWERMODE::OFF);
      break;

    case OPC_IDLE:                                 // 0x85 Stop emergency
      DCC::setThrottle(0, 1, 1);                   // Arrêt urgence toutes loco.
      break;

    case OPC_LOCO_ADR:                             // 0xBF Request of Loco
      // Check if it is in slot already and also gets the first possible free slot
      // Slot 0 is not examined as it is used as BT2 slot (TODO not implemented)
      for (rwSlot = 1; rwSlot < MAX_MAIN_REGISTERS; rwSlot++) {
        if (((locoNetSlots[rwSlot][3] & LOCOSTAT_MASK) == LOCO_FREE) && rwSlot < MAX_MAIN_REGISTERS)
          freeslot = rwSlot;      // Slot libre trouvé
        else if (locoNetSlots[rwSlot][4] == LnPacket->lnData[2] && locoNetSlots[rwSlot][9] == LnPacket->lnData[1])
          break;
      }
      if (rwSlot == MAX_MAIN_REGISTERS && freeslot == MAX_MAIN_REGISTERS) {       // Loco not found and no free slots
        if (DIAG_LONET) Serial.println("!LONGACK! No free slots for loco");
        LocoNetESP::sendLongAck(OPC_LONG_ACK, OPC_PEER_XFER, 0);         // 0xB4   0xE5    0x00   (échec)
        break;
      }
      // Loco not found, add to the first free slot speed 0, direction front, F0 ON
      if (rwSlot == MAX_MAIN_REGISTERS) {
        rwSlot = freeslot;
        locoNetSlots[rwSlot][0] = 0xE7;
        locoNetSlots[rwSlot][1] = 0x0E;
        locoNetSlots[rwSlot][2] = rwSlot;
        locoNetSlots[rwSlot][3] = LOCO_IDLE | DEC_MODE_128;
        locoNetSlots[rwSlot][4] = LnPacket->lnData[2];
        locoNetSlots[rwSlot][5] = 0;
        locoNetSlots[rwSlot][6] = DIRF_F0;
        locoNetSlots[rwSlot][7] &= GTRK_POWER & GTRK_MLOK1; // POWER ON & Loconet 1.1 by default
        locoNetSlots[rwSlot][8] = 0;
        locoNetSlots[rwSlot][9] = LnPacket->lnData[1];
        locoNetSlots[rwSlot][10] = 0;
        locoNetSlots[rwSlot][11] = 0;
        locoNetSlots[rwSlot][12] = 0;
      }
      if (DIAG_LONET) Serial.println("SLOT SENT");
      for (int i = 0; i < 13; i++)
        Data[i] = locoNetSlots[rwSlot][i];       // Copie data
      LocoNetESP::sendData(14);
      break;

    case OPC_MOVE_SLOTS:                               // 0xBA
      // Check slot range (0 DISPATCH NOT SUPPORTED, DIFFERENT NOT SUPPORTED)
      if (LnPacket->lnData[2] >= MAX_MAIN_REGISTERS || LnPacket->lnData[1] >= MAX_MAIN_REGISTERS || LnPacket->lnData[2] != LnPacket->lnData[1] || LnPacket->lnData[2] < 1 || LnPacket->lnData[1] < 1) {
        LocoNetESP::sendLongAck(OPC_LONG_ACK, OPC_PEER_XFER, 0);         // 0xB4   0xE5    0x00   (échec)
        return;
      }
      locoNetSlots[LnPacket->lnData[2]][3] |= LOCO_IN_USE;
      myAddress = locoNetSlots[LnPacket->lnData[2]][4] + (locoNetSlots[LnPacket->lnData[2]][9] << 7);
      // sprintf(s,"%d %d %d %d",LnPacket->sm.dest, myAddress, locoNetSlots[LnPacket->sm.dest].spd, bitRead(locoNetSlots[LnPacket->sm.dest].dirf,5));
      mySpeed = locoNetSlots[LnPacket->lnData[2]][5];
      myDir = bitRead(locoNetSlots[LnPacket->lnData[2]][6], 5);
      DCC::setThrottle(myAddress, mySpeed, myDir);
      F0F4 = locoNetSlots[LnPacket->lnData[2]][6] ;                      //               F0 F4 F3 F2 F1
      F0F4 = (F0F4 & 0b00010000) >> 4 | (F0F4 & 0b00001111) << 1 ;       // moveBit4to0:  F4 F3 F2 F1 F0
      for (int i = 0; i < 5; i++) {
        if ( bitRead(memoF0F4, i) !=  bitRead(F0F4, i))
          DCC::setFn(myAddress, i, bitRead(F0F4, i));
      }
      memoF0F4 = F0F4;
      break;

    case OPC_SLOT_STAT1:                                                 // 0xB5
      /*locoNetSlots[LnPacket->lnData[1]][3] = LnPacket->lnData[2];
         myAddress = locoNetSlots[LnPacket->lnData[1]][4];
         mySpeed = locoNetSlots[LnPacket->lnData[1]][5];
         myDir = bitRead(locoNetSlots[LnPacket->lnData[1]][6], 5);
         DCC::setThrottle(myAddress, mySpeed, myDir); */
      break;

    case OPC_LOCO_SPD:                                                   // 0xA0
      locoNetSlots[LnPacket->lnData[1]][5] = LnPacket->lnData[2];
      myAddress = locoNetSlots[LnPacket->lnData[1]][4] + (locoNetSlots[LnPacket->lnData[1]][9] << 7);
      mySpeed = locoNetSlots[LnPacket->lnData[1]][5];
      myDir = bitRead(locoNetSlots[LnPacket->lnData[1]][6], 5);
      DCC::setThrottle(myAddress, mySpeed, myDir);
      break;

    case OPC_LOCO_DIRF:                                                   // 0xA1    F0 à F4
      locoNetSlots[LnPacket->lnData[1]][6] = LnPacket->lnData[2];
      myAddress = locoNetSlots[LnPacket->lnData[1]][4] + (locoNetSlots[LnPacket->lnData[1]][9] << 7);
      mySpeed = locoNetSlots[LnPacket->lnData[1]][5];
      myDir = bitRead(locoNetSlots[LnPacket->lnData[1]][6], 5);
      DCC::setThrottle(myAddress, mySpeed, myDir);
      F0F4 = locoNetSlots[LnPacket->lnData[1]][6] ;                        // F0 F4 F3 F2 F1
      F0F4 = (F0F4 & 0b00010000) >> 4 | (F0F4 & 0b00001111) << 1 ;         // moveBit4to0:  F4 F3 F2 F1 F0
      for (i = 0; i < 5; i++) {
        if (bitRead(memoF0F4, i) !=  bitRead(F0F4, i))
          DCC::setFn(myAddress, i, bitRead(F0F4, i));
      }
      memoF0F4 = F0F4;
      break;

    case OPC_LOCO_SND:                                                    // 0xA2    F5 à F8
      locoNetSlots[LnPacket->lnData[1]][10] = LnPacket->lnData[2];
      myAddress = locoNetSlots[LnPacket->lnData[1]][4] + (locoNetSlots[LnPacket->lnData[1]][9] << 7);
      F5F8 = locoNetSlots[LnPacket->lnData[1]][10];
      for (i = 0; i < 4; i++) {
        if (bitRead(memoF5F8, i) !=  bitRead(F5F8, i))
          DCC::setFn(myAddress, i + 5, bitRead(F5F8, i));
      }
      memoF5F8 = F5F8;
      break;

    case OPC_SW_REQ:                                                         // 0xB0     Fonctions loco et  Aiguillages
      addres = (LnPacket->lnData[1]) >> 2;
      addres += OffsetAdressLo;
      port = (LnPacket->lnData[1]) & 3;
      port += OffsetPortLo;
      gate = bitRead((LnPacket->lnData[2]), 5);
      OnOff = bitRead((LnPacket->lnData[2]), 4);
      DCC::setAccessory(addres, port, gate, OnOff);
      //Serial.print(addres); Serial.print(" ");Serial.print(port); Serial.print(" "); Serial.print(gate); Serial.print(" ");Serial.println(OnOff);
      break;

    default:
      if (DIAG_LONET) {
        Serial.print("Ignore message, opcode: ");      // ignore the message...
        Serial.println(opcode, HEX);
      }
      break;
  }
}
//------------------------------------------------------
void LocoNetESP::sendLongAck(uint8_t opCode, uint8_t data, uint8_t ucCode) {
  lnTransmitMsg myMsg;
  myMsg.lnMsgSize = 4;
  myMsg.reqID = random(2000);
  myMsg.lnData[0] = opCode;
  myMsg.lnData[1] = data ; //- 0x80;
  myMsg.lnData[2] = ucCode;
  myMsg.lnData[3] = myMsg.lnData[0] ^ myMsg.lnData[1] ^ myMsg.lnData[2] ^ 0xFF;   // Checksum à la fin.
  if (lnSerial.carrierOK())
    int numBytes = lnSerial.lnWriteMsg(myMsg);
  else
    Serial.println("LocoNet not connected");
}
//-------------------------------------------------------------------------
void LocoNetESP::sendData(uint8_t len) {
  int i;
  lnTransmitMsg myMsg;
  myMsg.lnMsgSize = len;
  myMsg.reqID = random(2000);
  chk = 0;
  for (i = 0; i < len - 1; i++) {
    myMsg.lnData[i] = Data[i];       // Copie datas et calcul checksum.
    chk ^= Data[i];
  }
  myMsg.lnData[i] =  chk ^ 0xFF;     // Checksum à la fin.
  if (lnSerial.carrierOK())
    int numBytes = lnSerial.lnWriteMsg(myMsg);
  else
    Serial.println("LocoNet not connected");
}
//----------------------------------------------------
#endif
