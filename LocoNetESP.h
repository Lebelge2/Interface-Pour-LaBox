

#ifndef __LocoNet_h
#define __LocoNet_h
#include <stdint.h>
#include "EXComm.h"
#include <arduino.h>

#ifdef ENABLE_LOCONET
#include <IoTT_LocoNetHBESP32.h>

#define MAX_MAIN_REGISTERS 20               // Nombre max. de loco.
// OPCODES
#define OPC_BUSY          0x81
#define OPC_GPOFF         0x82              // GLOBAL power OFF request 
#define OPC_GPON          0x83              // GLOBAL power ON request
#define OPC_IDLE          0x85
#define OPC_LOCO_SPD      0xA0              // SET SLOT speed
#define OPC_LOCO_DIRF     0xA1              // SET SLOT dir,F0-4 state
#define OPC_LOCO_SND      0xA2              // SET SLOT sound functions 
#define OPC_SW_REQ        0xB0              // REQ SWITCH function
#define OPC_LONG_ACK      0xB4              // Long acknowledge 
#define OPC_SLOT_STAT1    0xB5              // WRITE slot stat1
#define OPC_MOVE_SLOTS    0xBA              // MOVE slot SRC to DEST
#define OPC_LOCO_ADR      0xBF              // REQ  loco ADR
#define OPC_PEER_XFER     0xE5              // move 8 bytes PEER to PEER, SRC->DST
#define OPC_SL_RD_DATA    0xE7              // SLOT DATA return, 10 bytes
#define OPC_WR_SL_DATA    0xEF              // WRITE SLOT DATA, 10 bytes
#define MAX_SLOTS         0X0A
#define GTRK_MLOK1        0x04      /* 0 = Master is DT200, 1=Master implements LocoNet 1.1 */
#define GTRK_POWER        0x01
#define LOCO_FREE         (0)
#define STAT1_SL_SPD14    0x02
#define STAT1_SL_SPD28    0x01
#define DEC_MODE_128      (STAT1_SL_SPD14 | STAT1_SL_SPD28)
#define STAT1_SL_BUSY     0x20
#define STAT1_SL_ACTIVE   0x10
#define LOCOSTAT_MASK     (STAT1_SL_BUSY  | STAT1_SL_ACTIVE)
#define LOCO_IDLE         (STAT1_SL_BUSY)
#define DIRF_F0           0x10      /* Function 0 bit   */
#define LOCO_IN_USE       (STAT1_SL_BUSY  | STAT1_SL_ACTIVE)
#define PCMD_RW           0x40      /* 1 = write, 0 = read                                  */
#define PCMD_BYTE_MODE    0x20      /* 1 = byte operation, 0 = bit operation (if possible)  */
#define PCMD_OPS_MODE     0x04      /* 1 = Ops mode, 0 = Service Mode                       */
#define PRG_SLOT          0x7c      /* This slot communicates with the programming track    */
#define CVH_CV8_CV9       0x30      /* mask for CV# bits 8 and 9    */
#define CVH_CV7           0x01      /* mask for CV# bit 7           */
#define CVH_D7            0x02      /* MSbit for data value         */
#define OPC_MASK          0x7f      /* mask for acknowledge opcodes */

const bool DIAG_LONET = false;    // Sans Debug.
//const bool DIAG_LONET = true;     // Avec Debug.

class LocoNetESP: public EXCommItem {
  public:
    LocoNetESP(int inRxPin, int inTxPin, bool invSerial);                  // Niveaux séries inversés (par défaut)
    bool begin() override;
    bool loop() override;
   // void getInfos(String *pMess1, String *pMess2, String *pMess3, byte maxSize) override;
    static void onLocoNetMessage(lnReceiveBuffer *LnPacket);
    static void sendLongAck(uint8_t a, uint8_t b, uint8_t c);
    static void sendData(uint8_t len);
};

#endif
#endif

