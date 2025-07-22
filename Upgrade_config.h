
// Use EXComm XPressNet protocol via Serial2
//#define ENABLE_XPRESSNET                         //--------------- Enable XpressNet -----------------

#ifdef ENABLE_XPRESSNET
#define XPRESSNETCOMM    new XPressNetESP(13, 15)  
#else
#define XPRESSNETCOMM		NULL
#endif

// Use EXComm LocoNet protocol via Serial2
//#define ENABLE_LOCONET                            //--------------- Enable LocoNet -----------------

#ifdef ENABLE_LOCONET                            
#define LOCONETCOMM    new LocoNetESP(39, 13)   
#else
#define LOCONETCOMM   NULL
#endif

// Use EXComm S88 protocol via Pin I/O
//#define ENABLE_RETROS88                              //---------------- Enable S88 -----------------------

#ifdef ENABLE_RETROS88                                     
#define RETROS88COMM    new RetroS88(12, 26, 25, 14)        
#else
#define RETROS88COMM   NULL
#endif

#define LABOX_EXCOMMS \
  Z21COMM, \
  CANMARKLINCOMM, \
  SPROGCOMM, \
  XPRESSNETCOMM, \
  LOCONETCOMM, \
  RETROS88COMM
#endif


