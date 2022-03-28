#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/*
 * Defines the pins on which the flowmeters are connected. 
 * These MUST be interrupt-capable pins!
 */
#define FLOWMETER_A  18
#define FLOWMETER_B  19
#define FLOWMETER_C  20
#define FLOWMETER_D  21
#define FLOWMETER_E  3

/*
 * Defines the flow rate factor of each flowmeter
 */
#define FLOWFACTOR_A 7.5
#define FLOWFACTOR_B 7.5
#define FLOWFACTOR_C 0.2
#define FLOWFACTOR_D 0.2
#define FLOWFACTOR_E 5.5

#define VALVE_A  22 
#define VALVE_B  23 
#define VALVE_C  24 
#define VALVE_D  25 
#define VALVE_E  26

/*
 * Defines the flow count interval (in seconds),
 * i.e. how to often the flow counting sequence takes place
 */
#define FCOUNT_INTVAL 1

/*
 * Defines the uplink message interval (in seconds),
 * i.e. how often the system transmits messages to the infrastructure
 */
#define UPLINK_INTVAL 30

#define DNLINK_LENGTH 16  // Downlink message payload length (in bytes)

/*
 * MAX 51 BYTES!!
 */
#define UPLINK_LENGTH 51  // Uplink message payload length (in bytes) 


unsigned long lastCount = 0;
unsigned long lastCommn = 0;

bool loraMessageSent  = false;
bool loraStatusJoined = false;

uint8_t dnlink[DNLINK_LENGTH];
uint8_t uplink[UPLINK_LENGTH];

String msg;

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

uint8_t buf[40];

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
//static const u1_t PROGMEM APPEUI[8] = { 0x97, 0xE6, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM APPEUI[8] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 };

void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
//static const u1_t PROGMEM DEVEUI[8] = { 0xA6, 0x24, 0xB7, 0x94, 0x0F, 0x54, 0xCB, 0x00 };
static const u1_t PROGMEM DEVEUI[8] = { 0x36, 0x41, 0x98, 0x12, 0x9C, 0x20, 0x2F, 0x00 };

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
//static const u1_t PROGMEM APPKEY[16] = { 0x0C, 0x21, 0x65, 0xFC, 0xA8, 0x8D, 0x99, 0x5D, 0xEC, 0xFB, 0xB0, 0xF0, 0x71, 0x60, 0xF4, 0x73 };
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x5E, 0x44, 0x54, 0x60, 0xA1, 0x4C, 0xC9, 0xA9, 0x8C, 0x81, 0x9F, 0x79, 0x20, 0x9C, 0xD8 };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;
void loraSend(osjob_t*);

// Pin mapping
const lmic_pinmap lmic_pins =
{
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {2, 6, 7},
};

volatile unsigned long long tickCount0 = 0;
volatile unsigned long long tickCount1 = 0;
volatile unsigned long long tickCount2 = 0;
volatile unsigned long long tickCount3 = 0;
volatile unsigned long long tickCount4 = 0;

double flowRate0 = 0.0;
double flowRate1 = 0.0;
double flowRate2 = 0.0;
double flowRate3 = 0.0;
double flowRate4 = 0.0;

double flowCurr0 = 0.0;
double flowCurr1 = 0.0;
double flowCurr2 = 0.0;
double flowCurr3 = 0.0;
double flowCurr4 = 0.0;

double flowTotal0 = 0.0;
double flowTotal1 = 0.0;
double flowTotal2 = 0.0;
double flowTotal3 = 0.0;
double flowTotal4 = 0.0;

void ticker0(void);
void ticker1(void);
void ticker2(void);
void ticker3(void);
void ticker4(void);

void onEvent(ev_t ev) {

  switch (ev)
  {
    case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	          // size, we don't use it in this example.
            loraStatusJoined = true ;

            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(UPLINK_INTVAL), loraSend);

            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:

            //loraMessageSent = true;

            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
            {
              Serial.println(F("Received ack"));
            }
            if (LMIC.dataLen > 0)
            {
              /*
               * Clear the downlink message container;
               */
              memset(dnlink, 0, DNLINK_LENGTH);

              for (uint8_t i = 0; i < LMIC.dataLen; i++)
              {
                dnlink[i] = LMIC.frame[LMIC.dataBeg + i];
              }

              if (LMIC.dataLen >= DNLINK_LENGTH)
              {
                dnlink[DNLINK_LENGTH - 1] = 0;
              }
              else
              {
                dnlink[LMIC.dataLen] = 0;
              }

            }
            break;

        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
  }
}

void setup()
{
  pinMode(FLOWMETER_A, INPUT_PULLUP) ;
  attachInterrupt(digitalPinToInterrupt(FLOWMETER_A), ticker0, FALLING);

  pinMode(FLOWMETER_B, INPUT_PULLUP) ;
  attachInterrupt(digitalPinToInterrupt(FLOWMETER_B), ticker1, FALLING);

  pinMode(FLOWMETER_C, INPUT_PULLUP) ;
  attachInterrupt(digitalPinToInterrupt(FLOWMETER_C), ticker2, FALLING);

  pinMode(FLOWMETER_D, INPUT_PULLUP) ;
  attachInterrupt(digitalPinToInterrupt(FLOWMETER_D), ticker3, FALLING);

  pinMode(FLOWMETER_E, INPUT_PULLUP) ;
  attachInterrupt(digitalPinToInterrupt(FLOWMETER_E), ticker3, FALLING);

  // replace HIGH with LOW if relay open on 1
  pinMode(VALVE_A, OUTPUT);   
  digitalWrite(VALVE_A, HIGH);    
  pinMode(VALVE_B, OUTPUT);   
  digitalWrite(VALVE_B, HIGH);    
  pinMode(VALVE_C, OUTPUT);   
  digitalWrite(VALVE_C, HIGH);    
  pinMode(VALVE_D, OUTPUT);   
  digitalWrite(VALVE_D, HIGH);    
  pinMode(VALVE_E, OUTPUT);   
  digitalWrite(VALVE_E, HIGH);

  Serial.begin(38400);

  memset(dnlink, 0, DNLINK_LENGTH);

  os_init();

  loraStatusJoined = false;

  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
  LMIC_startJoining();

  while (!loraStatusJoined)
  {
    os_runloop_once();
  }

  Serial.print("JOINED!");
  Serial.println();
}

void loop()
{
  os_runloop_once() ;
  
  if (dnlink[0] != 0)
  {
    String payload = String((char*)dnlink);
    Serial.println("Payload: " + payload) ;

    for (uint8_t i = 0; i < payload.length(); i++)
    {
      if (payload.charAt(i) == 'O' || payload.charAt(i) == 'C')
      {
        uint8_t nextControlCharIndex = payload.length();

        for (uint8_t j = i+1; j < payload.length(); j++)
        {
          if (payload.charAt(j) == 'O' || payload.charAt(j) == 'C')
          {
            nextControlCharIndex = j;
            break;
          }
        }

        String pin = payload.substring(i+1, nextControlCharIndex);

        digitalWrite(pin.toInt(), payload.charAt(i) == 'C' ? HIGH : LOW); // replace with C with O if relay open on 1
        
        Serial.println("Pin: " + pin);  
      }
    }

    memset(dnlink, 0, DNLINK_LENGTH);
  }

  if ((millis() - lastCount) > (FCOUNT_INTVAL * 1000))
  {
    detachInterrupt(digitalPinToInterrupt(FLOWMETER_A));
    detachInterrupt(digitalPinToInterrupt(FLOWMETER_B));
    detachInterrupt(digitalPinToInterrupt(FLOWMETER_C));
    detachInterrupt(digitalPinToInterrupt(FLOWMETER_D));
    detachInterrupt(digitalPinToInterrupt(FLOWMETER_E));

    flowRate0 = 0.0;
    flowRate1 = 0.0;
    flowRate2 = 0.0;
    flowRate3 = 0.0;
    flowRate4 = 0.0;

    if (tickCount0)
      flowRate0 = (float)((1000.0 / (millis() - lastCount)) * tickCount0) / FLOWFACTOR_A;

    if (tickCount1)
      flowRate1 = (float)((1000.0 / (millis() - lastCount)) * tickCount1) / FLOWFACTOR_B;

    if (tickCount2)
      flowRate2 = (float)((1000.0 / (millis() - lastCount)) * tickCount2) / FLOWFACTOR_C;

    if (tickCount3)
      flowRate3 = (float)((1000.0 / (millis() - lastCount)) * tickCount3) / FLOWFACTOR_D;

    if (tickCount4)
      flowRate4 = (float)((1000.0 / (millis() - lastCount)) * tickCount4) / FLOWFACTOR_E;


    flowCurr0 = (flowRate0 / 60) * 1000;
    flowCurr1 = (flowRate1 / 60) * 1000;
    flowCurr2 = (flowRate2 / 60) * 1000;
    flowCurr3 = (flowRate3 / 60) * 1000;
    flowCurr4 = (flowRate4 / 60) * 1000;


    flowTotal0 += flowCurr0;
    flowTotal1 += flowCurr1;
    flowTotal2 += flowCurr2;
    flowTotal3 += flowCurr3;
    flowTotal4 += flowCurr4;

    tickCount0 = 0;
    tickCount1 = 0;
    tickCount2 = 0;
    tickCount3 = 0;
    tickCount4 = 0;

    lastCount = millis();

    attachInterrupt(digitalPinToInterrupt(FLOWMETER_A), ticker0, FALLING);
    attachInterrupt(digitalPinToInterrupt(FLOWMETER_B), ticker1, FALLING);
    attachInterrupt(digitalPinToInterrupt(FLOWMETER_C), ticker2, FALLING);
    attachInterrupt(digitalPinToInterrupt(FLOWMETER_D), ticker3, FALLING);
    attachInterrupt(digitalPinToInterrupt(FLOWMETER_E), ticker4, FALLING);
    
    Serial.print("Time: ");
    Serial.print(millis()/1000);
    Serial.println();
  }
}
void loraSend(osjob_t* _j)
{
    msg = "";

    msg += String(flowTotal0 / 1000, 2);
    msg += "_";

    msg += String(flowTotal1 / 1000, 2);
    msg += "_";

    msg += String(flowTotal2 / 1000, 2);
    msg += "_";

    msg += String(flowTotal3 / 1000, 2);
    msg += "_";

    msg += String(flowTotal4 / 1000, 2);

    flowTotal0 = 0.0;
    flowTotal1 = 0.0;
    flowTotal2 = 0.0;
    flowTotal3 = 0.0;
    flowTotal4 = 0.0;

    Serial.print("Message: ");
    Serial.print(msg);
    Serial.println() ;

    memset(uplink, 0, UPLINK_LENGTH);
    strcpy(uplink, msg.c_str());
    
    if (LMIC.opmode & OP_TXRXPEND)
    {
      Serial.println(F("OP_TXRXPEND, not sending"));
      LMIC_clrTxData();
      Serial.println(F("TX DATA CLEARED"));
    }
    
    LMIC_setTxData2(1, uplink, sizeof(uplink) - 1, 1);
    Serial.print(F("Packet queued: "));
    Serial.print((char*)buf);
    Serial.println();

    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(UPLINK_INTVAL), loraSend);
}

void ticker0(void)
{
  tickCount0++;
}
void ticker1(void)
{
  tickCount1++;
}
void ticker2(void)
{
  tickCount2++;
}
void ticker3(void)
{
  tickCount3++;
}
void ticker4(void)
{
  tickCount4++;
}
