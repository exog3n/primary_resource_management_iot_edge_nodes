#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "flowmeter.h"
#include "valve.h"
#include "utils.h"

//  Set this to 1 in order to
//  print debugging and info messages:
//
#define DEBUG 1

/*
   Defines the flow count interval (in seconds),
   i.e. how to often the flow counting sequence takes place
*/
#define FCOUNT_INTVAL 1

/*
   Defines the uplink message interval (in seconds),
   i.e. how often the system transmits messages to the infrastructure
*/
#define UPLINK_INTVAL 30

#define DNLINK_LENGTH 16  // Downlink message payload length (in bytes)

/*
   MAX 51 BYTES!!
*/
#define UPLINK_LENGTH 51  // Uplink message payload length (in bytes) 


unsigned long lastCount = 0;
unsigned long lastCommn = 0;

bool loraMessageSent  = false;
bool loraStatusJoined = false;

char dnlink[DNLINK_LENGTH];
char uplink[UPLINK_LENGTH];

Valve valves[4] =
{
  Valve(5),
  Valve(37),
  Valve(39),
  Valve(22, 28, LATCHING)
};

Flowmeter flowmeter[4] =
{
  Flowmeter(18, 450),
  Flowmeter(19, 450),
  Flowmeter(20, 450),
  Flowmeter(21, 450)
};

void ticker0() {
  flowmeter[0].tick();
}
void ticker1() {
  flowmeter[1].tick();
}
void ticker2() {
  flowmeter[2].tick();
}
void ticker3() {
  flowmeter[3].tick();
}

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x97, 0xE6, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0xA6, 0x24, 0xB7, 0x94, 0x0F, 0x54, 0xCB, 0x00 };

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x0C, 0x21, 0x65, 0xFC, 0xA8, 0x8D, 0x99, 0x5D, 0xEC, 0xFB, 0xB0, 0xF0, 0x71, 0x60, 0xF4, 0x73 };
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
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
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

      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(UPLINK_INTVAL), loraSend);

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
           Clear the downlink message container;
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
  attachInterrupt(digitalPinToInterrupt(flowmeter[0].getPin()), ticker0, FALLING);
  attachInterrupt(digitalPinToInterrupt(flowmeter[1].getPin()), ticker1, FALLING);
  attachInterrupt(digitalPinToInterrupt(flowmeter[2].getPin()), ticker2, FALLING);
  attachInterrupt(digitalPinToInterrupt(flowmeter[3].getPin()), ticker3, FALLING);

  Serial.begin(38400);

  memset(dnlink, 0, DNLINK_LENGTH);
  memset(uplink, 0, UPLINK_LENGTH);

  os_init();

  loraStatusJoined = false;

  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
  LMIC_startJoining();

  while (!loraStatusJoined)
  {
    os_runloop_once();
  }

  if (DEBUG)
    Serial.println("JOINED!");
}

void loop()
{
  os_runloop_once() ;

  if (dnlink[0] != 0)
  {
    if (DEBUG)
    {
      Serial.print("Downlink message rcvd. Payload: ");
      Serial.print((char*) dnlink);
      Serial.println();
    }

    // Checking if the message rcvd refers to the same number of valves
    // that we have installed on the system:
    //
    if ( strlen(dnlink) != sizeof(valves) / sizeof(* valves))
    {
      if (DEBUG)
        Serial.println("There is a different number of valves on the system than the message specifies.");
    }
    else
    {
      for (uint8_t i = 0; i < strlen(dnlink); i++)
      {
        if (dnlink[i] == '1')
        {
          valves[i].open();
          continue;
        }
        if (dnlink[i] == '0')
        {
          valves[i].shut();
        }
      }
    }

    // Now that the message has been parsed,
    // it can be cleared:
    //
    memset(dnlink, 0, DNLINK_LENGTH);
  }

  if ((millis() - lastCount) > (FCOUNT_INTVAL * 1000))
  {
    // Detach all interrupts to avoid messing with the calculations:
    //
    detachInterrupt(digitalPinToInterrupt(flowmeter[0].getPin()));
    detachInterrupt(digitalPinToInterrupt(flowmeter[1].getPin()));
    detachInterrupt(digitalPinToInterrupt(flowmeter[2].getPin()));
    detachInterrupt(digitalPinToInterrupt(flowmeter[3].getPin()));

    // Calculate the flowrate for the lapsed time since
    // the last count. This automatically updates the total flow:
    //
    for (uint8_t i = 0; i < sizeof(flowmeter) / sizeof(*flowmeter); i++)
    {
      flowmeter[i].update();
    }

    // Re-attach the interrupts for continued operation:
    //
    attachInterrupt(digitalPinToInterrupt(flowmeter[0].getPin()), ticker0, FALLING);
    attachInterrupt(digitalPinToInterrupt(flowmeter[1].getPin()), ticker1, FALLING);
    attachInterrupt(digitalPinToInterrupt(flowmeter[2].getPin()), ticker2, FALLING);
    attachInterrupt(digitalPinToInterrupt(flowmeter[3].getPin()), ticker3, FALLING);

    if (DEBUG)
    {
      Serial.print("Time: ");
      Serial.print(millis() / 1000);
      Serial.println();
    }

    lastCount = millis();
  }
}
void loraSend(osjob_t* _j)
{
  // Clear the uplink message:
  //
  memset(uplink, 0, UPLINK_LENGTH);

  char temp[8] ;

  for (uint8_t i = 0; i < sizeof(flowmeter) / sizeof(*flowmeter); i++)
  {
    // Convert the floating point value to a C-string
    // and append it to the uplink uplink message:
    //
    // NOTE: Using getTotal() on the flowmeter will automatically
    // reset the total to 0
    //
    dtostrf(flowmeter[i].getLitresTotal(), 4, 2, temp);
    strcat(uplink, temp);

    // Append to separator character (underscore)
    // to the uplink message:
    //
    strcat(uplink, "_") ;

    // Clear the temporaray string holder in order to be used again:
    //
    memset(temp, 0, sizeof(temp));
  }

  if (DEBUG)
  {
    Serial.print(F("Message: "));
    Serial.print(uplink);
    Serial.println();
  }
  if (LMIC.opmode & OP_TXRXPEND)
  {
    if (DEBUG)
      Serial.println(F("OP_TXRXPEND, not sending"));

    LMIC_clrTxData();

    if (DEBUG)
      Serial.println(F("TX DATA CLEARED"));
  }

  LMIC_setTxData2(1, uplink, sizeof(uplink) - 1, 1);
  if (DEBUG)
  {
    Serial.print(F("Packet queued: "));
    Serial.println();
  }

  memset(uplink, 0, UPLINK_LENGTH);

  os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(UPLINK_INTVAL), loraSend);
}
