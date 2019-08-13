#include "adafruit_swserial_ble.h"
#include <stdio.h>
#include <stdlib.h>
#include <alloca.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "debug.h"
#include "pincontrol.h"
#include "timer.h"
#include "action_util.h"
#include "ringbuffer.hpp"
#include <string.h>

#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1


#define digit2AsciiByte(v, la, ra)                \
   (ra) = v & 0x0f; (la) = v >> 4;                \
   if ((ra)<10) (ra) += (unsigned char) '0';      \
   else (ra) = ((ra) - 10) + (unsigned char) 'a'; \
   if ((la)<10) (la) += (unsigned char) '0';      \
   else (la) = ((la) - 10) + (unsigned char) 'a'



// DEFAULT PIN ASSIGNMENTS
// can be overridden but have never tested

#ifndef AdaFruitUARTBleRXPin
#define AdaFruitUARTBleRXPin B5
#endif

#ifndef AdaFruitUARTBleTXPin
#define AdaFruitUARTBleTXPin B7
#endif

#ifndef AdaFruitUARTBleCTSPin
#define AdaFruitUARTBleCTSPin F0
#endif

#ifndef AdaFruitUARTBleRTSPin
#define AdaFruitUARTBleRTSPin F1
#endif

// Simple class that addes hw flow control around writing and checking
//
#include "SoftwareSerial.h"

class MySWSerial: public SoftwareSerial {
  uint8_t _cts_pin, _rts_pin;
 public:
  MySWSerial(uint8_t receivePin, uint8_t transmitPin,
	     uint8_t ctsPin, uint8_t rtsPin);
  void     begin(void);
  virtual  size_t write(uint8_t c);
  virtual  int available(void);

  size_t write(const uint8_t *buffer, size_t size) {
    size_t n = 0;
    while (size--) {
      if (write(*buffer++)) n++;
      else break;
  }
    return n;
  }
  
  size_t write(const char *buffer, size_t size) {
      return write((const uint8_t *)buffer, size);
    }
  
  size_t write(const char *str) {
      if (str == NULL) return 0;
      return write((const uint8_t *)str, strlen(str));
    }
};



MySWSerial::MySWSerial(uint8_t receivePin, uint8_t transmitPin, uint8_t ctsPin, uint8_t rtsPin) :
  SoftwareSerial(receivePin, transmitPin), _cts_pin(ctsPin), _rts_pin(rtsPin)
{
}

void MySWSerial::begin()
{
  SoftwareSerial::begin(9600);
  pinMode(_cts_pin, OUTPUT);
  digitalWrite(_cts_pin, HIGH);
  pinMode(_rts_pin, INPUT);
}

size_t MySWSerial::write(uint8_t c)
{
  // flush left-over before a new command
//  if (c == '\r') flush();

  // if (_verbose) SerialDebug.print((char) c);
  
  while (digitalRead(_rts_pin)) _delay_ms(1);
    
  _delay_ms(50);
  return SoftwareSerial::write(c);
}

int MySWSerial::available(void)
{
  if (!SoftwareSerial::available()) {
    // toggle flow control to get more byteses
    digitalWrite(_cts_pin, LOW);
    _delay_ms(1);
    digitalWrite(_cts_pin, HIGH);
  }
  return SoftwareSerial::available();
}



MySWSerial _mySerial(AdaFruitUARTBleRXPin, AdaFruitUARTBleTXPin,
		     AdaFruitUARTBleCTSPin, AdaFruitUARTBleRTSPin);

static bool ble_init(void);

#define RespTimeout 10 /* milliseconds */
#define SendTimeout 10 /* milliseconds */
static bool at_command(const char *cmd, char *resp, uint16_t resplen,
                       bool verbose, uint16_t timeout = RespTimeout);
static bool at_command_P(const char *cmd, char *resp, uint16_t resplen,
                         bool verbose = false);

#define SAMPLE_BATTERY
#define ConnectionUpdateInterval 1000 /* milliseconds */

static struct {
  bool is_connected;
  bool initialized;
  bool configured;

#define ProbedEvents 1
#define UsingEvents 2
  bool event_flags;

#ifdef SAMPLE_BATTERY
  uint16_t last_battery_update;
  uint32_t vbat;
#endif
  uint16_t last_connection_update;
} state;


static void set_connected(bool connected);

extern "C" {
  extern void test_init(void) {
    //_mySerial.begin();
    // ble_init();
    // this is now handled by initlization logic of ble_task/enable keyboard
  }

  extern void test_pin(int i, int p) {
    switch (i) {
    case 0: {
      set_connected(true);
    } break;
      
#if 0
    case 1: {
      if (p) {
	uint8_t cmd[4]="AT\n";
	_mySerial.write((const uint8_t *)cmd, 3);
	dprintf("%s\n",cmd);
	cmd[1]=0;
	while (_mySerial.available()) {
	  cmd[0] = _mySerial.read();
	  dprintf("%s", cmd);				  
	}
	dprintln();
      }
    }
      break;
    case 2: {
      if (p) {
	uint8_t cmd[16]="AT+EVENTENABLE\n";
	_mySerial.write((const uint8_t *)cmd, 15);
	dprintf("%s\n",cmd);
	cmd[1]=0;
	while (_mySerial.available()) {
	  cmd[0] = _mySerial.read();
	  dprintf("%s", cmd);				  
	}
	dprintln();
      }
    }
      break;
    case 3:{
      if (p) {
	char rbuf[16];
	at_command("AT\n",rbuf,sizeof(rbuf),true,0);
	dprintf("[%s]",rbuf);
      }
    }
      break;
    case 4:{
      if (p) {
	at_command("AT\n",NULL,0,true,0);
      }
    }
      break;
    case 5:{
      if (p) {
	if (!state.configured && !adafruit_ble_enable_keyboard()) {
	  adafruit_ble_enable_keyboard();
	} else dprint("$\n");
      }
    }
      break;
    default:
      dprint("?");
#endif
    }
  }
};


#define BatteryUpdateInterval 10000 /* milliseconds */



static bool ble_init(void) {
  //  dprint("ble_init\n");
  state.initialized = false;
  state.configured = false;
  state.is_connected = false;

  _mySerial.begin();

  state.initialized = true;
  return state.initialized;
}

static inline uint8_t min(uint8_t a, uint8_t b) {
  return a < b ? a : b;
}


class TermBuf {
  char buf[4];
  int n;
  bool needTermination;
public:

  void reset() {
    for(int i=0; i<sizeof(buf); i++) buf[i]=0;
    n=0;
    needTermination=true;
  }
  
  TermBuf() {
    reset();
    // inialize so no termination required for first transaction;
    needTermination = false; 
  }

  void add(char c) { buf[n%4]=c; n++; }

  bool check() {
    char c1,c2,c3,c4;
    
    if (!needTermination) return true; // already terminated

    c4 = buf[(n-1)%4]; // last character is n-1
    c3 = buf[(n-2)%4]; // second last character is n-2
    c2 = buf[(n-3)%4]; // third last char is n-3;
    c1 = buf[(n-4)%4]; // third last char is n-3;

    //    dprintf("%d:*%d,%d,%d,%d*\n", n, c1, c2, c3, c4);
    
    if (n>3 && c4 == '\n' && c3 == '\r' && c2 == 'K' && c1 == 'O' ) {
      needTermination = false;
      //      dprint("+\n");
      return true;
    }
    // dprint("-\n");
    return false;
  }
  
  bool addAndcheck(char c) {
    add(c);
    return check();
  }
  
} termBuf;

static int read_response(char *resp, uint16_t resplen, bool verbose, uint16_t timeout)
{
  char c;  
  int i=0;
  bool terminated=false;

 retry:
  while (_mySerial.available()) {
    c = _mySerial.read();
    if (verbose) dprintf("(%d)", c);
    if (resp && resplen) {
      resp[i] = c;
      i++;
      resplen--;
    }
    if (termBuf.addAndcheck(c)) terminated=true; // response terminated we can leave now
  }
  if (resp && !terminated) goto retry;
  return i;
}



static bool at_command(const char *cmd, char *resp, uint16_t resplen,
                       bool verbose, uint16_t timeout) {
  //  dprint("at_command\n");

  if (resp && !termBuf.check()) {
    // we need a reponse but there seems to be an outstand reply
    // that has not been received we must wait and consume old reponse
    // first
    //dprint("!\n");
    read_response(NULL, 0, false, 0);
  }
  // start a new transaction
  termBuf.reset();  
  int n=_mySerial.write(cmd);
  if (verbose) dprintf("%s", cmd);
  if (resp) {
    int c=read_response(resp, resplen, false, timeout);
    if (c<resplen) resp[c]=0; // null terminate if possible
    if (verbose) dprintf("%s", resp);
  }
  return true;
}

bool at_command_P(const char *cmd, char *resp, uint16_t resplen, bool verbose) {
  //  dprint("at_command_P");
  auto cmdbuf = (char *)alloca(strlen_P(cmd) + 1);
  strcpy_P(cmdbuf, cmd);
  return at_command(cmdbuf, resp, resplen, verbose);
}

bool adafruit_ble_is_connected(void) {
  return state.is_connected;
}

bool adafruit_ble_enable_keyboard(void) {
  char resbuf[128];

  if (!state.initialized && !ble_init()) {
    //dprint("ERR:enkb: !ble_init\n");
    return false;
  }

  state.configured = false;

  // Disable command echo
  static const char kEcho[] PROGMEM = "ATE=0\n";
  // Make the advertised name match the keyboard
  static const char kGapDevName[] PROGMEM =
      "AT+GAPDEVNAME=" STR(PRODUCT) " " STR(DESCRIPTION) "\n";
  // Turn on keyboard support
  static const char kHidEnOn[] PROGMEM = "AT+BLEHIDEN=1\n";

  // Adjust intervals to improve latency.  This causes the "central"
  // system (computer/tablet) to poll us every 10-30 ms.  We can't
  // set a smaller value than 10ms, and 30ms seems to be the natural
  // processing time on my macbook.  Keeping it constrained to that
  // feels reasonable to type to.
  static const char kGapIntervals[] PROGMEM = "AT+GAPINTERVALS=10,30,,\n";

  // Reset the device so that it picks up the above changes
  static const char kATZ[] PROGMEM = "ATZ\n";

  // Turn down the power level a bit
  static const char kPower[] PROGMEM = "AT+BLEPOWERLEVEL=-12\n";
  static PGM_P const configure_commands[] PROGMEM = {
    kEcho,
    kGapIntervals,
    kGapDevName,
    kHidEnOn,
    kPower,
    kATZ
  };

  uint8_t i;
  for (i = 0; i < sizeof(configure_commands) / sizeof(configure_commands[0]);
       ++i) {
    PGM_P cmd;
    memcpy_P(&cmd, configure_commands + i, sizeof(cmd));

    if (!at_command_P(cmd, resbuf, sizeof(resbuf), true)) {
      //printf("failed BLE command: %S: %s\n", cmd, resbuf);
      goto fail;
    }
  }
  
  state.configured = true;
  
  // Check connection status in a little while; allow the ATZ time
  // to kick in.
  state.last_connection_update = timer_read();
fail:
  return state.configured;
}

#if 1
static void set_connected(bool connected) {
  if (connected != state.is_connected) {
    if (connected) {
       print("Conn\n");
    } else {
      print("Dconn\n");
    }
    state.is_connected = connected;

    // TODO: if modifiers are down on the USB interface and
    // we cut over to BLE or vice versa, they will remain stuck.
    // This feels like a good point to do something like clearing
    // the keyboard and/or generating a fake all keys up message.
    // However, I've noticed that it takes a couple of seconds
    // for macOS to to start recognizing key presses after BLE
    // is in the connected state, so I worry that doing that
    // here may not be good enough.
  }
}
#endif

void adafruit_ble_task(void) {
  char resbuf[48];
  
  if (!state.configured && !adafruit_ble_enable_keyboard()) {
    return;
  }


  if (!termBuf.check()) {
    // there is an outstanding reponse
    // that has not been received we must consume
    // as much as is available
    int n=read_response(NULL, 0, false, 0);
  }


#if 0
  if (resp_buf.empty() && (state.event_flags & UsingEvents) &&
      digitalRead(AdafruitBleIRQPin)) {
    // Must be an event update
    if (at_command_P(PSTR("AT+EVENTSTATUS"), resbuf, sizeof(resbuf))) {
      uint32_t mask = strtoul(resbuf, NULL, 16);

      if (mask & BleSystemConnected) {
        set_connected(true);
      } else if (mask & BleSystemDisconnected) {
        set_connected(false);
      }
    }
  }

  if (timer_elapsed(state.last_connection_update) > ConnectionUpdateInterval) {
    bool shouldPoll = true;
    if (!(state.event_flags & ProbedEvents)) {
      // Request notifications about connection status changes.
      // This only works in SPIFRIEND firmware > 0.6.7, which is why
      // we check for this conditionally here.
      // Note that at the time of writing, HID reports only work correctly
      // with Apple products on firmware version 0.6.7!
      // https://forums.adafruit.com/viewtopic.php?f=8&t=104052
      if (at_command_P(PSTR("AT+EVENTENABLE=0x1"), resbuf, sizeof(resbuf))) {
        at_command_P(PSTR("AT+EVENTENABLE=0x2"), resbuf, sizeof(resbuf));
        state.event_flags |= UsingEvents;
      }
      state.event_flags |= ProbedEvents;

      // leave shouldPoll == true so that we check at least once
      // before relying solely on events
    } else {
      shouldPoll = false;
    }

    static const char kGetConn[] PROGMEM = "AT+GAPGETCONN";
    state.last_connection_update = timer_read();

    if (at_command_P(kGetConn, resbuf, sizeof(resbuf))) {
      set_connected(atoi(resbuf));
    }
  }

#ifdef SAMPLE_BATTERY
  // I don't know if this really does anything useful yet; the reported
  // voltage level always seems to be around 3200mV.  We may want to just rip
  // this code out.
  if (timer_elapsed(state.last_battery_update) > BatteryUpdateInterval &&
      resp_buf.empty()) {
    state.last_battery_update = timer_read();

    if (at_command_P(PSTR("AT+HWVBAT"), resbuf, sizeof(resbuf))) {
      state.vbat = atoi(resbuf);
    }
  }
#endif
#endif 
}

const int8_t MODLDIDX=19; 
const int8_t keyLdIdx[6] = { 25, 28, 31, 34, 37, 40 };

bool adafruit_ble_send_keys(uint8_t hid_modifier_mask, uint8_t *keys,
                            uint8_t nkeys) {

#if 1
  static char kstr[] = "AT+BLEKEYBOARDCODE=00-00-00-00-00-00-00-00\n";
  unsigned char ki;
  //dprintf("<- %d\n", nkeys);
  
  digit2AsciiByte(hid_modifier_mask, kstr[MODLDIDX], kstr[MODLDIDX+1]);
 
  //dprintf("$%u %u %u\n", hid_modifier_mask, kstr[MODLDIDX], kstr[MODLDIDX+1]);
  for (int i=0; i<nkeys; i++) {
    ki = keyLdIdx[i];
    digit2AsciiByte(keys[i], kstr[ki], kstr[ki+1]);
    //dprintf(",%u %u %u\n", keys[i], kstr[ki], kstr[ki+1]);
  }
  //  dprintf("%s", kstr);

  if (state.is_connected) return at_command(kstr, NULL, 0, false, 0);
  else return true;
#endif
  

    
  #if 0
    char cmdbuf[48];
  char fmtbuf[64];
  strcpy_P(fmtbuf,
	   PSTR("AT+BLEKEYBOARDCODE=  x-00-%02x-%02x-%02x-%02x-%02x-%02x\n"));


  dprintf(cmdbuf, sizeof(cmdbuf), fmtbuf, hid_modifier_mask,
	   keys[0],
	   nkeys >= 1 ? keys[1] : 0,
	   nkeys >= 2 ? keys[2] : 0,
	   nkeys >= 3 ? keys[3] : 0,
	   nkeys >= 4 ? keys[4] : 0,
	   nkeys >= 5 ? keys[5] : 0);
  return at_command(cmdbuf, NULL, 0, true, 0);
#endif
#if 0
      struct queue_item item;
  bool didWait = false;

  item.queue_type = QTKeyReport;
  item.key.modifier = hid_modifier_mask;
  item.added = timer_read();

  while (nkeys >= 0) {
    item.key.keys[0] = keys[0];
    item.key.keys[1] = 
    item.key.keys[2] = 
    item.key.keys[3] = 
    item.key.keys[4] = 
    item.key.keys[5] = 

    if (!send_buf.enqueue(item)) {
      if (!didWait) {
        //dprint("wait for buf space\n");
        didWait = true;
      }
      send_buf_send_one();
      continue;
    }

    if (nkeys <= 6) {
      return true;
    }

    nkeys -= 6;
    keys += 6;
  }
  return true;
#endif
  return true;
}

bool adafruit_ble_send_consumer_key(uint16_t keycode, int hold_duration) {
#if 0
  struct queue_item item;

  item.queue_type = QTConsumer;
  item.consumer = keycode;

  while (!send_buf.enqueue(item)) {
    send_buf_send_one();
  }
#endif
  return true;
}

#ifdef MOUSE_ENABLE
bool adafruit_ble_send_mouse_move(int8_t x, int8_t y, int8_t scroll,
                                  int8_t pan, uint8_t buttons) {
#if 0
  struct queue_item item;

  item.queue_type = QTMouseMove;
  item.mousemove.x = x;
  item.mousemove.y = y;
  item.mousemove.scroll = scroll;
  item.mousemove.pan = pan;
  item.mousemove.buttons = buttons;

  while (!send_buf.enqueue(item)) {
    send_buf_send_one();
  }
  return true;
#endif
}
#endif

uint32_t adafruit_ble_read_battery_voltage(void) {
  return state.vbat;
}

bool adafruit_ble_set_mode_leds(bool on) {
#if 0
  if (!state.configured) {
    return false;
  }

  // The "mode" led is the red blinky one
  at_command_P(on ? PSTR("AT+HWMODELED=1") : PSTR("AT+HWMODELED=0"), NULL, 0);

  // Pin 19 is the blue "connected" LED; turn that off too.
  // When turning LEDs back on, don't turn that LED on if we're
  // not connected, as that would be confusing.
  at_command_P(on && state.is_connected ? PSTR("AT+HWGPIO=19,1")
                                        : PSTR("AT+HWGPIO=19,0"),
               NULL, 0);
#endif
  return true;
}

// https://learn.adafruit.com/adafruit-feather-32u4-bluefruit-le/ble-generic#at-plus-blepowerlevel
bool adafruit_ble_set_power_level(int8_t level) {
#if 0
  char cmd[46];
  if (!state.configured) {
    return false;
  }
  snprintf(cmd, sizeof(cmd), "AT+BLEPOWERLEVEL=%d", level);
  return at_command(cmd, NULL, 0, false);
#endif
  return true;

}
