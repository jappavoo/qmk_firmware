#include "adafruit_ble.h"
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

  static void test_init(void);
  static void test_pin(int i, int p);

  void     CTS(bool v);
  uint8_t  RTS();
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

void MySWSerial::test_init(void) {
  dprint("MySWSerial::test_init");
#if 0
  pinMode(B5, PinDirectionOutput);
  pinMode(B7, PinDirectionOutput);
  pinMode(C7, PinDirectionOutput);
  pinMode(F1, PinDirectionOutput);
  pinMode(F0, PinDirectionOutput);
#endif  
}

void MySWSerial::test_pin(int i, int p) {

#if 0
  switch (i) {
  case 1:
    if (p) {
      digitalWrite(B5, PinLevelHigh);
      dprint("+B5:H\n");
    }else {
      digitalWrite(B5, PinLevelLow);
      dprint("-B5:L\n");
    }
    break;
  case 2:
    if (p) {
      digitalWrite(B7, PinLevelHigh);
      dprint("+B7:H\n");
    }else {
      digitalWrite(B7, PinLevelLow);
	dprint("-B7:L\n");
    }
    break;
  case 3:
    if (p) {
      digitalWrite(C7, PinLevelHigh);
      dprint("+C7:H\n");
    }else {
      digitalWrite(C7, PinLevelLow);
      dprint("-C7:L\n");
      }
    break;
  case 4:
    if (p) {
      digitalWrite(F1, PinLevelHigh);
      dprint("+F1:H\n");
    }else {
      digitalWrite(F1, PinLevelLow);
      dprint("-F1:L\n");
    }
    break;
  case 5:
    if (p) {
      digitalWrite(F0, PinLevelHigh);
      dprint("+F0:H\n");
    }else {
      digitalWrite(F0, PinLevelLow);
      dprint("-F0:L\n");
    }
    break;
  default:
    dprint("?");
  }
#endif
}

void MySWSerial::CTS(bool v) {
  if (v)
    digitalWrite(_cts_pin, HIGH);
  else
    digitalWrite(_cts_pin, LOW);
}
  
uint8_t MySWSerial::RTS() {
  return digitalRead(_rts_pin);
}


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

MySWSerial _mySerial(AdaFruitUARTBleRXPin, AdaFruitUARTBleTXPin,
		     AdaFruitUARTBleCTSPin, AdaFruitUARTBleRTSPin);

extern "C" {
  extern void test_init(void) {
    _mySerial.begin();
  }

  extern void test_pin(int i, int p) {
    switch (i) {
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
    default:
      dprint("?");
    }
  }
};

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


// The recv latency is relatively high, so when we're hammering keys quickly,
// we want to avoid waiting for the responses in the matrix loop.  We maintain
// a short queue for that.  Since there is quite a lot of space overhead for
// the AT command representation wrapped up in SDEP, we queue the minimal
// information here.

enum queue_type {
  QTKeyReport, // 1-byte modifier + 6-byte key report
  QTConsumer,  // 16-bit key code
#ifdef MOUSE_ENABLE
  QTMouseMove, // 4-byte mouse report
#endif
};

struct queue_item {
  enum queue_type queue_type;
  uint16_t added;
  union __attribute__((packed)) {
    struct __attribute__((packed)) {
      uint8_t modifier;
      uint8_t keys[6];
    } key;

    uint16_t consumer;
    struct __attribute__((packed)) {
      int8_t x, y, scroll, pan;
      uint8_t buttons;
    } mousemove;
  };
};

// Items that we wish to send
static RingBuffer<queue_item, 4> send_buf;
// Pending response; while pending, we can't send any more requests.
// This records the time at which we sent the command for which we
// are expecting a response.
static RingBuffer<uint16_t, 2> resp_buf;

static bool process_queue_item(struct queue_item *item, uint16_t timeout);


enum ble_system_event_bits {
  BleSystemConnected = 0,
  BleSystemDisconnected = 1,
  BleSystemUartRx = 8,
  BleSystemMidiRx = 10,
};

// The SDEP.md file says 2MHz but the web page and the sample driver
// both use 4MHz
#define SpiBusSpeed 4000000

#define SdepTimeout 150 /* milliseconds */
#define SdepShortTimeout 10 /* milliseconds */
#define SdepBackOff 25 /* microseconds */
#define BatteryUpdateInterval 10000 /* milliseconds */

static bool at_command(const char *cmd, char *resp, uint16_t resplen,
                       bool verbose, uint16_t timeout = SdepTimeout);
static bool at_command_P(const char *cmd, char *resp, uint16_t resplen,
                         bool verbose = false);


static void resp_buf_read_one(bool greedy) {
}

static void send_buf_send_one(uint16_t timeout = SdepTimeout) {
}

static void resp_buf_wait(const char *cmd) {
  bool didPrint = false;
  while (!resp_buf.empty()) {
    if (!didPrint) {
      dprintf("wait on buf for %s\n", cmd);
      didPrint = true;
    }
    resp_buf_read_one(true);
  }
}

static bool ble_init(void) {
#if 0
  state.initialized = false;
  state.configured = false;
  state.is_connected = false;

  pinMode(AdafruitBleIRQPin, PinDirectionInput);
  pinMode(AdafruitBleCSPin, PinDirectionOutput);
  digitalWrite(AdafruitBleCSPin, PinLevelHigh);


  // Perform a hardware reset
  pinMode(AdafruitBleResetPin, PinDirectionOutput);
  digitalWrite(AdafruitBleResetPin, PinLevelHigh);
  digitalWrite(AdafruitBleResetPin, PinLevelLow);
  _delay_ms(10);
  digitalWrite(AdafruitBleResetPin, PinLevelHigh);

  _delay_ms(1000); // Give it a second to initialize
#endif
  state.initialized = true;
  return state.initialized;
}

static inline uint8_t min(uint8_t a, uint8_t b) {
  return a < b ? a : b;
}

static bool read_response(char *resp, uint16_t resplen, bool verbose) {
    bool success = false;
  return success;
}

static bool at_command(const char *cmd, char *resp, uint16_t resplen,
                       bool verbose, uint16_t timeout) {
  return true;
}

bool at_command_P(const char *cmd, char *resp, uint16_t resplen, bool verbose) {
  auto cmdbuf = (char *)alloca(strlen_P(cmd) + 1);
  strcpy_P(cmdbuf, cmd);
  return at_command(cmdbuf, resp, resplen, verbose);
}

bool adafruit_ble_is_connected(void) {
  return state.is_connected;
}

bool adafruit_ble_enable_keyboard(void) {
#if 0
  char resbuf[128];

  if (!state.initialized && !ble_init()) {
    return false;
  }

  state.configured = false;

  // Disable command echo
  static const char kEcho[] PROGMEM = "ATE=0";
  // Make the advertised name match the keyboard
  static const char kGapDevName[] PROGMEM =
      "AT+GAPDEVNAME=" STR(PRODUCT) " " STR(DESCRIPTION);
  // Turn on keyboard support
  static const char kHidEnOn[] PROGMEM = "AT+BLEHIDEN=1";

  // Adjust intervals to improve latency.  This causes the "central"
  // system (computer/tablet) to poll us every 10-30 ms.  We can't
  // set a smaller value than 10ms, and 30ms seems to be the natural
  // processing time on my macbook.  Keeping it constrained to that
  // feels reasonable to type to.
  static const char kGapIntervals[] PROGMEM = "AT+GAPINTERVALS=10,30,,";

  // Reset the device so that it picks up the above changes
  static const char kATZ[] PROGMEM = "ATZ";

  // Turn down the power level a bit
  static const char kPower[] PROGMEM = "AT+BLEPOWERLEVEL=-12";
  static PGM_P const configure_commands[] PROGMEM = {
    kEcho,
    kGapIntervals,
    kGapDevName,
    kHidEnOn,
    kPower,
    kATZ,
  };

  uint8_t i;
  for (i = 0; i < sizeof(configure_commands) / sizeof(configure_commands[0]);
       ++i) {
    PGM_P cmd;
    memcpy_P(&cmd, configure_commands + i, sizeof(cmd));

    if (!at_command_P(cmd, resbuf, sizeof(resbuf))) {
      dprintf("failed BLE command: %S: %s\n", cmd, resbuf);
      goto fail;
    }
  }

  state.configured = true;

  // Check connection status in a little while; allow the ATZ time
  // to kick in.
  state.last_connection_update = timer_read();
fail:
  return state.configured;
#endif
  return false;
}

static void set_connected(bool connected) {
  if (connected != state.is_connected) {
    if (connected) {
      print("****** BLE CONNECT!!!!\n");
    } else {
      print("****** BLE DISCONNECT!!!!\n");
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

void adafruit_ble_task(void) {
#if 0
  char resbuf[48];

  if (!state.configured && !adafruit_ble_enable_keyboard()) {
    return;
  }
  resp_buf_read_one(true);
  send_buf_send_one(SdepShortTimeout);

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

static bool process_queue_item(struct queue_item *item, uint16_t timeout) {
  char cmdbuf[48];
  char fmtbuf[64];

  // Arrange to re-check connection after keys have settled
  state.last_connection_update = timer_read();

#if 1
  if (TIMER_DIFF_16(state.last_connection_update, item->added) > 0) {
    dprintf("send latency %dms\n",
            TIMER_DIFF_16(state.last_connection_update, item->added));
  }
#endif

  switch (item->queue_type) {
    case QTKeyReport:
      strcpy_P(fmtbuf,
          PSTR("AT+BLEKEYBOARDCODE=%02x-00-%02x-%02x-%02x-%02x-%02x-%02x"));
      snprintf(cmdbuf, sizeof(cmdbuf), fmtbuf, item->key.modifier,
               item->key.keys[0], item->key.keys[1], item->key.keys[2],
               item->key.keys[3], item->key.keys[4], item->key.keys[5]);
      return at_command(cmdbuf, NULL, 0, true, timeout);

    case QTConsumer:
      strcpy_P(fmtbuf, PSTR("AT+BLEHIDCONTROLKEY=0x%04x"));
      snprintf(cmdbuf, sizeof(cmdbuf), fmtbuf, item->consumer);
      return at_command(cmdbuf, NULL, 0, true, timeout);

#ifdef MOUSE_ENABLE
    case QTMouseMove:
      strcpy_P(fmtbuf, PSTR("AT+BLEHIDMOUSEMOVE=%d,%d,%d,%d"));
      snprintf(cmdbuf, sizeof(cmdbuf), fmtbuf, item->mousemove.x,
          item->mousemove.y, item->mousemove.scroll, item->mousemove.pan);
      if (!at_command(cmdbuf, NULL, 0, true, timeout)) {
        return false;
      }
      strcpy_P(cmdbuf, PSTR("AT+BLEHIDMOUSEBUTTON="));
      if (item->mousemove.buttons & MOUSE_BTN1) {
        strcat(cmdbuf, "L");
      }
      if (item->mousemove.buttons & MOUSE_BTN2) {
        strcat(cmdbuf, "R");
      }
      if (item->mousemove.buttons & MOUSE_BTN3) {
        strcat(cmdbuf, "M");
      }
      if (item->mousemove.buttons == 0) {
        strcat(cmdbuf, "0");
      }
      return at_command(cmdbuf, NULL, 0, true, timeout);
#endif
    default:
      return true;
  }
}

bool adafruit_ble_send_keys(uint8_t hid_modifier_mask, uint8_t *keys,
                            uint8_t nkeys) {
#if 0
  struct queue_item item;
  bool didWait = false;

  item.queue_type = QTKeyReport;
  item.key.modifier = hid_modifier_mask;
  item.added = timer_read();

  while (nkeys >= 0) {
    item.key.keys[0] = keys[0];
    item.key.keys[1] = nkeys >= 1 ? keys[1] : 0;
    item.key.keys[2] = nkeys >= 2 ? keys[2] : 0;
    item.key.keys[3] = nkeys >= 3 ? keys[3] : 0;
    item.key.keys[4] = nkeys >= 4 ? keys[4] : 0;
    item.key.keys[5] = nkeys >= 5 ? keys[5] : 0;

    if (!send_buf.enqueue(item)) {
      if (!didWait) {
        dprint("wait for buf space\n");
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
  return true;
#endif
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
}
