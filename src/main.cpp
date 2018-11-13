#include <Arduino.h>

// Bit Timing settings
#define PROP_SEG 1
#define SEG_1 7
#define PHASE_SEG_1 (PROP_SEG + SEG_1)
#define PHASE_SEG_2 7
#define BIT_TQ (1 + PHASE_SEG_1 + PHASE_SEG_2)
#define SJW 1
#define DEFAULT_BTL_STATE BTL_SEG1
#define Fosc 16000000  // 16MHz
#define BITRATE 500000  // 500kbps
#define TIMEQUANTAPULSES (Fosc / ( BITRATE * BIT_TQ))
#define TIMERBASE (65536 - TIMEQUANTAPULSES)

// Pins
const int RX_PIN = 3;  // From transceiver
const int TX_PIN = 2;  // To transceiver

// Enums
enum FLAGS_VALUE {
  DISABLED = 0,
  ENABLED
};

enum BIT_TIMING_STATES{
  BTL_SYNC = 0,
  BTL_SEG1,
  BTL_SEG2
};

// Writing and sampling
FLAGS_VALUE sample_point = DISABLED;
FLAGS_VALUE write_point = DISABLED;
int sample_bit = HIGH;
int write_bit = HIGH;

// Flags
FLAGS_VALUE idle = ENABLED;
FLAGS_VALUE hard_sync = DISABLED;
FLAGS_VALUE resync = DISABLED;

// Errors
// Empty for now

// Tests - pins and flags
const int TQ_INDICATOR_PIN = 4;
const int HARDSYNC_PIN = 5;
const int SOFTSYNC_PIN = 6;
const int STATE_HIGH_PIN = 7;
const int STATE_LOW_PIN = 8;
const int IDLE_TEST_PIN = 11;
int tq_indicator = 0;
int hs_indicator = 0;
int ss_indicator = 0;

// Prototype
void edge_detection();                          // Callback to RX falling edges
void bit_timing();                              // Bit timing state machine
void sample();                                  // Sampling logic
void write();                                   // Writing logic
void testWriteState(BIT_TIMING_STATES target);  // Tests - State pins logic

// Timer configuration
void init_timer() {
  TCCR1A = 0;                       // Sets timer to normal operation, pins OC1A and OC1B desconnected
  TCCR1B = 0;                       // Clean register
  // TCCR1B |= (1<<CS10);              // Sets prescaler to mode 1 (Fosc/1): CS12 = 0, CS11 = 0, CS10 = 1
  TCCR1B |= (1<<CS10) | (1<<CS12);  // Tests - Sets prescaler to mode 5 (Fosc/1024): CS12 = 1, CS11 = 0, CS10 = 1

  TCNT1 = TIMERBASE;                // Offset, number of pulses to count
  TCNT1 = 0xC2F7;                   // Tests - Offset to interrupt every second
  TIMSK1 |= (1 << TOIE1);           // Enables timer interrupt
}

ISR(TIMER1_OVF_vect)  // TIMER1 interrupt
{
  // TCNT1 = TIMERBASE;  // Resets timer
  TCNT1 = 0xC2F7;     // Tests - Resets timer
  bit_timing();
}

void setup() {
  Serial.begin(9600);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(RX_PIN), edge_detection, FALLING);
  init_timer();

  // Tests
  pinMode(TQ_INDICATOR_PIN, OUTPUT);
  pinMode(HARDSYNC_PIN, OUTPUT);
  pinMode(SOFTSYNC_PIN, OUTPUT);
  pinMode(STATE_HIGH_PIN, OUTPUT);
  pinMode(STATE_LOW_PIN, OUTPUT);
  pinMode(IDLE_TEST_PIN, INPUT);
  digitalWrite(TQ_INDICATOR_PIN, (tq_indicator) ? HIGH : LOW);
  digitalWrite(HARDSYNC_PIN, (hs_indicator) ? HIGH : LOW);
  digitalWrite(SOFTSYNC_PIN, (ss_indicator) ? HIGH : LOW);
  testWriteState(DEFAULT_BTL_STATE);
}

void loop() {
  // Where decoding/encoding/writing/sampling will be handled. Next task assignment
  // if (write_point) {
  // encode();
  write();
  // }
  // if (sample_point) {
  sample();
  // decode();
  // }
}

void edge_detection() {
  // Tests - idle flag workaround. The decoder controls the idle flag and it's not implemented yet.
  idle = digitalRead(IDLE_TEST_PIN) ? ENABLED : DISABLED;

  if (idle == ENABLED) {
    // Enables hard_sync flag if decoder is idle
    hard_sync = ENABLED;
    hs_indicator = !hs_indicator;                             // Tests - Toggle pin state
    digitalWrite(HARDSYNC_PIN, (hs_indicator) ? HIGH : LOW);  // Tests - Write indicator to pin
  } else {
    // Enables resync flag otherwise (decoder is busy)
    resync = ENABLED;
    ss_indicator = !ss_indicator;                             // Tests - Toggle pin sate
    digitalWrite(SOFTSYNC_PIN, (ss_indicator) ? HIGH : LOW);  // Tests - Write indicator to pin
  }
}

void bit_timing() {
  static BIT_TIMING_STATES state = DEFAULT_BTL_STATE;
  static int tq_count = 0;  // Counts from 0 to segment size

  switch (state){
    case BTL_SYNC:
      // Sync segment - always 1 tq long
      // No sync flags handling, syncs occurring here means that the controller is synchronized
      hard_sync = DISABLED;
      resync = DISABLED;
      state = BTL_SEG1;
      break;

    case BTL_SEG1:
      // T1 segment - defaults to (Propagation + Phase_1) long
      tq_count++;
      if (hard_sync == ENABLED) {
        // Hard_sync handling
        tq_count = 0;
        hard_sync = DISABLED;
      } else {
        if (resync == ENABLED) {
          // Soft sync handling
          // Extends duration by minimun between phase_error and SJW
          tq_count = max(0, tq_count - SJW);
          resync = DISABLED;
        }
        if (tq_count >= PHASE_SEG_1) {
          // End of segment handling
          tq_count = 0;
          sample_point = ENABLED;
          state = BTL_SEG2;
        }
      }
      break;

    case BTL_SEG2:
      tq_count++;
      if (hard_sync == ENABLED) {
        // Hard sync handling
        tq_count = 0;
        hard_sync = DISABLED;
        state = BTL_SEG1;
      } else {
        if (resync == ENABLED) {
          // Soft sync handling
          // Shrinks duration by minimun between phase_error and SJW
          tq_count = min(PHASE_SEG_2 + 1, tq_count + SJW);
          resync = DISABLED;
        }
        if (tq_count == PHASE_SEG_2) {
          // Default end of segment handling
          tq_count = 0;
          write_point = ENABLED;
          state = BTL_SYNC;
        }
        if (tq_count > PHASE_SEG_2) {
          // Desynchronized end of segment, it should have been on sync state
          tq_count = 0;
          state = BTL_SEG1;
        }
      }
      break;
  }

  tq_indicator = !tq_indicator;                                 // Tests - Toggle pin state
  digitalWrite(TQ_INDICATOR_PIN, (tq_indicator) ? HIGH : LOW);  // Tests - Write indicator to pin
  testWriteState(state);                                        // Tests - Write state to pins

  // Tests - To serialplotter
  // Repeats for visibility
  // Prints one line with separated values -> separated plots
  // Adds different offsets so plots dont overlap
  for(int i=0;i<10;i++) {
    Serial.print(state);
    Serial.print("\t");
    Serial.print(tq_indicator + 4);
    Serial.print("\t");
    Serial.print(hs_indicator + 7);
    Serial.print("\t");
    Serial.println(ss_indicator + 10);
  }

}

void sample() {
  if (sample_point == ENABLED) {
    sample_bit = digitalRead(RX_PIN);
    sample_point = DISABLED;
  }
}

void write() {
  if (write_point == ENABLED) {
    digitalWrite(TX_PIN, write_bit);
    write_point = DISABLED;
  }
}

void testWriteState(BIT_TIMING_STATES target) {
  if (target == BTL_SYNC) {
    digitalWrite(STATE_HIGH_PIN, LOW);
    digitalWrite(STATE_LOW_PIN, LOW);
  }
  else if (target == BTL_SEG1) {
    digitalWrite(STATE_HIGH_PIN, LOW);
    digitalWrite(STATE_LOW_PIN, HIGH);
  }
  else if (target == BTL_SEG2) {
    digitalWrite(STATE_HIGH_PIN, HIGH);
    digitalWrite(STATE_LOW_PIN, LOW);
  }
}