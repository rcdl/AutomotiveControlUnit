#include <Arduino.h>

#define PROP_SEG 1
#define SEG_1 7
#define PHASE_SEG_1 (PROP_SEG + SEG_1)
#define PHASE_SEG_2 7
#define BIT_TQ (1 + PHASE_SEG_1 + PHASE_SEG_2)
#define SJW 1

#define Fosc 16000000  // 16MHz
#define BITRATE 10000  // 500kbps
#define TIMEQUANTAPULSES (Fosc / ( BITRATE * BIT_TQ)) 
#define TIMERBASE (65536 - TIMEQUANTAPULSES)

// Constants
const int RX_PIN = 2;
const int TX_PIN = 3;

// Enum
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
// TODO

void setup() {
  // put your setup code here, to run once:
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(RX_PIN), edge_detection, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void edge_detection() {
  if (idle == ENABLED) {
    hard_sync = ENABLED;
  } else {
    resync = ENABLED;
  }
}

void bit_timing() {
  static BIT_TIMING_STATES state = BTL_SYNC;
  static int tq_count = 0;

  switch (state){
    case BTL_SYNC:
      // sample_point = DISABLED;
      hard_sync = DISABLED;
      resync = DISABLED;
      state = BTL_SEG1;
      break;

    case BTL_SEG1:
      tq_count++;
      if (hard_sync == ENABLED) {
        tq_count = 0;
        hard_sync = DISABLED;
      } else {
        if (resync == ENABLED) {
          tq_count = max(0, tq_count - SJW);
          resync = DISABLED;
        }
        if (tq_count >= PHASE_SEG_1) {
          tq_count = 0;
          sample_point = ENABLED;
          // write_point = DISABLED;
          state = BTL_SEG2;
        }*
      }
      break;

    case BTL_SEG2:
      tq_count++;
      if (hard_sync == ENABLED) {
        tq_count = 0;
        hard_sync = DISABLED;
        state = BTL_SEG1;
      } else {
        if (resync == ENABLED) {
          tq_count = min(PHASE_SEG_2 + 1, tq_count + SJW);
          resync = DISABLED;
        }
        if (tq_count == PHASE_SEG_2) {
          tq_count = 0;
          write_point = ENABLED;
          state = BTL_SYNC;
        }
        if (tq_count > PHASE_SEG_2) {
          tq_count = 0;
          state = BTL_SEG1;
        }
      }
      break;
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