#include <Arduino.h>

#define PROP_SEG 1
#define SEG_1 7
#define PHASE_SEG_1 (PROP_SEG + SEG_1)
#define PHASE_SEG_2 7
#define BIT_TQ (1 + PHASE_SEG_1 + PHASE_SEG_2)
#define SJW 1

#define RX_PIN 2
#define TX_PIN 3

enum BIT_VALUE {
  DOMINANT = 0,
  RECESSIVE
};

enum BIT_TIMING_STATES{
  BTL_SYNC = 0,
  BTL_SEG1,
  BTL_SEG2
};

// Writing and sampling
BIT_VALUE sample_point, writing_point;
// Flags
BIT_VALUE idle=RECESSIVE, hard_sync, resync;
// Errors
// TODO

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(RX_PIN), edge_detection, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void edge_detection() {
  if (idle == RECESSIVE) {
    hard_sync = RECESSIVE;
  } else {
    resync = RECESSIVE;
  }
}

void bit_timing() {
  static BIT_TIMING_STATES state = BTL_SYNC;
  static int tq_count = 0;

  switch (state){
    case BTL_SYNC:
      sample_point = RECESSIVE;
      hard_sync = DOMINANT;
      resync = DOMINANT;
      state = BTL_SEG1;
      break;
    case BTL_SEG1:
      tq_count++;
      if (hard_sync == RECESSIVE) {
        tq_count = 0;
        hard_sync = DOMINANT;
      }
      if (resync == RECESSIVE) {
        tq_count = max(0, tq_count - SJW);
        resync = DOMINANT;
      }
      if (tq_count >= PHASE_SEG_1) {
        tq_count = 0;
        sample_point = DOMINANT;
        writing_point = RECESSIVE;
        state = BTL_SEG2;
      }
      break;
    case BTL_SEG2:
      tq_count++;
      if (hard_sync == RECESSIVE) {
        tq_count = 0;
        hard_sync = DOMINANT;
        state = BTL_SEG1;
      } else {
        if (resync == RECESSIVE) {
          tq_count = min(PHASE_SEG_2 + 1, tq_count + SJW);
          resync = DOMINANT;
        }
        if (tq_count == PHASE_SEG_2) {
          tq_count = 0;
          writing_point = DOMINANT;
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
