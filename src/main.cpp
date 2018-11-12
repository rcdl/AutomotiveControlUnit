#include <Arduino.h>

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

// Constants
const int RX_PIN = 3;
const int TX_PIN = 2;

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


// Tests
const int TQ_INDICATOR_PIN = 4;
const int HARDSYNC_PIN = 5;
const int SOFTSYNC_PIN = 6;
const int STATE_HIGH_PIN = 7;
const int STATE_LOW_PIN = 8;
const int IDLE_TEST_PIN = 11;
int tq_indicator = 0;
int hs_indicator = 0;
int ss_indicator = 0;

// Protótipo
void edge_detection();
void bit_timing();
void sample();
void write();
void testWriteState(BIT_TIMING_STATES target);

void init_timer() {
  TCCR1A = 0;                        //confira timer para operação normal pinos OC1A e OC1B desconectados
  TCCR1B = 0;                         //limpa registrador
  // TCCR1B |= (1<<CS10);                // configura prescaler para 1: CS12 = 0, CS11 = 0 e CS10 = 1
  TCCR1B |= (1<<CS10) | (1<<CS12);

  // TCNT1 = TIMERBASE;               // 0xFFFE = 65534, offset para contar apenas 2 pulsos
  TCNT1 = 0xC2F7;
  TIMSK1 |= (1 << TOIE1);             // habilita interrupcao do timer
}

ISR(TIMER1_OVF_vect)                              //interrupção do TIMER1
{
  // TCNT1 = TIMERBASE;                           // Renicia TIMER
  TCNT1 = 0xC2F7;
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

  // timer1.start();
}

void loop() {
  // timer1.update();
  // put your main code here, to run repeatedly:
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
  idle = digitalRead(IDLE_TEST_PIN) ? ENABLED : DISABLED;
  if (idle == ENABLED) {
    hard_sync = ENABLED;
    hs_indicator = !hs_indicator;
    digitalWrite(HARDSYNC_PIN, (hs_indicator) ? HIGH : LOW);
  } else {
    resync = ENABLED;
    ss_indicator = !ss_indicator;
    digitalWrite(SOFTSYNC_PIN, (ss_indicator) ? HIGH : LOW);
  }
}

void bit_timing() {
  static BIT_TIMING_STATES state = DEFAULT_BTL_STATE;  
  static int tq_count = 0;


  tq_indicator = !tq_indicator;
  digitalWrite(TQ_INDICATOR_PIN, (tq_indicator) ? HIGH : LOW);

  switch (state){
    case BTL_SYNC:
      hard_sync = DISABLED;
      resync = DISABLED;
      state = BTL_SEG1;
      testWriteState(state);
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
          state = BTL_SEG2;
          testWriteState(state);
        }
      }
      break;

    case BTL_SEG2:
      tq_count++;
      if (hard_sync == ENABLED) {
        tq_count = 0;
        hard_sync = DISABLED;
        state = BTL_SEG1;
        testWriteState(state);
      } else {
        if (resync == ENABLED) {
          tq_count = min(PHASE_SEG_2 + 1, tq_count + SJW);
          resync = DISABLED;
        }
        if (tq_count == PHASE_SEG_2) {
          tq_count = 0;
          write_point = ENABLED;
          state = BTL_SYNC;
          testWriteState(state);
        }
        if (tq_count > PHASE_SEG_2) {
          tq_count = 0;
          state = BTL_SEG1;
          testWriteState(state);
        }
      }
      break;
  }

  for(int i=0;i<10;i++){
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