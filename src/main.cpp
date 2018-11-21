#include <Arduino.h>

// Bit Timing settings
#define PROP_SEG 1
#define SEG_1 7
#define PHASE_SEG_1 (PROP_SEG + SEG_1)
#define PHASE_SEG_2 7
#define BIT_TQ (1 + PHASE_SEG_1 + PHASE_SEG_2)
#define SJW 1
#define DEFAULT_BTL_STATE BTL_SEG1
#define DEFAULT_DECODER_STATE IDLE
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

enum DECODER_STATES{
  IDLE = 0,
  ID_STANDARD,
  ID_EXTENDED,
  RTR_SRR,
  IDE,
  R0,
  R1,
  DLC,
  DATA,
  CRC,
  CRC_DEL,
  ACK_SLOT,
  ACK_DEL,
  _EOF,
  INTERMISSION,
  ACTIVE_ERROR,
  PASSIVE_ERROR
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
FLAGS_VALUE arbitration = DISABLED;
FLAGS_VALUE stuffing = DISABLED;

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

typedef struct Frame_fields {
  unsigned start_of_frame     : 1;
  unsigned  id_standard       : 11;
  unsigned long  id_extended  : 18;
  unsigned rtr_srr            : 1;
  unsigned ide                : 1;
  unsigned r0                 : 11;
  unsigned r1                 : 11;
  unsigned dlc                : 4;
  unsigned long data1         : 32;
  unsigned long data2         : 32;
  unsigned crc                : 15;
  unsigned crc_del            : 1;
  unsigned ack_slot           : 1;
  unsigned ack_del            : 1;
  unsigned eof                : 7;
  unsigned intermission       : 3;
} frame_fields;

union frame_union {
  char raw[19];
  frame_fields fields;
} frame;

// Prototype
void edge_detection();                          // Callback to RX falling edges
void bit_timing();                              // Bit timing state machine
void sample();                                  // Sampling logic
void write();                                   // Writing logic
void testWriteState(BIT_TIMING_STATES target);  // Tests - State pins logic
void decode();                                  // Decoder state machine
int check_crc();

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
  static int stuffing_count = 0;
  static int last_read;
  if (sample_point == ENABLED) {
    sample_bit = digitalRead(RX_PIN);
    if (stuffing == ENABLED) {
      if (last_read == sample_bit) stuffing_count++;
      else stuffing_count = 0;
    }
    if(stuffing_count == 5) sample_bit = !sample_bit;
    last_read = sample_bit;
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

void decode(){
  static DECODER_STATES state = DEFAULT_DECODER_STATE;
  static unsigned count = 0;
  

  switch (state) {
    case IDLE:
      if (sample_bit == LOW) {
        frame.fields.start_of_frame = sample_bit;
        state = ID_STANDARD;
      }
      break;

    case ID_STANDARD:
      idle = DISABLED;
      arbitration = ENABLED;
      stuffing = ENABLED;
      frame.fields.id_standard <<= 1;
      frame.fields.id_standard = (frame.fields.id_standard & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count == 11) state = RTR_SRR;
      break;

    case RTR_SRR:
      frame.fields.rtr_srr = sample_bit;
      if (frame.fields.ide == 0) state = IDE;
      else state = R1;
      break;

    case IDE:
      //arbitration = sample_bit?
      frame.fields.ide = sample_bit;
      if (frame.fields.ide == 0) state = R0;
      else state = ID_EXTENDED;
      break;

    case ID_EXTENDED:
      frame.fields.id_extended <<= 1;
      frame.fields.id_extended = (frame.fields.id_extended & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count == 29) {
        state = RTR_SRR;
        count = 0;
      }
      break;

    case R0:
      frame.fields.r0 = sample_bit;
      state = DLC;
      break;

    case R1:
      frame.fields.r1 = sample_bit;
      //arbitration = 0;
      state = R0;
      break;

    case DLC:
      frame.fields.dlc <<= 1;
      frame.fields.dlc = (frame.fields.dlc & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
      count++;
      if(count == 4 && frame.fields.rtr_srr == 0 && frame.fields.dlc > 0) {
        state = DATA;
        count = 0;
      } else {
        state = CRC;
        count = 0;
      }
      break;

    case DATA:
      if (count < 32){
        frame.fields.data1 <<= 1;
        frame.fields.data1 = (frame.fields.data1 & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
      } else {
        frame.fields.data2 <<= 1;
        frame.fields.data2 = (frame.fields.data2 & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
      }
      count++;
      if(count == frame.fields.dlc*8) {
        state = CRC;
        count = 0;
      }
      break;

    case CRC:
      frame.fields.crc <<= 1;
      frame.fields.crc = (frame.fields.crc & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count == 15) {
        state = CRC_DEL;
        count = 0;
      }
      break;

    case CRC_DEL:
      stuffing = DISABLED;
      //sample_stuff=0; ?
      frame.fields.crc_del = sample_bit;
      if (frame.fields.crc_del == 1) state = ACK_SLOT;
      else state = ACTIVE_ERROR;
      break;

    case ACK_SLOT:
      frame.fields.ack_slot = sample_bit;
      if (frame.fields.ack_slot == 0) state = ACK_DEL;
      else state = ACTIVE_ERROR;
      break;

    case ACK_DEL:
      frame.fields.ack_del = sample_bit;
      if (frame.fields.ack_del != 1 || check_crc() ) state = ACTIVE_ERROR;
      else state = _EOF;
      break;

    case _EOF:
      frame.fields.eof <<= 1;
      frame.fields.eof = (frame.fields.eof & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count == 7) {
        state = INTERMISSION;
        count = 0;
      }
      break;

    case INTERMISSION:
      frame.fields.intermission <<= 1;
      frame.fields.intermission = (frame.fields.intermission & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count == 3 && sample_bit == 0){
        state = ID_STANDARD;
        count = 0;
      } else if (count == 3 && sample_bit == 1) {
        state = IDLE;
        count = 0;
      }
      count = 0;
      break;

    case ACTIVE_ERROR:
      //not requested, empty for now
      break;

  } // end of switch


}