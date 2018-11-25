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

// Stuffing settings
#define NO_STUFFING 0
#define YES_STUFFING 1
#define STUFFING_ERROR 2

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
  RTR,
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
FLAGS_VALUE jackson_enable = DISABLED;


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
  unsigned rtr                : 1;
  unsigned srr                : 1;
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
int check_crc();
void jackson(int sample_bit);
void reset_frame();
int check_stuffing(int sample_bit);

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
  // Where decoding/encoding will be handled. Next task assignment
  if (jackson_enable == ENABLED) {
    jackson_enable = DISABLED;
    jackson(sample_bit);
  }
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
          sample();
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
          write();
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
  sample_bit = digitalRead(RX_PIN);
  jackson_enable = ENABLED;
}

void write() {
  digitalWrite(TX_PIN, write_bit);
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

void jackson(int ctx_bit) {
  static DECODER_STATES state = DEFAULT_DECODER_STATE;
  static unsigned count = 0;
  static int rtr_srr;
  int stuffing_state;
  
  switch (state) {
    case IDLE:
      idle = ENABLED;
      if (ctx_bit == LOW) {
        reset_frame();
        frame.fields.start_of_frame = ctx_bit;
        idle = DISABLED;
        arbitration = ENABLED;
        stuffing = ENABLED;
        state = ID_STANDARD;
      }
      break;

    case ID_STANDARD:
      stuffing_state = check_stuffing(ctx_bit);
      if(stuffing_state == NO_STUFFING){
        frame.fields.id_standard <<= 1;
        frame.fields.id_standard |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        if (count >= 11) {
          count = 0;
          state = RTR_SRR;
        }
      } else if (stuffing_state == STUFFING_ERROR) {
        count = 0;
        state = ACTIVE_ERROR;
        //ToDo
      }
      break;

    case RTR_SRR:
      stuffing_state = check_stuffing(ctx_bit);
      if(stuffing_state == NO_STUFFING){
        rtr_srr = ctx_bit;
        state = IDE;
      } else if (stuffing_state == STUFFING_ERROR) {
        state = ACTIVE_ERROR;
        //ToDo
      }
      break;

    case IDE:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        frame.fields.ide = ctx_bit;
        if (ctx_bit == HIGH) { // extended frame
          frame.fields.srr = rtr_srr;
          state = ID_EXTENDED;
        } else {  // standard
          arbitration = DISABLED;
          frame.fields.rtr = rtr_srr;
          state = R0;
        }
      } else if (stuffing_state == STUFFING_ERROR) {
        state = ACTIVE_ERROR;
        //ToDo
      }
      break;

    case ID_EXTENDED:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        frame.fields.id_extended <<= 1;
        frame.fields.id_extended = (frame.fields.id_extended & 0x1) | (sample_bit == HIGH ? 0x1 : 0x0);
        count++;
        if (count >= 18) {
          count = 0;
          state = RTR;
        }
      } else if (stuffing_state == STUFFING_ERROR) {
        count = 0;
        state = ACTIVE_ERROR;
        //ToDo
      }
      
      break;

    case RTR:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        frame.fields.rtr = ctx_bit;
        state = R1;                
      } else if (stuffing_state == STUFFING_ERROR) {
        state = ACTIVE_ERROR;
        //ToDo                                
      }

    case R1:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        frame.fields.r1 = ctx_bit;
        state = R0;
      } else if (stuffing_state == STUFFING_ERROR) {
        state = ACTIVE_ERROR;
        //Todo
      }     
      break;

    case R0:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        frame.fields.r0 = ctx_bit;
        state = DLC;
      } else if (stuffing_state == STUFFING_ERROR) {
        state = ACTIVE_ERROR;
        //ToDo
      }
      break;

    case DLC:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        frame.fields.dlc <<= 1;
        frame.fields.dlc |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        if ( count >= 4 ) {
          if (frame.fields.rtr == LOW && frame.fields.dlc > 0) {
            state = DATA;
            count = 0;
          } else if (frame.fields.rtr == HIGH || frame.fields.dlc == 0) {
            state = CRC;
            count = 0;
          } // Else?
        }
      } else if (stuffing_state == STUFFING_ERROR) {
        count = 0;
        state = ACTIVE_ERROR;
        //ToDo
      }

      break;

    case DATA:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        if (count < 32){
          frame.fields.data1 <<= 1;
          frame.fields.data1 |= (ctx_bit == HIGH ? 0x1 : 0x0);
        } else {
          frame.fields.data2 <<= 1;
          frame.fields.data2 |= (ctx_bit == HIGH ? 0x1 : 0x0);
        }
        count++;
      } else if (stuffing_state == STUFFING_ERROR){
        //ToDo
      }

      if(count >= frame.fields.dlc*8) {
        state = CRC;
        count = 0;
      }
      break;

    case CRC:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        frame.fields.crc <<= 1;
        frame.fields.crc |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
      } else if (stuffing_state == STUFFING_ERROR){
        //ToDo
      }
      if (count >= 15) {
        state = CRC_DEL;
        count = 0;
      }
      break;

//From here stuffing disabled
    case CRC_DEL:
      stuffing = DISABLED;
      frame.fields.crc_del = ctx_bit;
      if (frame.fields.crc_del == 1) state = ACK_SLOT;
      else state = ACTIVE_ERROR;
      break;

    case ACK_SLOT:
      frame.fields.ack_slot = ctx_bit;
      if (frame.fields.ack_slot == 0) state = ACK_DEL;
      else state = ACTIVE_ERROR;
      break;

    case ACK_DEL:
      frame.fields.ack_del = ctx_bit;
      if (frame.fields.ack_del != 1 || check_crc() ) state = ACTIVE_ERROR;
      else state = _EOF;
      break;

    case _EOF:
      stuffing_state = check_stuffing(ctx_bit);
      if (stuffing_state == NO_STUFFING) {
        frame.fields.eof <<= 1;
        frame.fields.eof |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
      } else if (stuffing_state == STUFFING_ERROR){
        //ToDo
      }
      if (count >= 7) {
        state = INTERMISSION;
        count = 0;
      }
      break;

    case INTERMISSION:
      frame.fields.intermission <<= 1;
      frame.fields.intermission |= (ctx_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count == 3 && ctx_bit == 0){
        state = ID_STANDARD;
        count = 0;
      } else if (count == 3 && ctx_bit == 1) {
        state = IDLE;
        count = 0;
      }
      count = 0;
      break;

    case ACTIVE_ERROR:
      // :)
      break;

  } // end of switch

}


int check_stuffing(int ctx_bit) {
  static int count = 0;
  static int last_bit = HIGH;
  int stuffing_state;
  

  if (count < 5) stuffing_state = NO_STUFFING;
  else if (count == 5) stuffing_state = YES_STUFFING;
  else stuffing_state = STUFFING_ERROR;


  if (last_bit == ctx_bit) count++;
  else count = 1;

  last_bit = ctx_bit;
  return stuffing_state;

}

void reset_frame() {
  memset(frame.raw, 0, 19);
}

int check_crc() {
  return 0;
}