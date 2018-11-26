#include <Arduino.h>
#include <math.h>

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

// CRC settings
#define CRC_DIVISOR 0x4599
unsigned int crc = 0;

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
FLAGS_VALUE crc_enable = DISABLED;
FLAGS_VALUE write_mode = DISABLED;


// Errors
// Empty for now


// Decoder/Encoder 
typedef struct Frame_fields {
  unsigned start_of_frame     : 1;
  unsigned id_standard        : 11;
  unsigned rtr                : 1;
  unsigned r0                 : 11;
  unsigned ide                : 1;
  unsigned dlc                : 4;
  unsigned long data1         : 32;
  unsigned long data2         : 32;
  unsigned long id_extended   : 18;
  unsigned srr                : 1;
  unsigned r1                 : 11;
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

int rtr_srr = 0;

// Tests - pins and flags
#define STANDARD_FRAME 0
#define EXTENDED_FRAME 1
#define DATA_FRAME 0
#define REMOTE_FRAME 1
unsigned test_crc = 0;

// Prototype
void edge_detection();                          // Callback to RX falling edges
void bit_timing();                              // Bit timing state machine
void sample();                                  // Sampling logic
void write();                                   // Writing logic
void update_crc(int ctx_bit);                   // Update the calculation of the CRC
void jackson(int sample_bit);                   // Encoder/Decoder function
void reset_frame();                             // Reset to 0 all fields of the frame
int check_stuffing(int sample_bit);             // Check the bit stuffing
void print_decoder(int rtr_srr, frame_union _frame);
union frame_union create_test_frame(unsigned ide, unsigned rtr, unsigned id_std, unsigned long ext, unsigned long data1, unsigned long data2);
void update_test_crc(unsigned long data, unsigned bits);


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

  //test_frame = create_test_frame(STANDARD_FRAME, DATA_FRAME, 0x0D, 100);
}

void loop() {
  // Where decoding/encoding will be handled. Next task assignment
  if (jackson_enable == ENABLED) {
    jackson_enable = DISABLED;
    jackson(sample_bit);
  }
}

void edge_detection() {
  if (idle == ENABLED) {
    // Enables hard_sync flag if decoder is idle
    hard_sync = ENABLED;
  } else {
    // Enables resync flag otherwise (decoder is busy)
    resync = ENABLED;
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
}

void sample() {
  sample_bit = digitalRead(RX_PIN);
  Serial.print("Just sampled bit: ");
  Serial.println(ctx_bit);
  jackson_enable = ENABLED;
}

void write() {
  digitalWrite(TX_PIN, write_bit);
  Serial.print("Just wrote bit: ");
  Serial.println(write_bit);
}

void jackson(int ctx_bit) {
  static DECODER_STATES state = DEFAULT_DECODER_STATE;
  static unsigned count = 0;

  if (stuffing) {
    int stuffing_state = check_stuffing(ctx_bit);
    if (stuffing_state == YES_STUFFING) {
      Serial.println("Stuffing");
      return;
    } else if (stuffing_state == STUFFING_ERROR) {
      stuffing = DISABLED;
      arbitration = DISABLED;
      crc_enable = DISABLED;
      count = 0;
      state = ACTIVE_ERROR;
    }
  }

  if (crc_enable == ENABLED) {
    update_crc(ctx_bit);
  }

  // Encoder related 
  // if (arbitration == ENABLED) {
  //   if (write_bit != ctx_bit) {
  //     write_mode = DISABLED;
  //   }
  // }else {
  //   if (write_bit != ctx_bit) {
  //     //??? error?
  //   }
  // }

  switch (state) {
    case IDLE:
      idle = ENABLED;
      if (ctx_bit == LOW) {
        // Start of Frame
        Serial.println("Current state: Start of Frame");
        reset_frame();
        frame.fields.start_of_frame = ctx_bit;
        idle = DISABLED;          // Turn off Idle flag
        arbitration = ENABLED;    // Turn on arbitration phase
        stuffing = ENABLED;       // Enable stuffing check
        crc_enable = ENABLED;     // Enable cyclic redundancy check
        state = ID_STANDARD;
      } else {
        Serial.println("Current state: Idle");
      }
      break;

    case ID_STANDARD:
      Serial.println("Current state: Identifier A");

      frame.fields.id_standard <<= 1;
      frame.fields.id_standard |= (ctx_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count >= 11) {
        count = 0;
        state = RTR_SRR;
      }
      break;

    case RTR_SRR:
      Serial.println("Current state: RTR(Standard) or SRR(Extended)");

      rtr_srr = ctx_bit;
      state = IDE;
      break;

    case IDE:
      Serial.println("Current state: Identifier extension bit (IDE)");

      frame.fields.ide = ctx_bit;
      if (ctx_bit == HIGH) { // extended frame
        frame.fields.srr = rtr_srr;
        state = ID_EXTENDED;
      } else {  // standard
        arbitration = DISABLED;
        frame.fields.rtr = rtr_srr;
        state = R0;
      }
      break;

    case ID_EXTENDED:
      Serial.println("Current state: Identifier B");

      frame.fields.id_extended <<= 1;
      frame.fields.id_extended |= (ctx_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count >= 18) {
        count = 0;
        state = RTR;
      }
      break;

    case RTR:
      Serial.println("Current state: RTR(Extended)");

      frame.fields.rtr = ctx_bit;
      state = R1;                
      break;

    case R1:
      Serial.println("Current state: Reserved bit 1");

      frame.fields.r1 = ctx_bit;
      arbitration = DISABLED;
      state = R0;
      break;

    case R0:
      Serial.println("Current state: Reserved bit 0");

      frame.fields.r0 = ctx_bit;
      state = DLC;
      break;

    case DLC:
      Serial.println("Current state: Data length code");

      frame.fields.dlc <<= 1;
      frame.fields.dlc |= (ctx_bit == HIGH ? 0x1 : 0x0);
      count++;
      if ( count >= 4 ) {
        if (frame.fields.rtr == LOW && frame.fields.dlc > 0) {
          state = DATA;
          count = 0;
        } else {  // if (frame.fields.rtr == HIGH || frame.fields.dlc == 0) 
          state = CRC;
          count = 0;
        }
      }
      break;

    case DATA:
      Serial.println("Current state: Data field");

      if (count < 32){
        frame.fields.data1 <<= 1;
        frame.fields.data1 |= (ctx_bit == HIGH ? 0x1 : 0x0);
      } else {
        frame.fields.data2 <<= 1;
        frame.fields.data2 |= (ctx_bit == HIGH ? 0x1 : 0x0);
      }
      count++;
      if(count >= frame.fields.dlc*8) {
        crc_enable = DISABLED;
        state = CRC;
        count = 0;
      }
      break;

    // From here needs review
    case CRC:
      Serial.println("Current state: Cyclic redundancy check");

      frame.fields.crc <<= 1;
      frame.fields.crc |= (ctx_bit == HIGH ? 0x1 : 0x0);
      count++;
      
      if (count >= 15) {
        state = CRC_DEL;
        count = 0;
      }

      break;

    case CRC_DEL:
      Serial.println("Current state: CRC delimiter");

      stuffing = DISABLED;
      frame.fields.crc_del = ctx_bit;

      if (frame.fields.crc_del == 1) state = ACK_SLOT;
      else state = ACTIVE_ERROR;  // Form error
      break;

    // From here stuffing is disabled
    case ACK_SLOT:
      Serial.println("Current state: Acknowledgement");
      
      frame.fields.ack_slot = ctx_bit;
      if (crc != frame.fields.crc) state = ACTIVE_ERROR;
      else if (frame.fields.ack_slot == 0) state = ACK_DEL;
      else {
        count = 0;
        stuffing = DISABLED;
        arbitration = DISABLED;
        state = ACTIVE_ERROR;
      }
      break;

    case ACK_DEL:
      frame.fields.ack_del = ctx_bit;
      if (frame.fields.ack_del != 1 ) {
        count = 0;
        stuffing = DISABLED;
        arbitration = DISABLED;
        state = ACTIVE_ERROR;
        }
      else state = _EOF;
      Serial.println("jackson state: ACK_DEL");
      break;

    case _EOF:
      frame.fields.eof <<= 1;
      frame.fields.eof |= (ctx_bit == HIGH ? 0x1 : 0x0);
      count++;
      if (count >= 7) {
        state = INTERMISSION;
        count = 0;
      }
      Serial.println("jackson state: EOF");
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
      Serial.println("jackson state: INTERMISSION");
      break;

    case ACTIVE_ERROR:
      count++; // verifico se sao recessivos?
      if (count >= 6) {
        count = 0;
        state = PASSIVE_ERROR;
      }
      break;

    case PASSIVE_ERROR:
      count++;
      if (ctx_bit == LOW || count >= 6) {
        count = 0;
        state = _EOF;
      }
      break;

  } // end of switch
  print_decoder(rtr_srr, frame);
}

void print_decoder(int rtr_srr, frame_union _frame) {
  Serial.print("SoF: ");
  Serial.println(_frame.fields.start_of_frame, BIN);
  Serial.print("ID: ");
  Serial.println(_frame.fields.id_standard, BIN);
  Serial.print("RTR_SRR: ");
  Serial.println(rtr_srr, BIN);
  Serial.print("IDE: ");
  Serial.println(_frame.fields.ide, BIN);
  Serial.print("ID: ");
  Serial.println(_frame.fields.id_extended, BIN);
  Serial.print("RTR: ");
  Serial.println(_frame.fields.rtr, BIN);
  Serial.print("R1: ");
  Serial.println(_frame.fields.r1, BIN); 
  Serial.print("R0: ");
  Serial.println(_frame.fields.r0, BIN);
  Serial.print("DLC: ");
  Serial.println(_frame.fields.dlc, BIN);
  Serial.print("DATA1: ");
  Serial.println(_frame.fields.data1, BIN);
  Serial.print("CRC: ");
  Serial.print(_frame.fields.crc, BIN);
  Serial.print("\tCRC_REG: ");
  Serial.println(crc);
  Serial.print("CRC_DEL: ");
  Serial.println(_frame.fields.crc_del, BIN);
  Serial.print("ACK_SLOT: ");
  Serial.println(_frame.fields.ack_slot, BIN);
  Serial.print("ACK_DEL: ");
  Serial.println(_frame.fields.ack_del, BIN);
  Serial.print("EOF: ");
  Serial.println(_frame.fields.eof, BIN);
  Serial.print("INTERMISSION: ");
  Serial.println(_frame.fields.intermission, BIN);
}

int check_stuffing(int ctx_bit) {
  // Pode dar errado depois de rodar uma vez, por conta do SOF, chamar a função na transição de SOF pode resolver
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
  crc = 0;
  rtr_srr = 0;
}

void update_crc(int ctx_bit) {
  int crc_next = ctx_bit ^ ((crc & 0x4000) >> 14);
  crc <<= 1;
  if (crc_next) {
    crc ^= CRC_DIVISOR;
  }
}

frame_union create_test_frame(unsigned ide, unsigned rtr, unsigned id_std, unsigned long id_ext, unsigned long data1, unsigned long data2) {
  frame_union test;

  test.fields.start_of_frame = 0;
  test.fields.ide = ide;
  test.fields.rtr = rtr;
  test.fields.r0 = 0;
  test.fields.r1 = 0;
  test.fields.data1 = data1;
  test.fields.data2 = data2;
  test.fields.dlc = ceil(data1/8.0) + ceil(data2/8.0);

  if (ide == STANDARD_FRAME) {
    test.fields.id_standard = id_std; 
    update_test_crc(test.fields.id_standard, 11);
    update_test_crc(test.fields.rtr, 1);
    update_test_crc(test.fields.ide, 1);
    update_test_crc(test.fields.r0, 1);
    update_test_crc(test.fields.dlc, 4);
    update_test_crc(test.fields.data1, test.fields.dlc*8);

  } else {
    test.fields.id_standard = id_std;
    test.fields.id_extended = id_ext;
    test.fields.srr = 1;

  }



  return test;
}

void update_test_crc(unsigned long data, unsigned bits) {
  unsigned bit = 0;
  for(int i = 0 ; i < bits; i++) {
    bit = data | 0x1;
    data >>= 1;
    int crc_next = bit ^ ((test_crc & 0x4000) >> 14);
    test_crc <<= 1;
    if (crc_next) {
      test_crc ^= CRC_DIVISOR;
    }
  }

}
