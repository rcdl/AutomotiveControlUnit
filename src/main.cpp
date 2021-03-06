#include <Arduino.h>
#include <test_frames.h>

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
#define CRC_MASK (crc & 0x7FFF)
unsigned int crc = 0;

// Pins
const int RX_PIN = 3;  // From transceiver
const int TX_PIN = 5;  // To transceiver

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
  ERROR_FLAGS,
  ERROR_DELIMITER
};

// Writing and sampling
int sample_bit = HIGH;
int write_bit = HIGH;

// Flags
FLAGS_VALUE idle = ENABLED;
FLAGS_VALUE hard_sync = DISABLED;
FLAGS_VALUE resync = DISABLED;
FLAGS_VALUE jackson_enable = DISABLED;

// Errors
// Empty for now

// Decoder/Encoder 
typedef struct Frame_fields {
  unsigned start_of_frame     : 1;
  unsigned id_standard        : 11;
  unsigned srr                : 1;
  unsigned ide                : 1;
  unsigned long id_extended   : 18;
  unsigned rtr                : 1;
  unsigned r1                 : 11;
  unsigned r0                 : 11;
  unsigned dlc                : 4;
  unsigned long data1         : 32;
  unsigned long data2         : 32;
  unsigned crc                : 15;
  unsigned crc_del            : 1;
  unsigned ack_slot           : 1;
  unsigned ack_del            : 1;
  unsigned eof                : 7;
  unsigned intermission       : 3;
  unsigned active_error       : 6;
  unsigned passive_error      : 6;
} frame_fields;

union frame_union {
  char raw[19];
  frame_fields fields;
} frame;

int rtr_srr = 0;

// Tests - pins and flags
#define DATA_FRAME 0
#define REMOTE_FRAME 1
bool debug = true;
int debug_count = 0;
std_frame_union test_frame;
bool print_decoder_states = true;
bool print_once = true;
char *test_packet = packet19;
char *output_frame = packet19_nostuffing;
char ack_error[] = "ACK ERROR";
char crc_error[] = "CRC ERROR";
char form_error[] = "FORM ERROR";
char stuff_error[] = "STUFFING ERROR";
char no_error[] = "NO ERROR";
char *error = no_error;
const int WRITE_CTRL_PIN = 2;
FLAGS_VALUE manual_write = DISABLED;


// Prototype
void bit_timing();                              // Bit timing state machine
void edge_detection();                          // Callback to RX falling edges
void sample();                                  // Sampling logic
void write();                                   // Writing logic
void jackson(int sample_bit);                   // Encoder/Decoder function
void print_decoder(frame_union _frame);
int check_stuffing(int sample_bit);             // Check the bit stuffing
void reset_frame();                             // Reset to 0 all fields of the frame
void update_crc(int ctx_bit);                   // Update the calculation of the CRC
void get_from_write_frame();   // Next bit to send with/out stuffing
void write_manual_control();                    // Tests


// Timer settings
void init_timer() {
  TCCR1A = 0;                       // Sets timer to normal operation, pins OC1A and OC1B desconnected
  TCCR1B = 0;                       // Clean register
  //TCCR1B |= (1<<CS10);              // Sets prescaler to mode 1 (Fosc/1): CS12 = 0, CS11 = 0, CS10 = 1
  TCCR1B |= (1<<CS10) | (1<<CS12);  // Tests - Sets prescaler to mode 5 (Fosc/1024): CS12 = 1, CS11 = 0, CS10 = 1

  //TCNT1 = TIMERBASE;                // Offset, number of pulses to count
  //TCNT1 = 0xC2F7;                   // Tests - Offset to interrupt every second
  TCNT1 = 0xFFC7;                   // Tests - Offset to interrupt
  TIMSK1 |= (1 << TOIE1);           // Enables timer interrupt
}

ISR(TIMER1_OVF_vect)  // TIMER1 interrupt
{
  //TCNT1 = TIMERBASE;  // Resets timer
  //TCNT1 = 0xC2F7;     // Tests - Resets timer every second
  TCNT1 = 0xFFC7;     // Tests - Resets timer

  bit_timing();
}

// Main arduino
void setup() {
  Serial.begin(9600);
  //Serial.println("bom dia\n");
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  write();
  attachInterrupt(digitalPinToInterrupt(RX_PIN), edge_detection, FALLING);

  // Tests
  pinMode(WRITE_CTRL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WRITE_CTRL_PIN), write_manual_control, FALLING);
  init_timer();
  
}

void loop() {
  // Where decoding/encoding will be handled. Next task assignment
  if (jackson_enable == ENABLED) {
    jackson_enable = DISABLED;
    jackson(sample_bit);
  }
}

// Functions
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

void edge_detection() {
  if (idle == ENABLED) {
    // Enables hard_sync flag if decoder is idle
    hard_sync = ENABLED;
  } else {
    // Enables resync flag otherwise (decoder is busy)
    resync = ENABLED;
  }
}

void sample() {
  if (debug) {
    if (test_packet[debug_count] != 'q') {
      sample_bit = test_packet[debug_count] == '1' ? 1 : 0;
      debug_count++;
    } else {
      if (print_once) {
        print_decoder(frame);
        print_once = false;
      }
    }
  } else {
    sample_bit = digitalRead(RX_PIN);
  }
  if( print_once) {
    Serial.print("Just sampled bit: ");
    Serial.print(sample_bit);
  }
  jackson_enable = ENABLED;
}

void write() {
  digitalWrite(TX_PIN, write_bit);
  if (print_once) {
    Serial.print("Just wrote bit: ");
    Serial.println(write_bit);
  }
}

void jackson(int ctx_bit) {
  static DECODER_STATES state = DEFAULT_DECODER_STATE;
  static unsigned count = 0;
  static FLAGS_VALUE arbitration = DISABLED;
  static FLAGS_VALUE stuffing = DISABLED;
  static FLAGS_VALUE crc_enable = DISABLED;
  static FLAGS_VALUE read_mode = ENABLED;
  static FLAGS_VALUE conditional_write = DISABLED;
  static FLAGS_VALUE write_mode = DISABLED;
  
  // Arbitration handling
  if (write_mode == ENABLED && idle == DISABLED && ctx_bit != write_bit) {  // TODO: Handle Nack
    if (arbitration == ENABLED) write_mode = DISABLED;
    else {
      count = 0;
      stuffing = DISABLED;
      crc_enable = DISABLED;
      state = ERROR_FLAGS;  // Bit error
      read_mode = DISABLED;
      conditional_write = ENABLED;
    }
  }

  // Stuffing handling
  if (stuffing == ENABLED) {
    int stuffing_state = check_stuffing(ctx_bit);
    if (stuffing_state == YES_STUFFING) {
      Serial.println("-> Stuffed");
      read_mode = DISABLED;
    } else if (stuffing_state == STUFFING_ERROR) {
      count = 0;
      stuffing = DISABLED;
      crc_enable = DISABLED;
      arbitration = DISABLED;
      state = ERROR_FLAGS;
      error = stuff_error;
      Serial.println("Stuff ERROR");
      read_mode = DISABLED;
      conditional_write = ENABLED;
    }
  }

  if (read_mode == ENABLED) {  // Destuffed bits
    if (crc_enable == ENABLED) {
      update_crc(ctx_bit);
    }

    switch (state) {
      case IDLE:
        idle = ENABLED;
        if (ctx_bit == LOW) {
          // Start of Frame
          if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Start of Frame");
          reset_frame();
          frame.fields.start_of_frame = ctx_bit;
          idle = DISABLED;          // Turn off Idle flag
          arbitration = ENABLED;    // Turn on arbitration phase
          stuffing = ENABLED;       // Enable stuffing check
          crc_enable = ENABLED;     // Enable cyclic redundancy check
          state = ID_STANDARD;
        } else {
          if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Idle");
        }
        break;

      case ID_STANDARD:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Identifier A");

        frame.fields.id_standard <<= 1;
        frame.fields.id_standard |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        if (count >= 11) {
          count = 0;
          state = RTR_SRR;
        }
        break;

      case RTR_SRR:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: RTR(Standard) or SRR(Extended)");

        rtr_srr = ctx_bit;
        state = IDE;
        break;

      case IDE:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Identifier extension bit (IDE)");

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
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Identifier B");

        frame.fields.id_extended <<= 1;
        frame.fields.id_extended |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        if (count >= 18) {
          count = 0;
          state = RTR;
        }
        break;

      case RTR:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: RTR(Extended)");

        frame.fields.rtr = ctx_bit;
        state = R1;                
        break;

      case R1:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Reserved bit 1");

        frame.fields.r1 = ctx_bit;
        arbitration = DISABLED;
        state = R0;
        break;

      case R0:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Reserved bit 0");

        frame.fields.r0 = ctx_bit;
        state = DLC;
        break;

      case DLC:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Data length code");

        frame.fields.dlc <<= 1;
        frame.fields.dlc |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        if ( count >= 4 ) {
          if (frame.fields.rtr == LOW && frame.fields.dlc > 0) {
            state = DATA;
            count = 0;
          } else {  // if (frame.fields.rtr == HIGH || frame.fields.dlc == 0) 
            crc_enable = DISABLED;
            state = CRC;
            count = 0;
          }
        }
        break;

      case DATA:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Data field");

        if (count < 32){
          frame.fields.data1 <<= 1;
          frame.fields.data1 |= (ctx_bit == HIGH ? 0x1 : 0x0);
        } else {
          frame.fields.data2 <<= 1;
          frame.fields.data2 |= (ctx_bit == HIGH ? 0x1 : 0x0);
        }
        count++;
        if(count >= min(frame.fields.dlc*8, 64)) {
          crc_enable = DISABLED;
          state = CRC;
          count = 0;
        }
        break;

      // From here needs review
      case CRC:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Cyclic redundancy check");

        frame.fields.crc <<= 1;
        frame.fields.crc |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        
        if (count >= 15) {
          state = CRC_DEL;
          count = 0;
        }
        break;

      case CRC_DEL:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: CRC delimiter");

        stuffing = DISABLED;
        frame.fields.crc_del = ctx_bit;

        if (frame.fields.crc_del == HIGH) state = ACK_SLOT;
        else {
          state = ERROR_FLAGS;  // Form error
          error = form_error;
          Serial.println("Form ERROR");
          conditional_write = ENABLED;
        }
        break;

      // From here stuffing is disabled
      case ACK_SLOT:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Acknowledgement slot");
        
        frame.fields.ack_slot = ctx_bit;
        state = ACK_DEL;
        if (ctx_bit == HIGH) {
          state = ERROR_FLAGS;
          error = ack_error;
          Serial.println("ACK ERROR");
        }
        break;

      case ACK_DEL:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Acknowledgement delimiter");

        frame.fields.ack_del = ctx_bit;
        state = _EOF;
        if (CRC_MASK != frame.fields.crc) {
          state = ERROR_FLAGS;
          error = crc_error;
          Serial.print("CRC ERROR: ");
          Serial.print(CRC_MASK, HEX);
          Serial.print("-");
          Serial.println(frame.fields.crc, HEX);
        }
        break;

      case _EOF:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: End of frame");

        frame.fields.eof <<= 1;
        frame.fields.eof |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        if (count >= 7) {
          state = INTERMISSION;
          count = 0;
        }
        break;

      case INTERMISSION:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Intermission");
      
        frame.fields.intermission <<= 1;
        frame.fields.intermission |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        if(count == 3 && ctx_bit == LOW) {
          if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Start of Frame");
          reset_frame();
          frame.fields.start_of_frame = ctx_bit;
          idle = DISABLED;          // Turn off Idle flag
          arbitration = ENABLED;    // Turn on arbitration phase
          stuffing = ENABLED;       // Enable stuffing check
          crc_enable = ENABLED;     // Enable cyclic redundancy check
          state = ID_STANDARD;
        } else if (count >= 3) {
          //print_decoder(frame);
          count = 0;
          if(ctx_bit == LOW) {
            state = ID_STANDARD;
          } else {
            state = IDLE;
            idle = ENABLED;
            write_mode = DISABLED;
          }
        }
        break;

      case ERROR_FLAGS:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Error Flags ");
        
        frame.fields.active_error <<= 1;
        frame.fields.active_error |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++; 
        conditional_write = ENABLED;
        if (count >= 6) {
          count = 0;
          state = ERROR_DELIMITER;
        }
        break;

      case ERROR_DELIMITER:
        if(print_decoder_states & print_once) Serial.println("\t\t\t\tCurrent state: Error Delimiter");
        
        frame.fields.passive_error <<= 1;
        frame.fields.passive_error |= (ctx_bit == HIGH ? 0x1 : 0x0);
        count++;
        conditional_write = ENABLED;
        if (ctx_bit == HIGH && count >= 8) {
          count = 0;
          state = IDLE;
          // print_decoder(frame);
        }
        break;
      
    
    
    }
    //print_decoder(frame);
  }

  // May write
  if (state == IDLE && manual_write == ENABLED) {
    manual_write = DISABLED;
    write_mode = ENABLED;
  }

  Serial.print("\n\n");
  if (write_mode == ENABLED || conditional_write == ENABLED) {
    switch(state) {
      case IDLE:
      case ERROR_FLAGS:
        write_bit = LOW;
        break;
      case ID_STANDARD:
      case RTR_SRR:
      case IDE:
      case ID_EXTENDED:
      case RTR:
      case R1:
      case R0:
      case DLC:
      case DATA:
      case CRC:
        get_from_write_frame();
        break;
      case CRC_DEL:
      case ACK_DEL:
      case _EOF:
      case INTERMISSION:
      case ERROR_DELIMITER:
        write_bit = HIGH;
        break;
      case ACK_SLOT:
        Serial.print(CRC_MASK, BIN);
        Serial.print("\t");
        Serial.println(frame.fields.crc, BIN);
        write_bit = (CRC_MASK == frame.fields.crc) ? LOW : HIGH;
        break;
    }

    Serial.println("write_bit: " + String(write_bit));
  }

  read_mode = ENABLED;
  conditional_write = DISABLED;
}

void print_decoder(frame_union _frame) {
  Serial.print("SoF: ");
  Serial.println(_frame.fields.start_of_frame, BIN);
  Serial.print("ID A: ");
  Serial.println(_frame.fields.id_standard, HEX);
  Serial.print("SRR: ");
  Serial.println(_frame.fields.srr, DEC);
  Serial.print("IDE: ");
  Serial.println(_frame.fields.ide, DEC);
  Serial.print("ID B: ");
  Serial.println(_frame.fields.id_extended, HEX);
  Serial.print("RTR: ");
  Serial.println(_frame.fields.rtr, BIN);
  Serial.print("R1: ");
  Serial.println(_frame.fields.r1, BIN); 
  Serial.print("R0: ");
  Serial.println(_frame.fields.r0, BIN);
  Serial.print("DLC: ");
  Serial.println(_frame.fields.dlc, DEC);
  Serial.print("DATA1: ");
  Serial.println(_frame.fields.data1, HEX);
  Serial.print("DATA2: ");
  Serial.println(_frame.fields.data2, HEX);
  Serial.print("CRC: ");
  Serial.print(_frame.fields.crc, HEX);
  Serial.print("\tCRC_REG: ");
  Serial.println(CRC_MASK, HEX);
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
  Serial.println(); // </br>
  Serial.println(error);
  Serial.print("ERROR_FLAGS: ");
  Serial.println(_frame.fields.active_error, BIN);
  Serial.print("ERROR_DELIMITER: ");
  Serial.println(_frame.fields.passive_error, BIN);
}

int check_stuffing(int ctx_bit) {
  static int count = 1;
  static int last_bit = LOW;
  int stuffing_state;
  
  if (count < 5) stuffing_state = NO_STUFFING;
  else if (count == 5) stuffing_state = YES_STUFFING;
  else stuffing_state = STUFFING_ERROR;
  /*Serial.print(" stf_cnt: ");
  Serial.print(count);
  Serial.print(" stf_lst: ");
  Serial.print(last_bit);
  Serial.print(" smp_bt: ");
  Serial.print(ctx_bit);
  Serial.print(" stf_st: ");
  Serial.println(stuffing_state);*/

  if (last_bit == ctx_bit) count++;
  else count = 1;

  last_bit = ctx_bit;
  return stuffing_state;
}

void reset_frame() {
  memset(frame.raw, 0, 19);
  crc = 0;
  rtr_srr = 0;
  error = no_error;
}

void update_crc(int ctx_bit) {
  int crc_next = ctx_bit ^ ((crc & 0x4000) >> 14);
  crc <<= 1;
  if (crc_next) {
    crc ^= CRC_DIVISOR;
  }
}

void get_from_write_frame() {
  static int count = 0;
  // static int last_bit = HIGH;
  static int current_index = 0;
  static int next_bit_candidate = HIGH;
  static char bit_index[200];

  next_bit_candidate = output_frame[current_index] == '1' ? HIGH : LOW;
  if (count >= 5) {  // Stuffing
    write_bit = (write_bit == HIGH) ? LOW : HIGH;
    count = 1;
  } else {  // No stuffing
    if (write_bit == next_bit_candidate) {
      count++;
    } else {
      count = 1;
    }
    write_bit = next_bit_candidate;
    Serial.println(output_frame);
    for(int i = 0; i < current_index; i++){
      bit_index[i] = ' ';
    }
    bit_index[current_index] = '|';
    Serial.println(bit_index);
    current_index++;
  }

  // last_bit = write_bit;
}

void write_manual_control() {
  manual_write = ENABLED;
  Serial.println("manual_write ENABLED");
}
