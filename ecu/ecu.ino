
#define PROP_SEG 1
#define SEG_1 7
#define PHASE_SEG_1 (PROP_SEG + SEG_1)
#define PHASE_SEG_2 7
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

BIT_VALUE sample_point, writing_point;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(RX_PIN), bit_timing, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

void bit_timing() {
  static BIT_TIMING_STATES state = BTL_SYNC;
  static int tq_count = 0;
//  noInterrupts();
  switch (state){
    case BTL_SYNC:
      sample_point = RECESSIVE;
      state = BTL_SEG1;
      break;
    case BTL_SEG1:
      tq_count++;
      if (tq_count >= PHASE_SEG_1) {
        writing_point = RECESSIVE;
        state = BTL_SEG2;
        tq_count = 0;
        sample_point = DOMINANT;
      }
      break;
    case BTL_SEG2:
      tq_count++;
      if (tq_count >= PHASE_SEG_2) {
        state = BTL_SYNC;
        tq_count = 0;
        writing_point = DOMINANT;
      }
      break;
  }
//  interrupts();
}
