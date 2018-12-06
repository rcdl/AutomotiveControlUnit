#include <math.h>




typedef struct Std_frame_fields {
  unsigned start_of_frame     : 1;
  unsigned id_standard        : 11;
  unsigned rtr                : 1;
  unsigned ide                : 1;
  unsigned r0                 : 1;
  unsigned dlc                : 4;
  unsigned long data1         : 32;
  unsigned long data2         : 32;
  unsigned crc                : 15;
  unsigned crc_del            : 1;
  unsigned ack_slot           : 1;
  unsigned ack_del            : 1;
  unsigned eof                : 7;
  unsigned intermission       : 3;
} std_frame_fields;

union std_frame_union {
  char raw[14];
  std_frame_fields fields;
};

void update_test_crc(unsigned long data, unsigned bits);
std_frame_union create_test_stdframe(unsigned rtr, unsigned id, unsigned long data1, unsigned long data2);
unsigned test_crc;

std_frame_union create_test_stdframe(unsigned rtr, unsigned id, unsigned long data1, unsigned long data2) {
  std_frame_union test;

  test.fields.start_of_frame = 0;
  test.fields.id_standard = id;
  test.fields.ide = 0;
  test.fields.rtr = rtr;
  test.fields.r0 = 0;
  test.fields.data1 = data1;
  test.fields.data2 = data2;
  int dlc_len = ceil(data1/8.0) + ceil(data2/8.0);
  test.fields.dlc = dlc_len;
  
  update_test_crc(test.fields.id_standard, 11);
  update_test_crc(test.fields.rtr, 1);
  update_test_crc(test.fields.ide, 1);
  update_test_crc(test.fields.r0, 1);
  update_test_crc(test.fields.dlc, dlc_len);
  update_test_crc(test.fields.data1, dlc_len > 4 ? 32 : dlc_len*8);
  update_test_crc(test.fields.data1, dlc_len > 4 ? (dlc_len-4)*8 : 0);
  test.fields.crc = test_crc;
  test.fields.crc_del = 1;
  test.fields.ack_slot = 1;
  test.fields.ack_del = 1;
  test.fields.eof = 0x7F; //0111 1111
  test.fields.intermission = 0x7; // 0111


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
      test_crc ^= 0x4599;
    }
  }

}

char packet1[] = "0110011100100001000101010101010101010101010101010101010101010101010101010101010101000001000010100011011111111q";
char packet2[] = "011001110010000011111001010101010101010101010101010101010101010101010101010101011001100111011011111111q";
char packet4[] = "0110011100100000110110101010101010101010101010101010101010101100111001001101011111111q";
char packet6[] = "0110011100100000101110101010101010101010101001001011100000111011111111q";
char packet9[] = "011001110010000010000110010110101011011111111q";
char packet11[] = "011001110010100000110000100100010011011111111q";
char packet12[] = "0100010010011111000001000001111100100001000101010101010101010101010101010101010101010101010101010101010101011110111110101011011111111q";
char packet14[] = "0100010010011111000001000001111100101000001010111011111000011011111111q";
char packet17[] = "0110011100100001000101010101010101010101010101010101010101010101010101010101010101000001000010100011100000011111111q";
char packet18[] = "011001110010000100010101010101010101010101010101010101010101010101010101010101010100000000000011111111q";
char packet19[] = "01100111001000010001010101010101010101010101010101010101010101010101010101010101010000010000111000110100000011111111q";
char packet23[] = "0000010000011111000001000001000001000001001111101111101111101111101111101111101111101111101111101111101111101111101111101110101100001011111011111111q";

char packet1_nostuffing[] = "1100111001000010001010101010101010101010101010101010101010101010101010101010101010000000001010001111111";
char packet19_nostuffing[] = "110011100100001000101010101010101010101010101010101010101010101010101010101010101000000000111000110100000011111111";