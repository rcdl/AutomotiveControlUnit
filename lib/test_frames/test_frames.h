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

/*                                            id B              ide  srr   id A     sof
    10101010101010101010101010101010101010    101010101010101010 1    0  10101010101 0
    */
