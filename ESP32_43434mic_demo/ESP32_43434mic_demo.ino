//inspired by: https://forums.adafruit.com/viewtopic.php?f=57&t=124924&p=625289&hilit=I2S#p625289
#include "driver/i2s.h"

#define AUDIO_RATE           44100

namespace {

const int sample_rate = 44100;

/* RX: I2S_NUM_1 */
i2s_config_t i2s_config_rx  = {
mode: (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
sample_rate: sample_rate,
bits_per_sample: I2S_BITS_PER_SAMPLE_32BIT, //(i2s_bits_per_sample_t)48,    // Only 8-bit DAC support
channel_format: I2S_CHANNEL_FMT_ONLY_RIGHT,   // 2-channels
communication_format: (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB),
intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,        // Interrupt level 1
  dma_buf_count: 8,                            // number of buffers, 128 max.
  dma_buf_len: 8                          // size of each buffer
};

i2s_pin_config_t pin_config_rx = {
bck_io_num: GPIO_NUM_26,
ws_io_num: GPIO_NUM_25,
data_out_num: I2S_PIN_NO_CHANGE,
data_in_num: GPIO_NUM_22
};
};


void setup()
{
  Serial.begin(115200);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(22, INPUT);

  i2s_driver_install(I2S_NUM_1, &i2s_config_rx, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pin_config_rx);
  i2s_zero_dma_buffer(I2S_NUM_1);
}

void loop() {
  int32_t mic_sample = 0;

  //read 24 bits of signed data into a 48 bit signed container
  if (i2s_pop_sample(I2S_NUM_1, (char*)&mic_sample, portMAX_DELAY) == 4) {

    //like: https://forums.adafruit.com/viewtopic.php?f=19&t=125101 ICS-43434 is 1 pulse late
    //The MSBit is actually a false bit (always 1), the MSB of the real data is actually the second bit

    //Porting a 23 signed number into a 32 bits we note sign which is '-' if bit 23 is '1'
    mic_sample = (mic_sample & 0x400000) ?
                 //Negative: B/c of 2compliment unused bits left of the sign bit need to be '1's
                 (mic_sample |= 0xFF800000) :
                 //Positive: B/c of 2compliment unused bits left of the sign bit need to be '0's
                 (mic_sample &= 0x7FFFFF);

    //B/c the MSB was late, the LSBit is past the sample window, for us LSBit is always zero
    mic_sample << 1; //scale back up to proper 3 byte size unless you don't care

    printBarForGraph(abs(mic_sample));
    Serial.print("\t\t\t\t");
    Serial.println(mic_sample);
  }
}

void printBarForGraph(int32_t value) {
  const uint8_t NUMCHARS = 8;
  const uint8_t SEGSPERCHAR = 8;

  //only using 3 bytes of this long
  int y = map(value, 0x0, 0x3FFFFF, 0, 8 * NUMCHARS);

  for (int fullBlocks = 0; fullBlocks < (y / 8); fullBlocks++) {
    Serial.print("█");
  }

  switch (y % 8) {
    case (0): break;
    case (1): Serial.print("▏"); break;
    case (2): Serial.print("▎"); break;
    case (3): Serial.print("▍"); break;
    case (4): Serial.print("▌"); break;
    case (5): Serial.print("▋"); break;
    case (6): Serial.print("▊"); break;
    case (7): Serial.print("▉"); break;
  }
  //for (uint8_t mtSpaces = 0; mtSpaces <= NUMCHARS - (y/8 + (y%8 == 0)?0:1); mtSpaces++) Serial.print(" ");
  Serial.flush();
  //Serial.println();
}
