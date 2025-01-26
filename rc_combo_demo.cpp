//RSAXVC
//Program to transmit DSHOT600 and RC PWM using RPi SMI bus interface
#include <algorithm>
#include <fcntl.h>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "bcm2835_smi.h"

#define SMI_CLK 125e6
#define SMI_WRITE_STROBE_MASK 0x7F
#define SMI_BITS 16

#define DMA_MAX 2048 //Max physically contig from memalign()
#define DMA_BYTES_PER_CLOCK 2

#define DSHOT_BITS 16
#define DSHOT600_WRITE_STROBE_CYCLES 12
#define DSHOT600_DMA_PER_BIT 16 //must be < MASK, and DMA_PER_BIT * BITS must be < DMA_MAX
#define DSHOT600_DMA_PER_ZRO 6
#define DSHOT600_DMA_PER_ONE 12
//TODO: see if we can skip the tail transfers of the last bit,
//since the SMI seems to hold the last output value.
//Racing this may prove useful for BDSHOT
#define DSHOT600_DMA_COUNT (int)(DSHOT600_DMA_PER_BIT * DSHOT_BITS)
#define DSHOT600_DMA_TX_TIME ((DSHOT600_DMA_COUNT) * (DSHOT600_WRITE_STROBE_CYCLES) / SMI_CLK)
#define DSHOT600_DMA_TX_TIME_US (DSHOT600_DMA_TX_TIME * 1e6)

#define PWM_WRITE_STROBE_CYCLES SMI_WRITE_STROBE_MASK
#define PWM_RC_MIN_US 1000
#define PWM_RC_MAX_US 2000
#define PWM_RC_SPAN_US (PWM_RC_MAX_US - PWM_RC_MIN_US)
#define PWM_MAX_DMA_COUNT 2048
#define PWM_HZ 50.0
#define PWM_TX_TIME_US (1e6/PWM_HZ)

#define MAX_DMA_COUNT (((PWM_MAX_DMA_COUNT) > (DSHOT600_DMA_COUNT)) ? (PWM_MAX_DMA_COUNT):(DSHOT600_DMA_COUNT))

static void fail(const char *msg) {
  perror(msg);
  exit(1);
}

static void print_smi_settings(struct smi_settings *settings) {
/*
  printf("max_dma_count: %u\n", MAX_DMA_COUNT);
  printf("dshot600_dma_tx_time: %fus\n", DSHOT600_DMA_TX_TIME_US);
  printf("width: %d\n", settings->data_width);
  printf("pack: %c\n", settings->pack_data ? 'Y' : 'N');
  printf("read setup: %d, strobe: %d, hold: %d, pace: %d\n", settings->read_setup_time, settings->read_strobe_time, settings->read_hold_time, settings->read_pace_time);
  printf("write setup: %d, strobe: %d, hold: %d, pace: %d\n", settings->write_setup_time, settings->write_strobe_time, settings->write_hold_time, settings->write_pace_time);
  printf("dma enable: %c, passthru enable: %c\n", settings->dma_enable ? 'Y':'N', settings->dma_passthrough_enable ? 'Y':'N');
  printf("dma threshold read: %d, write: %d\n", settings->dma_read_thresh, settings->dma_write_thresh);
  printf("dma panic threshold read: %d, write: %d\n", settings->dma_panic_read_thresh, settings->dma_panic_write_thresh);
*/
}

class smi_sender {
public:
	smi_sender();
	~smi_sender();

	void configure_dshot600();
	void configure_pwm();

	void smi_send_pwmTicks(uint32_t ticks[SMI_BITS]);
	void smi_send_pwmTicks_all(uint32_t ticks);
	void smi_send_pwmMicros(uint16_t micros[SMI_BITS]);
	void smi_send_pwmMicros_all(uint16_t micros);
	void smi_send_dshot600(uint16_t vals[SMI_BITS]);
	void smi_send_dshot600_all(uint16_t vals);
private:
	int fd;
	uint16_t * dma_buf;
	struct smi_settings settings;
	void configure_smi();
	void logDmaBuf(uint16_t count);
};

smi_sender::smi_sender(){
  fd = open("/dev/smi", O_RDWR);
  if (fd < 0) fail("cant open");
  int ret = ioctl(fd, BCM2835_SMI_IOC_GET_SETTINGS, &settings);
  if (ret != 0) fail("ioctl 1");

  //Shouldn't matter until we do BDSHOT
  settings.read_setup_time = 10;
  settings.read_strobe_time = 20;
  settings.read_hold_time = 40;
  settings.read_pace_time = 80;

  //Turn these off
  settings.write_setup_time = 0;
  settings.write_hold_time = 0;
  settings.write_pace_time = 0;

  //Configure these one since we won't change them
  settings.pack_data = true;
  settings.dma_enable = 1;
  settings.data_width = SMI_WIDTH_16BIT;

  //We need physically contiguous memory
  //We can get some by not crossing a page boundary
  dma_buf = (uint16_t*)memalign(4096, MAX_DMA_COUNT * DMA_BYTES_PER_CLOCK);

  //Verify we are not crossing a page boundary
  static_assert(MAX_DMA_COUNT * DMA_BYTES_PER_CLOCK <= 4096);
}

smi_sender::~smi_sender(){
  free(dma_buf);
  dma_buf = NULL;
}

void smi_sender::logDmaBuf(uint16_t count){
  printf("Log:\n");
  for(unsigned i = 0; i < count;){
    uint16_t val = dma_buf[i];
    unsigned same;
    for(same = i + 1; same < count && dma_buf[same] == val; same++);
    printf("\t0x%04x x %u\n", val, same - i);
    i = same;
  }
}

void smi_sender::configure_smi(void){
  int ret = ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS, &settings);
  if (ret != 0) fail("ioctl 2");
  print_smi_settings(&settings);
}

void smi_sender::configure_dshot600(){
  settings.write_strobe_time = DSHOT600_WRITE_STROBE_CYCLES;

  configure_smi();

  //Pre-fill the dma buffer for the static parts of the bits
  for(int bit = 0; bit < DSHOT_BITS; bit++){
    uint16_t * bitBuf = dma_buf + DSHOT600_DMA_PER_BIT * bit;
    for(int dma = 0; dma < DSHOT600_DMA_PER_ZRO; ++dma)
      bitBuf[dma] = 0xFFFF;
    //Skip over this part of each bit - it's where the zero/one distinction is made
    for(int dma = DSHOT600_DMA_PER_ONE; dma < DSHOT600_DMA_PER_BIT; ++dma)
      bitBuf[dma] = 0x0000;
  }
}

void smi_sender::configure_pwm(void){
  settings.write_strobe_time = PWM_WRITE_STROBE_CYCLES;
  configure_smi();
}

static void wordset(uint16_t * output, uint16_t word, size_t count){
  while(count){
    *output++ = word;
    count--;
  }
}

static uint32_t findSmallestGreaterVal(uint32_t vals[SMI_BITS], uint16_t min){
  uint32_t best = 0xFFFFF;

  for(int i = 0; i < SMI_BITS; ++i){
    if(vals[i] > min){
      best = std::min(best, vals[i]);
    }
  }
  return best;
}

static int findMaxVal(uint32_t vals[SMI_BITS]){
  int bestIdx = -1;
  uint32_t best = 0;

  for(int i = 0; i < SMI_BITS; ++i){
    best = std::max(best, vals[i]);
  }
  return best;
}

static uint16_t packPwmWord(uint32_t ticks[SMI_BITS], uint32_t limit){
  uint16_t ret = 0;
  for(int i = 0; i < SMI_BITS; ++i){
    ret |= ((int)(ticks[i] > limit) << i);
  }
  return ret;
}

void smi_sender::smi_send_pwmTicks(uint32_t ticks[SMI_BITS]){
  //uint32_t maxDmaTicks = std::min(PWM_MAX_DMA_COUNT - 1, findMaxVal(ticks));
  uint32_t maxDmaTicks = PWM_MAX_DMA_COUNT;
  uint32_t dmaTick = 0;
  while(dmaTick < maxDmaTicks){
    uint32_t next = std::min((uint32_t)PWM_MAX_DMA_COUNT, findSmallestGreaterVal(ticks, dmaTick));
    uint16_t word = packPwmWord(ticks, dmaTick);
    wordset(dma_buf + dmaTick, word, next - dmaTick);
    dmaTick = next;
  }

  write(fd, dma_buf, (maxDmaTicks-1) * DMA_BYTES_PER_CLOCK);
  auto dma_tx_time_us = maxDmaTicks * (1e6 * PWM_WRITE_STROBE_CYCLES / SMI_CLK);
  if(dma_tx_time_us < PWM_TX_TIME_US){
    usleep(PWM_TX_TIME_US - dma_tx_time_us);
  }
}

static uint16_t bitslice(unsigned bit, const uint16_t vals[SMI_BITS]){
  //If this is a bottleneck, 8x8 bitboard rotation is much faster.
  uint16_t ret = 0;
  bit = 1 << bit;
  for(unsigned i = 0; i < SMI_BITS; ++i){
    if(vals[i] & bit){
      ret |= 1 << i;
    }
  }
  return ret;
}

void smi_sender::smi_send_dshot600(uint16_t vals[SMI_BITS]){
  for(int bit = DSHOT_BITS - 1; bit >= 0; bit--){
    uint16_t * bitBuf = dma_buf + DSHOT600_DMA_PER_BIT * bit;
    uint16_t slice = bitslice(bit, vals);
    wordset(bitBuf + DSHOT600_DMA_PER_ZRO, slice, DSHOT600_DMA_PER_ONE - DSHOT600_DMA_PER_ZRO);
  }
  write(fd, dma_buf, DSHOT600_DMA_COUNT * DMA_BYTES_PER_CLOCK);
}

void smi_sender::smi_send_pwmTicks_all(uint32_t val){
  uint32_t vals[SMI_BITS];
  for(int i = 0; i < SMI_BITS; ++i){
    vals[i] = val;
  }
  smi_send_pwmTicks(vals);
}

static uint32_t rintu32(double x){
  long long t = llrint(x);
  if(t < 0) return 0;
  if(t > 0xFFFFFFFF) return 0xFFFFFFFF;
  return t;
}

void smi_sender::smi_send_pwmMicros(uint16_t micros[SMI_BITS]){
  uint32_t ticks[SMI_BITS];
  for(int i = 0; i < SMI_BITS; ++i){
    ticks[i] = rintu32(micros[i] * 1e6 * PWM_WRITE_STROBE_CYCLES / SMI_CLK);
  }
  smi_send_pwmTicks(ticks);
}

void smi_sender::smi_send_pwmMicros_all(uint16_t micros){
  uint16_t micross[SMI_BITS];
  for(int i = 0; i < SMI_BITS; ++i){
    micross[i] = micros;
  }
  smi_send_pwmMicros(micross);
}

void smi_sender::smi_send_dshot600_all(uint16_t val){
  uint16_t vals[SMI_BITS];
  for(int i = 0; i < SMI_BITS; ++i){
    vals[i] = val;
  }
  smi_send_dshot600(vals);
}


int main(int argc, char **argv) {

  smi_sender sender;

  while(1){
#if 1
    sender.configure_dshot600();
    for(unsigned throttle11 = 0; throttle11 < 2000; throttle11++){
      unsigned telem1 = 0;
      unsigned upper12 = (throttle11 << 1) | telem1;
      unsigned crc4 = (upper12 ^ (upper12 >> 4) ^ (upper12 >> 8)) & 0x0F;
      uint16_t dshot = (upper12 << 4) | crc4;
      printf("Sending DSHOT dsThrottle=%u word=0x%04x\n", throttle11, dshot);
      sender.smi_send_dshot600_all(dshot);
      usleep(1000);
    }
#endif

#if 1
    sender.configure_pwm();
    for(int pwm = 0; pwm < 100; ++pwm){
      int pwm_us = 1000 + 10 * pwm;
      printf("Sending PWM %ius\n", pwm_us);
      sender.smi_send_pwmMicros_all(pwm_us);
      //sender.smi_send_pwmTicks_all(2022);
    }
#endif
  }
}
