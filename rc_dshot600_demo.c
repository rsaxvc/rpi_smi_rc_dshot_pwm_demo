//RSAXVC
//Program to transmit DSHOT600 using RPi SMI bus interface
#include <fcntl.h>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include "bcm2835_smi.h"

static void fail(const char *msg) {
  perror(msg);
  exit(1);
}

#define SMI_CLK 125e6
#define DMA_MAX 2048 //Max physically contig from memalign()
#define WRITE_STROBE_MASK 0x7F
#define WRITE_STROBE_CYCLES 12
//static_assert(WRITE_STROBE_CYCLE <= WRITE_STROBE_MASK);
#define DSHOT_BITS 16
#define DSHOT600_DMA_PER_BIT 16 //must be < MASK, and DMA_PER_BIT * BITS must be < DMA_MAX
#define DSHOT600_DMA_PER_ZRO 6
#define DSHOT600_DMA_PER_ONE 12
#define DMA_COUNT (int)(DSHOT600_DMA_PER_BIT * DSHOT_BITS)
//static_assert(DMA_COUNT < DMA_MAX);
#define DMA_BYTES_PER_CLOCK 2
#define DMA_TX_TIME ((DMA_COUNT) * (WRITE_STROBE_CYCLES) / SMI_CLK)
#define DMA_TX_TIME_US (DMA_TX_TIME * 1e6)

static void print_smi_settings(struct smi_settings *settings) {
  printf("dma_count: %u\n", DMA_COUNT);
  printf("dma_tx_time: %fus\n", DMA_TX_TIME_US);
  printf("width: %d\n", settings->data_width);
  printf("pack: %c\n", settings->pack_data ? 'Y' : 'N');
  printf("read setup: %d, strobe: %d, hold: %d, pace: %d\n", settings->read_setup_time, settings->read_strobe_time, settings->read_hold_time, settings->read_pace_time);
  printf("write setup: %d, strobe: %d, hold: %d, pace: %d\n", settings->write_setup_time, settings->write_strobe_time, settings->write_hold_time, settings->write_pace_time);
  printf("dma enable: %c, passthru enable: %c\n", settings->dma_enable ? 'Y':'N', settings->dma_passthrough_enable ? 'Y':'N');
  printf("dma threshold read: %d, write: %d\n", settings->dma_read_thresh, settings->dma_write_thresh);
  printf("dma panic threshold read: %d, write: %d\n", settings->dma_panic_read_thresh, settings->dma_panic_write_thresh);
}

int main(int argc, char **argv) {
  int fd = open("/dev/smi", O_RDWR);
  if (fd < 0) fail("cant open");
  struct smi_settings settings;
  int ret = ioctl(fd, BCM2835_SMI_IOC_GET_SETTINGS, &settings);
  if (ret != 0) fail("ioctl 1");
  settings.read_setup_time = 10;
  settings.read_strobe_time = 20;
  settings.read_hold_time = 40;
  settings.read_pace_time = 80;
  settings.write_setup_time = 0;
  settings.write_hold_time = 0;
  settings.write_pace_time = 0;
  settings.write_strobe_time = WRITE_STROBE_CYCLES;
  settings.dma_enable = 1;
  settings.data_width = SMI_WIDTH_16BIT;

  ret = ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS, &settings);
  if (ret != 0) fail("ioctl 2");
  print_smi_settings(&settings);

  uint16_t * buffer = memalign(4096, DMA_COUNT * DMA_BYTES_PER_CLOCK);
  while(1){
    for(int pwm = 0; pwm < 100; pwm++){
      unsigned throttle11 = 2000 * pwm / 100;
      unsigned telem1 = 0;
      unsigned upper12 = (throttle11 << 1) | telem1;
      unsigned crc4 = (upper12 ^ (upper12 >> 4) ^ (upper12 >> 8)) & 0x0F;
      uint16_t bits = (upper12 << 4) | crc4;
      unsigned bufIdx = 0;
      for(int bit = DSHOT_BITS - 1; bit >= 0; bit--){
        for(int dma = 0; dma < DSHOT600_DMA_PER_ZRO; ++dma)
          buffer[bufIdx++] = 0xFFFF;
        for(int dma = DSHOT600_DMA_PER_ZRO; dma < DSHOT600_DMA_PER_ONE; ++dma){
          bool bitval = !!(bits & (1<<bit));
          buffer[bufIdx++] = bitval ? 0xFFFF : 0x0;
        }
        for(int dma = DSHOT600_DMA_PER_ONE; dma < DSHOT600_DMA_PER_BIT; ++dma)
          buffer[bufIdx++] = 0x0;
      }
      printf("RC-PWM:%i%%\n", pwm);
      write(fd, buffer, DMA_COUNT * DMA_BYTES_PER_CLOCK);
      //For showing DMA->CPU turnaround time
      //uint16_t minibuffer[2] = {0xFFFF, 0x0};
      //write(fd, minibuffer, 2 * DMA_BYTES_PER_CLOCK);
      //usleep(100);
    }
  }
}
