//RSAXVC
//Program to transmit RC PWM using RPI SMI and CPU delays
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
#define WRITE_STROBE_MASK 0x7F
#define WRITE_STROBE_CYCLES WRITE_STROBE_MASK
#define RC_MIN_US 1000
#define RC_MAX_US 2000
#define RC_SPAN_US (RC_MAX_US - RC_MIN_US)
#define DMA_COUNT (int)((RC_MAX_US * 1e6) / (SMI_CLK / WRITE_STROBE_CYCLES) + 2)
//static_assert(DMA_COUNT < 2048);//Max physically contig from memalign()
#define DMA_BYTES_PER_CLOCK 2
#define DMA_TX_TIME ((DMA_COUNT) * (WRITE_STROBE_CYCLES) / SMI_CLK)
#define DMA_TX_TIME_US (DMA_TX_TIME * 1e6)
#define PWM_HZ 50.0
#define PWM_TX_TIME_US (1e6/PWM_HZ)

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
  int opt;
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

  while ((opt = getopt(argc, argv, "b:e:s:h:p:wE:S:H:P:")) != -1) {
    switch (opt) {
    case 'E':
      settings.write_setup_time = atoi(optarg);
      break;
    case 'S':
      settings.write_strobe_time = atoi(optarg);
      break;
    case 'H':
      settings.write_hold_time = atoi(optarg);
      break;
    case 'P':
      settings.write_pace_time = atoi(optarg);
      break;
    }
  }

  ret = ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS, &settings);
  if (ret != 0) fail("ioctl 2");
  print_smi_settings(&settings);

  uint16_t * buffer = memalign(4096, DMA_COUNT * DMA_BYTES_PER_CLOCK);
  while(1){
    for(int pwm = 0; pwm < 100; pwm++){
      int rc_us = RC_MIN_US + (RC_SPAN_US * pwm / 100);
      int rc_on = rint(rc_us * 1e6 / (SMI_CLK / WRITE_STROBE_CYCLES));
      for (int i=0; i<DMA_COUNT; i++) buffer[i] = i < rc_on ? 0xFFFF : 0;
      //for (int i=0; i<DMA_COUNT; i++) buffer[i] = i < DMA_COUNT - 1 ? 0xFFFF : 0;//Full frame for scoping
      buffer[1] = 0;//Glitch for scoping
      buffer[DMA_COUNT-1] = 0;//TX zero in the gap
      printf("RC-PWM:%i%%\n", pwm);
      write(fd, buffer, DMA_COUNT * DMA_BYTES_PER_CLOCK);
      usleep(PWM_TX_TIME_US - DMA_TX_TIME_US);
    }
  }
}
