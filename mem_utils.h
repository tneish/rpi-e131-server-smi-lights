#ifndef MEM_UTILS_H
#define MEM_UTILS_H

#include <stdio.h>
#include <stdint.h>
#include "mailbox.h"
#include "pi_board.h"

typedef struct {
	unsigned handle;
	size_t size;
	uint32_t bus_addr;
	void *virt_addr;
	
} dma_mem_t;

// DMA control block (must be 32-byte aligned)
typedef struct {
    uint32_t ti,    // Transfer info
        srce_ad,    // Source address
        dest_ad,    // Destination address
        tfr_len,    // Transfer length
        stride,     // Transfer stride
        next_cb,    // Next control block
        debug,      // Debug register, zero in control block
        unused;
} DMA_CB_t __attribute__ ((aligned(32)));
// DMA register values
#define DMA_WAIT_RESP   (1 << 3)
#define DMA_CB_DEST_INC (1 << 4)
#define DMA_DEST_DREQ   (1 << 6)
#define DMA_CB_SRC_INC  (1 << 8)
#define DMA_CB_SRCE_INC (1 << 8)
#define DMA_SRCE_DREQ   (1 << 10)
#define DMA_PRIORITY(n) ((n) << 16)

// Length of data for 1 row (1 LED on each channel)
#define LED_DLEN        (LED_NBITS * BIT_NPULSES)

// Transmit data type, 8 or 16 bits
#if LED_NCHANS > 8
#define TXDATA_T        uint16_t
#else
#define TXDATA_T        uint8_t
#endif



static const unsigned DMA_CHAN 	= 5;

void mem_utils_init();
void mem_utils_deinit();

/** DMA MEMORY **/
// TODO: error checking
void dma_mem_alloc (size_t size, dma_mem_t *d);
void dma_mem_release(dma_mem_t *d);
volatile void *dm_safe_memset(volatile void *s, int c, size_t n);
volatile void *dm_safe_memcpy(volatile void *s1, volatile const void *s2, size_t n);

uint32_t dma_mem_virt_offset_to_bus(dma_mem_t *m, void *virt);
uint32_t dma_mem_virt_to_bus(dma_mem_t *m);

// Enable/stop DMA channel
void enable_dma();
void stop_dma();

// Start DMA, given dma_mem_t pointer for first control block (see DMA_CB_t)
void start_dma(dma_mem_t *m);
void start_dma_from_offset(dma_mem_t *m, DMA_CB_t *cbp, uint32_t csval);
uint32_t dma_active();
void disp_dma();
void print_free_dma_channels();

/*** GPIO **/
void set_gpio_mode_in(int pin);
void set_gpio_mode_out(int pin);
void set_gpio_out(int pin, int val);

// 32-bit bus address for gpio bank 0 set register
uint32_t gpio_bus_set0();
// 32-bit bus address for gpio bank 0 clear register
uint32_t gpio_bus_clear0();

/*** SMI **/
void init_smi(int dma_req_thresh, int led_nchans, int led_d0_pin);
void setup_smi_dma(dma_mem_t *m, int nsamp);
void swap_bytes(void *data, int len);
void start_smi(dma_mem_t *mp);
void unselect_smi();


#endif
