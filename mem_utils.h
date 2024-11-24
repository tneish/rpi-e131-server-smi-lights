#ifndef MEM_UTILS_H
#define MEM_UTILS_H

//#include <stdio.h>
#include <cstdio>

//#include <stdint.h>
#include <cstdint>

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
#define DMA_CB_DEST_INC (1<<4)
#define DMA_CB_SRC_INC  (1<<8)


static const unsigned dma_chan 	= 5;


int dma_mem_alloc (size_t size, dma_mem_t *d);
void dma_mem_release(dma_mem_t *d);
void * dm_safe_memset(void *s, int c, size_t n);
uint32_t dma_mem_virt_offset_to_bus(dma_mem_t *m, void *virt);
void enable_dma();

// Start DMA, given dma_mem_t pointer for first control block (see DMA_CB_t)
void start_dma(dma_mem_t *m);

void disp_dma();
void print_free_dma_channels();



#endif
