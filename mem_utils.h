#ifndef MEM_UTILS_H
#define MEM_UTILS_H

#include <stdio.h>
#include <stdint.h>
#include "mailbox.h"

typedef struct {
	unsigned handle;
	size_t size;
	uint32_t bus_addr;
	void *virt_addr;
	
	
} dma_mem_t;


int dma_mem_alloc (size_t size, dma_mem_t *d);
void dma_mem_release(dma_mem_t* d);
void * dm_safe_memset(void *s, int c, size_t n);


#endif
