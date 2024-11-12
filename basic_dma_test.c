#include <errno.h>
#include <string.h>
#include <stdio.h>
#include "bcm_host.h"
#include "mailbox.h"


#define BUS_TO_PHYS(x) ((x)&~0xC0000000)


typedef struct {
	int mbox_fd;
	unsigned handle;
	size_t size;
	uint32_t bus_addr;
	void *virt_addr;
	
	
} dma_mem_t;


int dma_mem_alloc (size_t size, dma_mem_t *d) {
	int mbox_fd = d->mbox_fd;
	unsigned handle, mem_flg = 0x4;
	
	handle = mem_alloc(mbox_fd, size, 4096, mem_flg);

	d->size = size;
	d->bus_addr = mem_lock(mbox_fd, handle);
	d->virt_addr = mapmem(BUS_TO_PHYS(d->bus_addr), size);
	
}

void dma_mem_release(dma_mem_t* d) {
	int mbox_fd = d->mbox_fd;
	unsigned handle = d->handle;
	size_t size = d->size;
	
    unmapmem((void*)d->virt_addr, size);
    mem_unlock(mbox_fd, handle);
    mem_free(mbox_fd, handle);
	
}

int main() {
	dma_mem_t *m = calloc(1, sizeof(dma_mem_t));
	
	m->mbox_fd = mbox_open();
	dma_mem_alloc(4096, m);
	for (int i = 0; i < m->size; i++) {
		printf("%d\n", i);
		memset(m->virt_addr+i, 40, 1);
		
	}
	
	
	// OK!
	memset(m->virt_addr, 40, 128);
	// Too large write burst for the AXI bus.. (BCM2711 ARM Peripherals page 63)
	memset(m->virt_addr, 40, 129);
	
	
	dma_mem_release(m);
	
}
