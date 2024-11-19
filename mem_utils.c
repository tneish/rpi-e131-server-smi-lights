#include "mem_utils.h"

int __mem_utils_initialized = 0;
int __mem_utils_mbox_fd;

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

void mem_utils_init() {
	if (__mem_utils_initialized == 1) return;
	__mem_utils_mbox_fd = mbox_open();	
}

void * dm_safe_memset(void *s, int c, size_t n) {
	if (n != 0) {
		unsigned char *d = s;

		do
			*d++ = (unsigned char)c;
		while (--n != 0);
	}
	return (s);
}


int dma_mem_alloc (size_t size, dma_mem_t *d) {
	mem_utils_init();
	
	unsigned handle, mem_flg = 0x4;
	
	handle = mem_alloc(__mem_utils_mbox_fd, size, 4096, mem_flg);

	d->size = size;
	d->bus_addr = mem_lock(__mem_utils_mbox_fd, handle);
	d->virt_addr = mapmem(BUS_TO_PHYS(d->bus_addr), size);
	
}

void dma_mem_release(dma_mem_t* d) {
	mem_utils_init();

	unsigned handle = d->handle;
	size_t size = d->size;
	
    unmapmem((void*)d->virt_addr, size);
    mem_unlock(__mem_utils_mbox_fd, handle);
    mem_free(__mem_utils_mbox_fd, handle);
	
}
