/*- memset:
 * Copyright (c) 1990 The Regents of the University of California.
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Chris Torek.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


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
