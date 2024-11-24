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

/* dma_mem_virt_to_bus
 * Copyright (c) 2018 Brian Starkey <stark3y@gmail.com>
 * Portions derived from servod.
 * Copyright (c) 2013 Richard Hirst <richardghirst@gmail.com>
 *
 * This program provides very similar functionality to servoblaster, except
 * that rather than implementing it as a kernel module, servod implements
 * the functionality as a usr space daemon.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include "mem_utils.h"

static const uint32_t dma_base_offset			= 0x7000;
static const uint32_t dma_chan_offset			= 0x0100 * dma_chan;  
static const uint32_t dma_enable_offset			= 0x0ff0;

static const uint32_t dma_reg_offset_cs 		= 0x00;
static const uint32_t dma_reg_offset_conblk_ad 	= 0x04;
static const uint32_t dma_reg_offset_debug		= 0x20;


#define DEBUG 1

static int _mbox_fd = -1;
static struct board_cfg _board = {0};

static volatile uint32_t *virt_periph_regs;
static volatile uint32_t *virt_dma_regs;

static inline uint32_t dma_mem_bus_to_phys(uint32_t b) {
	return (b)&~_board.dram_phys_base;
}

uint32_t dma_mem_virt_offset_to_bus(dma_mem_t *m, void *virt) {
	uint32_t offset = (uint8_t *)virt - (uint8_t*)m->virt_addr;
	return m->bus_addr + offset;
}

volatile uint32_t *reg32_offset(uintptr_t base, unsigned offset) {
	return (volatile uint32_t *)((uintptr_t)base + offset);
	
}

void mem_utils_init() {
	static int initialized = 0;
	if (initialized > 0) return;
	
	_mbox_fd = mbox_open();
	get_model_and_revision(&_board);
	virt_periph_regs = mapmem(_board.periph_virt_base, _board.periph_virt_size);
	virt_dma_regs = (volatile uint32_t *)((uintptr_t)virt_periph_regs + dma_base_offset);
	initialized = 1;
	
	if (DEBUG) {
		printf("virt_periph_regs* = 0x%0LX, virt_dma_regs* = 0x%0LX.\n", virt_periph_regs, virt_dma_regs);
		
	}
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
	
	unsigned handle, mem_flg;

	mem_flg = _board.mem_flag;
	handle = mem_alloc(_mbox_fd, size, 4096, mem_flg);

	d->size = size;
	d->bus_addr = mem_lock(_mbox_fd, handle);
	d->virt_addr = mapmem(dma_mem_bus_to_phys(d->bus_addr), size);
	
}

void dma_mem_release(dma_mem_t *d) {
	mem_utils_init();

	unsigned handle = d->handle;
	size_t size = d->size;
	
	unmapmem((void*)d->virt_addr, size);
	mem_unlock(_mbox_fd, handle);
	mem_free(_mbox_fd, handle);
	
}

void print_free_dma_channels() {
	mem_utils_init();
	unsigned chans = 0;
	
	chans = get_dma_channels(_mbox_fd);
	
	printf("Available DMA channels: ");
	int found = 0;
	for (int i=0; i<16; i++) {
		if ((chans & (1 << i)) > 0) {
			printf("%s%d", found ? ", " : "", i);
			found = 1;
		}
	}
	printf(".\n");
	
}

void enable_dma() {
	
	__sync_synchronize();  
	*(reg32_offset((uintptr_t)virt_periph_regs, dma_enable_offset)) |= (1 << dma_chan);
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) = 1 << 31;
	
}


void start_dma(dma_mem_t *m) {
	mem_utils_init();

	__sync_synchronize();  // mem barrier
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_conblk_ad)) = (uint32_t)(m->bus_addr);
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) 	= 2;   // Clear 'end' flag
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_debug)) 	= 7;   // Clear error bits
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) 	= 1;   // Start DMA
    
}

void disp_dma() {
	char *dma_regstrs[] = {"DMA CS", "CB_AD", "TI", "SRCE_AD", "DEST_AD",
    "TFR_LEN", "STRIDE", "NEXT_CB", "DEBUG", ""};
    
    uint32_t dma_offset[] = {0x0, 0x4, 0x8, 0xc, 0x10, 0x14, 0x18, 0x1c, 0x20}; // in bytes!
    
    volatile uint32_t *p= (volatile uint32_t *)((uintptr_t)virt_dma_regs + dma_chan_offset);
    
    int i = 0;
    while (dma_regstrs[i][0])  
    {
        printf("%-7s %08X ", dma_regstrs[i], *(p+(dma_offset[i]/4)));
        if (i%5==0 || dma_regstrs[i][0]==0)
            printf("\n");
	i++;
    }
    __sync_synchronize();
    printf("\n");
}

