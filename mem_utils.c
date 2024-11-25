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

/*- memcpy:
 * 	$OpenBSD: memcpy.c,v 1.5 2021/05/16 04:45:58 jsg Exp $	
 * Copyright (c) 1993
 *	The Regents of the University of California.  All rights reserved.
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
#include "rpi_smi_defs.h"


/* DMA definitions */
// Offsets are in bytes!
static const uint32_t dma_base_offset			= 0x7000;
static const uint32_t dma_chan_offset			= 0x0100 * DMA_CHAN;  
static const uint32_t dma_enable_offset			= 0x0ff0;

static const uint32_t dma_reg_offset_cs 		= 0x00;
static const uint32_t dma_reg_offset_conblk_ad 	= 0x04;
static const uint32_t dma_reg_offset_debug		= 0x20;

// DMA channels and data requests
#define DMA_CHAN_A      10
#define DMA_CHAN_B      11
#define DMA_PWM_DREQ    5
#define DMA_SPI_TX_DREQ 6
#define DMA_SPI_RX_DREQ 7
#define DMA_BASE        (PHYS_REG_BASE + 0x007000)
// DMA register addresses offset by 0x100 * chan_num
#define DMA_CS          0x00
#define DMA_CONBLK_AD   0x04
#define DMA_TI          0x08
#define DMA_SRCE_AD     0x0c
#define DMA_DEST_AD     0x10
#define DMA_TXFR_LEN    0x14
#define DMA_STRIDE      0x18
#define DMA_NEXTCONBK   0x1c
#define DMA_DEBUG       0x20
#define DMA_REG(ch, r)  ((r)==DMA_ENABLE ? DMA_ENABLE : (ch)*0x100+(r))
#define DMA_ENABLE      0xff0



/* GPIO definitions */
static const uint32_t gpio_base_offset			= 0x200000;
static const uint32_t gpio_reg_offset_fsel0		= 0x00;
static const uint32_t gpio_reg_offset_set0		= 0x1c;
static const uint32_t gpio_reg_offset_clear0	= 0x28;
#define GPIO_ALT1       5

/* CLK definitions */
static const uint32_t clk_base_offset			= 0x101000;
// Clock
//void *virt_clk_regs;
#define CLK_PWM_CTL     0xa0
#define CLK_PWM_DIV     0xa4
#define VIRT_CLK_REG(a) ((volatile uint32_t *)((PTR_TYPE)virt_clk_regs + (a)))
#define CLK_PASSWD      0x5a000000
//#define CLOCK_KHZ       250000
#define CLOCK_KHZ       375000
#define PWM_CLOCK_ID    0xa


/* SMI definitions */
static const uint32_t smi_base_offset			= 0x600000;
static const uint32_t smi_reg_offset_cs			= 0x00; // control & status
static const uint32_t smi_dma_dreq				= 4;

// Pointers to SMI registers
volatile SMI_CS_REG  *smi_cs;
volatile SMI_L_REG   *smi_l;
volatile SMI_A_REG   *smi_a;
volatile SMI_D_REG   *smi_d;
volatile SMI_DMC_REG *smi_dmc;
volatile SMI_DSR_REG *smi_dsr;
volatile SMI_DSW_REG *smi_dsw;
volatile SMI_DCS_REG *smi_dcs;
volatile SMI_DCA_REG *smi_dca;
volatile SMI_DCD_REG *smi_dcd;

#define DEBUG 1

static int _mbox_fd = -1;
static int initialized = 0;
static struct board_cfg _board = {0};

static volatile uint32_t *virt_periph_regs;
static volatile uint32_t *virt_dma_regs;
static volatile uint32_t *virt_gpio_regs;
static volatile uint32_t *virt_smi_regs, *smi_regs;
static volatile uint32_t *virt_clk_regs, *clk_regs;

static inline uint32_t dma_mem_bus_to_phys(uint32_t b) {
	return (b)&~_board.dram_phys_base;
}

uint32_t dma_mem_virt_to_bus(dma_mem_t *m) {
	return m->bus_addr;
}

uint32_t dma_mem_virt_offset_to_bus(dma_mem_t *m, void *virt) {
	uint32_t offset = (uint8_t *)virt - (uint8_t*)m->virt_addr;
	return m->bus_addr + offset;
}

volatile uint32_t *reg32_offset(uintptr_t base, unsigned offset) {
	return (volatile uint32_t *)((uintptr_t)base + offset);
	
}

void mem_utils_init() {
	if (initialized > 0) return;
	
	_mbox_fd = mbox_open();
	get_model_and_revision(&_board);
	virt_periph_regs = mapmem(_board.periph_virt_base, _board.periph_virt_size);
	virt_dma_regs = (volatile uint32_t *)((uintptr_t)virt_periph_regs + dma_base_offset);
	virt_gpio_regs = (volatile uint32_t *)((uintptr_t)virt_periph_regs + gpio_base_offset);
	
	//virt_smi_regs = (volatile uint32_t *)((uintptr_t)virt_periph_regs + smi_base_offset);
	smi_regs = mapmem(_board.periph_virt_base + smi_base_offset, 4096);
	//virt_clk_regs = (volatile uint32_t *)((uintptr_t)virt_periph_regs + clk_base_offset);
	clk_regs = mapmem(_board.periph_virt_base + clk_base_offset, 4096);
	initialized = 1;
	
	if (DEBUG) {
		printf("virt_periph_regs* = 0x%0LX, virt_dma_regs* = 0x%0LX, virt_gpio_regs* = 0x%0LX, virt_smi_regs* = 0x%0LX, virt_clk_regs* = 0x%0LX.\n", 
			virt_periph_regs, 
			virt_dma_regs,
			virt_gpio_regs,
			virt_smi_regs,
			virt_clk_regs);
		
	}
}

void mem_utils_deinit() {
	if (initialized) {
		unselect_smi();
		stop_dma();
		if (virt_periph_regs) {
			unmapmem((void*)virt_periph_regs, _board.periph_virt_size);
		}
	}
}


// ----- GPIO -----
uint32_t gpio_bus_clear0() {
	return _board.periph_phys_base + gpio_base_offset + gpio_reg_offset_clear0;
}


uint32_t gpio_bus_set0() {
	return _board.periph_phys_base + gpio_base_offset + gpio_reg_offset_set0;
}

// Set input or output
void set_gpio_mode(int pin, int mode) {
	volatile uint32_t *reg = reg32_offset((uintptr_t)virt_gpio_regs, 
								gpio_reg_offset_fsel0) + pin / 10;
	unsigned shift = (pin % 10) * 3;
	
	*reg = (*reg & ~(7 << shift)) | (mode << shift);
}

void set_gpio_mode_out(int pin) {
	mem_utils_init();
	set_gpio_mode(pin, 1);
}

void set_gpio_mode_in(int pin) {
	mem_utils_init();
	set_gpio_mode(pin, 0);
}

void set_gpio_mode_alt1(int pin) {
	set_gpio_mode(pin, GPIO_ALT1);
}

// Set an O/P pin
void set_gpio_out(int pin, int val) {
	mem_utils_init();
	volatile uint32_t *reg = reg32_offset((uintptr_t)virt_gpio_regs, 
								val ? gpio_reg_offset_set0 : gpio_reg_offset_clear0) 
								+ pin / 32;
	*reg = 1 << (pin % 32);
}

// ----- DMA ------
volatile void * dm_safe_memset(volatile void *s, int c, size_t n) {
	if (n != 0) {
		volatile unsigned char *d = s;

		do
			*d++ = (unsigned char)c;
		while (--n != 0);
	}
	return (s);
}

volatile void * dm_safe_memcpy(volatile void *s1, volatile const void *s2, size_t n) {
	volatile const char *f = s2;
	volatile char *t = s1;

	while (n-- > 0)
		*t++ = *f++;
	return s1;
}

volatile void *dm_safe_reg32_offset(volatile void *target, volatile void *base, uintptr_t offset) {
	__sync_synchronize();
	return dm_safe_memcpy(target, (void *)((uintptr_t)(base) + offset), 4);
	
}

void dma_mem_alloc (size_t size, dma_mem_t *d) {
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
	*(reg32_offset((uintptr_t)virt_periph_regs, dma_enable_offset)) |= (1 << DMA_CHAN);
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) = 1 << 31;
	
}

// Halt current DMA operation by resetting controller
void stop_dma() {
	mem_utils_init();
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) = 1 << 31;
}

// Start DMA, given first control block
void start_dma(dma_mem_t *m) {
	mem_utils_init();

	__sync_synchronize();  // mem barrier
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_conblk_ad)) = (uint32_t)(m->bus_addr);
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) 	= 2;   // Clear 'end' flag
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_debug)) 	= 7;   // Clear error bits
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) 	= 1;   // Start DMA
	
}

void start_dma_from_offset(dma_mem_t *m, DMA_CB_t *cbp, uint32_t csval) {
	__sync_synchronize();  // mem barrier
    *(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_conblk_ad)) = dma_mem_virt_offset_to_bus(m, cbp);
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) 	= 2;   // Clear 'end' flag
	*(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_debug)) 	= 7;   // Clear error bits    
    *(reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) 	= 1|csval;   // Start DMA
}

// Check if DMA is active
uint32_t dma_active() {
    return((*reg32_offset((uintptr_t)virt_dma_regs, dma_chan_offset + dma_reg_offset_cs)) & 1);
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

// ------ SMI -------
uint32_t smi_bus_with_offset(unsigned offset) {
	return _board.periph_phys_base + smi_base_offset + offset;
}

// Initialise SMI, given data width, time step, and setup/hold/strobe counts
// Step value is in nanoseconds: even numbers, 2 to 30
void init_smi(int dma_req_thresh,
				int led_nchans,
				int led_d0_pin) {
    
	int width = led_nchans > 8 ? SMI_16_BITS : SMI_8_BITS;
    int ns = _board.smi_timing[0];
    int setup = _board.smi_timing[1];
    int strobe = _board.smi_timing[2];
    int hold = _board.smi_timing[3];
	int divi = ns / 2;

    smi_cs  = (SMI_CS_REG *) REG32(smi_regs, SMI_CS);
    smi_l   = (SMI_L_REG *)  REG32(smi_regs, SMI_L);
    smi_a   = (SMI_A_REG *)  REG32(smi_regs, SMI_A);
    smi_d   = (SMI_D_REG *)  REG32(smi_regs, SMI_D);
    smi_dmc = (SMI_DMC_REG *)REG32(smi_regs, SMI_DMC);
    smi_dsr = (SMI_DSR_REG *)REG32(smi_regs, SMI_DSR0);
    smi_dsw = (SMI_DSW_REG *)REG32(smi_regs, SMI_DSW0);
    smi_dcs = (SMI_DCS_REG *)REG32(smi_regs, SMI_DCS);
    smi_dca = (SMI_DCA_REG *)REG32(smi_regs, SMI_DCA);
    smi_dcd = (SMI_DCD_REG *)REG32(smi_regs, SMI_DCD);

	// Getting sigbus for some reason
	//smi_l->value = 0;
	dm_safe_memset(&(smi_l->value), 0, 4);
    //smi_cs->value = 0;
    dm_safe_memset(&(smi_cs->value), 0, 4);
    //smi_a->value = 0;
    dm_safe_memset(&(smi_a->value), 0, 4);
    //smi_dsr->value = 0;
    dm_safe_memset(&(smi_dsr->value), 0, 4);
    //smi_dsw->value = 0;
    dm_safe_memset(&(smi_dsw->value), 0, 4);
    //smi_dcs->value = 0;
    dm_safe_memset(&(smi_dcs->value), 0, 4);
	//smi_dca->value = 0;
    dm_safe_memset(&(smi_dca->value), 0, 4);
    

    if (*REG32(clk_regs, CLK_SMI_DIV) != divi << 12) {
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | (1 << 5);
        usleep(10);
        while (*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) ;
        usleep(10);
        *REG32(clk_regs, CLK_SMI_DIV) = CLK_PASSWD | (divi << 12);
        usleep(10);
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | 6 | (1 << 4);
        usleep(10);
        while ((*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) == 0) ;
        usleep(100);
    }
    if (smi_cs->seterr) { 
		smi_cs->seterr = 1; 
	}
    smi_dsr->rsetup = smi_dsw->wsetup = setup;
    smi_dsr->rstrobe = smi_dsw->wstrobe = strobe;
    smi_dsr->rhold = smi_dsw->whold = hold;
    smi_dmc->panicr = smi_dmc->panicw = 8;
    smi_dmc->reqr = smi_dmc->reqw = dma_req_thresh;
    smi_dsr->rwidth = smi_dsw->wwidth = width;
    for (int i=0; i<led_nchans; i++) {
        set_gpio_mode_alt1(led_d0_pin + i);
	}
}

void setup_smi_dma(dma_mem_t *m, int nsamp) {
    DMA_CB_t *cbs=m->virt_addr;

    TXDATA_T *txdata = (TXDATA_T *)(cbs+1);
    smi_dmc->dmaen = 1;
    smi_cs->enable = 1;
    smi_cs->clear = 1;
    smi_cs->pxldat = 1;
    smi_l->len = nsamp * sizeof(TXDATA_T);
    smi_cs->write = 1;
    enable_dma();
    cbs[0].ti = DMA_DEST_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_SRCE_INC | DMA_WAIT_RESP;
    cbs[0].tfr_len = nsamp * sizeof(TXDATA_T);
    cbs[0].srce_ad = dma_mem_virt_offset_to_bus(m, txdata);
    //cbs[0].dest_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[0].dest_ad = smi_bus_with_offset(SMI_D);

}

// Swap adjacent bytes in transmit data
void swap_bytes(void *data, int len) {
    uint16_t *wp = (uint16_t *)data;

    len = (len + 1) / 2;
    while (len-- > 0)
    {
        *wp = __builtin_bswap16(*wp);
        wp++;
    }
}

// Start SMI DMA transfers
void start_smi(dma_mem_t *mp) {
    DMA_CB_t *cbs=mp->virt_addr;

    start_dma_from_offset(mp, &cbs[0], 0);
    smi_cs->start = 1;
}

void unselect_smi() {
	
	//*REG32(smi_regs, SMI_CS) = 0;	
	dm_safe_memset(&(smi_cs->value), 0, 4);
	//*(reg32_offset((uintptr_t)virt_smi_regs, smi_reg_offset_cs)) = 0;
	
}

