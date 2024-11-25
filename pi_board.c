/* Derived from:
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
 

#include "pi_board.h"

static const unsigned MEM_FLAG_DIRECT = 1 << 2;
static const unsigned MEM_FLAG_COHERENT = 2 << 2;
static const unsigned MEM_FLAG_ZERO = 1 << 4;


#define DEBUG 1

void get_model_and_revision(struct board_cfg *board)
{
	//board->mem_flag  = 0x04;  
	board->mem_flag = (MEM_FLAG_COHERENT | MEM_FLAG_DIRECT | MEM_FLAG_ZERO);
	board->periph_virt_base = bcm_host_get_peripheral_address();
	board->dram_phys_base = bcm_host_get_sdram_address();
	board->periph_virt_size = bcm_host_get_peripheral_size();
	board->periph_phys_base = 0x7e000000; 
	board->smi_timing[0] = 10;  // RPi4
	board->smi_timing[1] = 15;
	board->smi_timing[2] = 30;
	board->smi_timing[3] = 15;
	
//TODO: Update SMI timings if not RPI4
/*	#if PHYS_REG_BASE==PI_4_REG_BASE        // Timings for RPi v4 (1.5 GHz)
#define SMI_TIMING       10, 15, 30, 15    // 400 ns cycle time
#else                                   // Timings for RPi v0-3 (1 GHz)
#define SMI_TIMING       10, 10, 20, 10   // 400 ns cycle time
#endif*/
	
	if (DEBUG) {
		printf("board {mem_flag: 0x%08X, periph_virt_base: 0x%08X, dram_phys_base: 0x%08X, periph_virt_size: 0x%08X, periph_phys_base: 0x%08X }\n",
			board->mem_flag,
			board->periph_virt_base,
			board->dram_phys_base,
			board->periph_virt_size,
			board->periph_phys_base);
	}

}
