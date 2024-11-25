#ifndef PI_BOARD_H
#define PI_BOARD_H


#include <stdint.h>
#include <stdio.h>
#include "bcm_host.h"


struct board_cfg {
	uint32_t periph_phys_base;
	uint32_t periph_virt_base;
	uint32_t periph_virt_size;
	uint32_t dram_phys_base;
	unsigned mem_flag;
	int smi_timing[4];

};

void get_model_and_revision(struct board_cfg *board);




#endif
