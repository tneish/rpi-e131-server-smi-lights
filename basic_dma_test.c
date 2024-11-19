#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
//#include "bcm_host.h"

#include "mem_utils.h"




int main() {
	dma_mem_t *m = calloc(1, sizeof(dma_mem_t));

	dma_mem_alloc(4096, m);
	
	// Works...
	for (int i = 0; i < m->size; i++) {
		printf("%d\n", i);
		memset(m->virt_addr+i, 40, 1);
		
	}
	
	// Works <= 128B
	memset(m->virt_addr, 40, 128);
	
	// Needs 'safe' memset to avoid cache
	dm_safe_memset(m->virt_addr, 40, 129);
	
	// Works..
	strcpy(m->virt_addr, "12345");
	
	// Doesn't work..
	strcpy(m->virt_addr, "123456");
	
	dma_mem_release(m);
	
}
