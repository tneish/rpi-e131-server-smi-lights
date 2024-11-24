//#include <errno.h>
#include <cerrno>

//#include <string.h>
#include <cstring>

//#include <stdio.h>
#include <cstdio>
//#include <stdlib.h>


#include "mem_utils.h"

#define DEBUG 1

int dma_test_mem_transfer(dma_mem_t* m) {
    DMA_CB_t *cbp = (void*)m->virt_addr;
    char *srce = (char *)(cbp+1);
    char *dest = srce + 0x100;

    strcpy(srce, "123");
    dm_safe_memset(cbp, 0, sizeof(DMA_CB_t));
    cbp->ti = DMA_CB_SRC_INC | DMA_CB_DEST_INC;
    cbp->srce_ad = dma_mem_virt_offset_to_bus(m, srce);
    cbp->dest_ad = dma_mem_virt_offset_to_bus(m, dest);
    cbp->tfr_len = 32;
    start_dma(m);
    usleep(10);
#if DEBUG
    disp_dma(m);
#endif
    printf("DMA test: %s\n", dest[0] ? dest : "failed");
    return(dest[0] != 0);
}



/* Note: use dm_safe_memset for writing > 128B to direct memory.
 * strcopy() and friends > 5 chars will use ARM cache which doesn't work either.
 **/
int main() {
	dma_mem_t *m = calloc(1, sizeof(dma_mem_t));

	print_free_dma_channels();
	dma_mem_alloc(4096*16, m);
	enable_dma();
	usleep(100);
	
	dma_test_mem_transfer(m);
	
	
	dma_mem_release(m);
	
}
