#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#include "mem_utils.h"

#define DEBUG 1

int dma_test_mem_transfer(dma_mem_t *m) {
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

void dma_test_led_flash(dma_mem_t *m, int pin) {
    DMA_CB_t *cbp = (void*)m->virt_addr;
    uint32_t *data = (uint32_t *)(cbp+1), n;

    printf("DMA test: flashing LED on GPIO pin %u\n", pin);
    dm_safe_memset(cbp, 0, sizeof(DMA_CB_t));
    *data = 1 << pin;
    cbp->tfr_len = 4;
    cbp->srce_ad = dma_mem_virt_offset_to_bus(m, data);
    for (n=0; n<16; n++)
    {
        usleep(200000);
	cbp->dest_ad = n&1 ? gpio_bus_clear0() : gpio_bus_set0();
        start_dma(m);
    }
}


int main() {
	const int ledpin = 21;
	
	dma_mem_t *m = calloc(1, sizeof(dma_mem_t));

	print_free_dma_channels();

	/* Note: use dm_safe_memset for writing > 128B to direct memory.
	* strcopy() and friends > 5 chars will use ARM cache which doesn't work, either.
	**/
	dma_mem_alloc(4096, m);

	enable_dma();
	usleep(100);
	
	// DMA memory copy 
	dma_test_mem_transfer(m);
	
	// Flash LED via DMA mem copy to GPIO registers
	set_gpio_mode_out(ledpin);
	set_gpio_out(ledpin, 0);
	dma_test_led_flash(m, ledpin);

	dma_mem_release(m);
}
