#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>

#include "mem_utils.h"

#define DEBUG 1

const int led_d0_pin 	= 8; 	// GPIO pin for D0 output
const int led_nchans 	= 8; 	// Number of LED channels (8 or 16)
const int led_nbits	= 24; 	// Number of data bits per LED
const int led_prebits	= 4;	// Number of zero bits before LED data
const int led_postbits	= 4;	// Number of zero bits after LED data
const int bit_npulses	= 3;	// Number of O/P pulses per LED bit
const int chan_maxleds	= 50;	// Maximum number of LEDs per channel
const int req_thresh	= 2;	// DMA request threshold
// TODO: selectable DMA chan

const int chase_msec	= 100;	// Delay time for chaser light test

#define TX_TEST         1   // If non-zero, use dummy Tx data
//#define LED_D0_PIN      8   // GPIO pin for D0 output
//#define LED_NCHANS      8   // Number of LED channels (8 or 16)
#define LED_NBITS       24  // Number of data bits per LED
#define LED_PREBITS     4   // Number of zero bits before LED data
#define LED_POSTBITS    4   // Number of zero bits after LED data
#define BIT_NPULSES     3   // Number of O/P pulses per LED bit
#define CHAN_MAXLEDS    50  // Maximum number of LEDs per channel
#define CHASE_MSEC      100 // Delay time for chaser light test
#define REQUEST_THRESH  2   // DMA request threshold
#define DMA_CHAN        10  // DMA channel to use

// Length of data for 1 row (1 LED on each channel)
#define LED_DLEN        (LED_NBITS * BIT_NPULSES)

// Ofset into Tx data buffer, given LED number in chan
#define LED_TX_OSET(n)      (LED_PREBITS + (LED_DLEN * (n)))

// Size of data buffers & NV memory, given number of LEDs per chan
#define TX_BUFF_LEN(n)      (LED_TX_OSET(n) + LED_POSTBITS)
#define TX_BUFF_SIZE(n)     (TX_BUFF_LEN(n) * sizeof(TXDATA_T))
#define VC_MEM_SIZE         (PAGE_SIZE + TX_BUFF_SIZE(CHAN_MAXLEDS))


#if TX_TEST
// Data for simple transmission test
TXDATA_T tx_test_data[] = {1, 2, 3, 4, 5, 6, 7, 0};
#endif

TXDATA_T *txd;                       // Pointer to uncached Tx data buffer

//TXDATA_T tx_buffer[TX_BUFF_LEN(CHAN_MAXLEDS)];  // Tx buffer for assembling data

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



// Free memory segments and exit
void terminate(int sig) {
    int i;

    printf("Closing\n");
    for (i=0; i < led_nchans; i++) {
            set_gpio_mode_in(led_d0_pin + i);
    }
    mem_utils_deinit();
    exit(0);
}

int main() {
    const int ledpin = 21;
    
    dma_mem_t *m = calloc(1, sizeof(dma_mem_t));

    print_free_dma_channels();

    signal(SIGINT, terminate);
        
    mem_utils_init();

    /* Note: use dm_safe_memset for writing > 128B to direct memory.
    * strcopy() and friends > 5 chars will use ARM cache which doesn't work, either.
    **/
    dma_mem_alloc(VC_MEM_SIZE, m);

    enable_dma();
    usleep(100);
    
    // DMA memory copy 
    dma_test_mem_transfer(m);
    
    // Flash LED via DMA mem copy to GPIO registers
    set_gpio_mode_out(ledpin);
    set_gpio_out(ledpin, 0);
    dma_test_led_flash(m, ledpin);

    // SMI test
    init_smi(req_thresh, led_nchans, led_d0_pin);
    setup_smi_dma(m, sizeof(tx_test_data)/sizeof(TXDATA_T));
#if LED_NCHANS <= 8
    swap_bytes(tx_test_data, sizeof(tx_test_data));
#endif
    DMA_CB_t *cb=m->virt_addr;
    txd = (TXDATA_T *)(cb+1);
    dm_safe_memcpy(txd, tx_test_data, sizeof(tx_test_data));
    start_smi(m);
    usleep(10);
    while (dma_active())
        usleep(10);

    dma_mem_release(m);
    terminate(0);

}
