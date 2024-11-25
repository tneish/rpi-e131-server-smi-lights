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
#define LED_NCHANS      8   // Number of LED channels (8 or 16)
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

TXDATA_T tx_buffer[TX_BUFF_LEN(CHAN_MAXLEDS)];  // Tx buffer for assembling data
int rgb_data[CHAN_MAXLEDS][LED_NCHANS]; // RGB data

// RGB values for test mode (1 value for each of 16 channels)
int on_rgbs[16] = {0xff0000, 0x00ff00, 0x0000ff, 0xffffff,
                  0xff4040, 0x40ff40, 0x4040ff, 0x404040,
                  0xff0000, 0x00ff00, 0x0000ff, 0xffffff,
                  0xff4040, 0x40ff40, 0x4040ff, 0x404040};
int off_rgbs[16];


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

// Set Tx data for 8 or 16 chans, 1 LED per chan, given 1 RGB val per chan
// Logic 1 is 0.8us high, 0.4 us low, logic 0 is 0.4us high, 0.8us low
void rgb_txdata(int *rgbs, TXDATA_T *txd) {
    int i, n, msk;

    // For each bit of the 24-bit RGB values..
    for (n=0; n < LED_NBITS; n++)
    {
        // Mask to convert RGB to GRB, M.S bit first
        msk = n==0 ? 0x8000 : n==8 ? 0x800000 : n==16 ? 0x80 : msk>>1;
        // 1st byte or word is a high pulse on all lines
        txd[0] = (TXDATA_T)0xffff;
        // 2nd has high or low bits from data
        // 3rd is a low pulse
        txd[1] = txd[2] = 0;
        for (i=0; i<LED_NCHANS; i++)
        {
            if (rgbs[i] & msk)
                txd[1] |= (1 << i);
        }
        txd += BIT_NPULSES;
    }
}



int main() {
    const int chan_ledcount = 50;
    const int testmode = 1;
    int oset = 0;
    
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

    init_smi(req_thresh, led_nchans, led_d0_pin);
    setup_smi_dma(m, TX_BUFF_LEN(chan_ledcount));
    printf("%s %u LED%s per channel, %u channels\n", testmode ? "Testing" : "Setting",
           chan_ledcount, chan_ledcount==1 ? "" : "s", LED_NCHANS);
    if (testmode) {
	while (1) {
	    for (int n=0; n<chan_ledcount; n++) {
		rgb_txdata(n==oset%chan_ledcount ? on_rgbs : off_rgbs,
                            &tx_buffer[LED_TX_OSET(n)]);
	    }

            oset++;
#if LED_NCHANS <= 8
            swap_bytes(tx_buffer, TX_BUFF_SIZE(chan_ledcount));
#endif
            printf("TX_BUFF_SIZE(chan_ledcount) = %d\n", TX_BUFF_SIZE(chan_ledcount));
	    DMA_CB_t *cb=m->virt_addr;
	    txd = (TXDATA_T *)(cb+1);
            dm_safe_memcpy(txd, tx_buffer, TX_BUFF_SIZE(chan_ledcount));
            start_smi(m);
            usleep(CHASE_MSEC * 1000);
        }
    }

    dma_mem_release(m);
    terminate(0);

}
