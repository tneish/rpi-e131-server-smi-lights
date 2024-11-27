#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <endian.h>
#include <byteswap.h>
#include <time.h>
#include <sys/epoll.h> 
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>





#include "mem_utils.h"
#include "ringbuf.h"

#define DEBUG 1

const int led_d0_pin 	= 8; 	// GPIO pin for D0 output
const int led_nchans 	= 8; 	// Number of LED channels (8 or 16)
const int led_nbits	= 24; 	// Number of data bits per LED
const int led_prebits	= 4;	// Number of zero bits before LED data
const int led_postbits	= 4;	// Number of zero bits after LED data
const int bit_npulses	= 3;	// Number of O/P pulses per LED bit
const int chan_maxleds	= 50;	// Maximum number of LEDs per channel
const int chans_sent_from_source = 4;
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

dma_mem_t *m_desc;

// Free memory segments and exit
void terminate(int sig) {
    int i;

    printf("Closing\n");
    for (i=0; i < led_nchans; i++) {
            set_gpio_mode_in(led_d0_pin + i);
    }
    if (m_desc) dma_mem_release(m_desc);
    mem_utils_deinit();
    exit(EXIT_SUCCESS);
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

uint64_t timespec_to_ms(struct timespec *ts) {
    return (ts->tv_sec * 1000ULL) + (ts->tv_nsec / 1000000ULL);
}

int max(int a, int b) { 
    return (a > b) ? a : b; 
}

int main() {
    const int chan_ledcount = 50;
    const int testmode = 1;
    const int udp_port = 5705;
    const int max_epoll_events = 30;
    const int treeframe_buffer_depth = 60 * 10; //60fps * 10sec
    int delta_timeout = 100;
    int oset = 0;
    int sockfd, epollfd, nfds;
    int num_rx = 0;
    uint64_t ts_source, ts_source_net, ts_sink, ts_sink_net;
    TreeFrame_t f = {0};
    struct timespec ts;
    
    struct sockaddr_in addr; 
    struct epoll_event ev, events[max_epoll_events];
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0); 

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(udp_port);

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        exit(EXIT_FAILURE);
    }
    
    epollfd = epoll_create1(0);

    ev.events = EPOLLIN; 
    ev.data.fd = sockfd;
    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, sockfd, &ev) < 0) {
        perror("epoll_ctl");
        exit(EXIT_FAILURE);
    }
    
    
    m_desc = calloc(1, sizeof(dma_mem_t));

    mem_utils_init();
    signal(SIGINT, terminate);
    print_free_dma_channels();
    dma_mem_alloc(VC_MEM_SIZE, m_desc);
    enable_dma();
    usleep(100);
    init_smi(req_thresh, led_nchans, led_d0_pin);
    setup_smi_dma(m_desc, TX_BUFF_LEN(chan_ledcount));
    
    printf("%u LEDs per channel, %u channels\n", chan_ledcount, 
	LED_NCHANS);

    RingBuffer_t *rb = ring_buffer_init(treeframe_buffer_depth);

    while (1) {
	do {
	    nfds = epoll_wait(epollfd, events, max_epoll_events, 
		delta_timeout);
	} while (nfds < 0 && errno == EINTR);

	if (nfds < 0) { perror("epoll_wait"); exit(EXIT_FAILURE); }
	printf("nfds = %d\n", nfds);
	for (int i = 0; i < nfds; i++) {
	    
	    // "Wall clock" so relatively close to sender
	    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
		perror("clock_gettime");
		break;
	    }
	    ts_sink = timespec_to_ms(&ts);
	    if (events[i].events & EPOLLIN) {
		char buf[10000];
		struct sockaddr_in client_addr;
		socklen_t client_len = sizeof(client_addr);
		ssize_t len = recvfrom(events[i].data.fd, buf, 
		    sizeof(buf), 0, (struct sockaddr *)&client_addr, 
		    &client_len);
		if (len < 0) {
		    perror("recvfrom");
		    continue;
		}
		if (len < 2* sizeof(uint64_t) + chan_maxleds * 
		    chans_sent_from_source * sizeof(uint32_t)) {
		    fprintf(stderr, "Error: Packet too small (%u)\n", 
			len);
			
		}
		
		// PACKET FORMAT
		// int64 source timestamp (for delay compensation)
		// int64 timestamp to display on tree
		// int32[] pixel values in frame		
		memset(&f, 0, sizeof(f));  // TreeFrame
		memcpy(&ts_source_net, buf, sizeof(ts_source_net));
		memcpy(&f.ts, buf + sizeof(ts_source_net), sizeof(f.ts));
		#if __BYTE_ORDER == __LITTLE_ENDIAN
		f.ts = __builtin_bswap64(f.ts);
		#endif
		memcpy(&f.rgbs, buf + 2 * sizeof(ts_source_net), 
		    chan_maxleds * chans_sent_from_source 
		    * sizeof(uint32_t));

		ring_buffer_put(rb, &f);

		// "often enough" (1-2 times per second)
		if (num_rx % 30 == 0) { 
		    // Return packet format: 
		    // int64 (source timestamp)
		    // int64 (sink timestamp)
		    char tbuff[sizeof(uint64_t)*2];
		    memcpy(tbuff, &ts_source_net, sizeof(ts_source_net));
		    #if __BYTE_ORDER == __LITTLE_ENDIAN
		    ts_sink_net = __builtin_bswap64(ts_sink);
		    #endif		    
		    memcpy(tbuff + sizeof(ts_source_net), &ts_sink_net, 
			sizeof(ts_sink_net));
		    ssize_t sent_bytes = sendto(sockfd, tbuff, 
			sizeof(tbuff), 0, 
			(struct sockaddr *)&client_addr, 
			sizeof(client_addr));
		    if (sent_bytes < 0) { perror("sendto"); }
		    printf("%d frames in ringbuf\n", 
			ring_buffer_numframes(rb));
		}
		num_rx++;
	    } // EPOLLIN

	} // for-each epoll event (up to 30 buffered packets)
	// All packets waiting have been received, and any return packets sent
	
	if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
	    perror("clock_gettime");
	    break;
	}

	// DEBUG
	/*if (num_rx % 500 == 0) {
	    printf("receiver timestamp now: %ull\n", timespec_to_ms(&ts));
	    while (!ring_buffer_is_empty(rb)) {
		TreeFrame_t t = ring_buffer_get(rb);
		printf("TreeFrame {.ts = %ull}\n", t.ts);
		
	    }
	    assert(0);
	}*/

	
	if (ring_buffer_is_empty(rb))
	    goto idle_exit;
	    
	
	TreeFrame_t *fptr = ring_buffer_peek(rb);

	// Pop buffered frames until current time
	int iters = 0;
	while (fptr->ts < timespec_to_ms(&ts)) {
	    f = ring_buffer_get(rb); // Pop
	    
	    // TODO: convert RGB data and send to smi
	    
	    if (ring_buffer_is_empty(rb)) {
		goto idle_exit;
	    }
	    
	    fptr = ring_buffer_peek(rb);
	    iters++;
	    printf("pop iters = %d\n", iters);
	    //if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
		//perror("clock_gettime");
		//break;
	    //}
	}
	
	// Still frames in buffer but not to be displayed yet..
	delta_timeout = max((fptr->ts - timespec_to_ms(&ts)), 5);
	//fprintf(stderr, "delta_timeout = %d\n", fptr->ts - timespec_to_ms(&ts));
	continue;
	
	idle_exit:
	delta_timeout = 100;
	

    } // for each epoll event or timeout
	
	// Display pixels from buffer, until current time.

	
	
	
	
	//for (int n=0; n<chan_ledcount; n++) {
	    //rgb_txdata(n==oset%chan_ledcount ? on_rgbs : off_rgbs,
			//&tx_buffer[LED_TX_OSET(n)]);
	//}

	//oset++;
//#if LED_NCHANS <= 8
	//swap_bytes(tx_buffer, TX_BUFF_SIZE(chan_ledcount));
//#endif
	//printf("TX_BUFF_SIZE(chan_ledcount) = %d\n", TX_BUFF_SIZE(chan_ledcount));
	//DMA_CB_t *cb=m_desc->virt_addr;
	//txd = (TXDATA_T *)(cb+1);
	//dm_safe_memcpy(txd, tx_buffer, TX_BUFF_SIZE(chan_ledcount));
	//start_smi(m_desc);
	//usleep(CHASE_MSEC * 1000);
    
    
    
    

    
    close(sockfd);
    close(epollfd);
    terminate(0);

}


