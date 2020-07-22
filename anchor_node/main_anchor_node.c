/**
 * This is the source file for the Anchor Node.
 * made by: blinded for review
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "nrf_gpio.h"
#include "nordic_common.h"
#include "nrf_delay.h"
#include "radio_config.h"
#include "boards.h"
#include "nrf.h"
#include "nrf51_bitfields.h"

#include "simple_uart.h"
#include "SEGGER_RTT.h"

/* This ifndef ensures that there is no code compiled used for debugging! */
#ifndef DEBUG
#define printf(str,...)    do{}while(0)
#endif

/* This ifndef defines the beacon number */
#ifndef BEACON
#define BEACON             0
#endif

#define BLE_FREQUENCY       0 /*2400 mHz + this value */

#define RTC_TIMEOUT        (uint32_t)(500 /10) /* ms */
#define TRANSMIT_TIMEOUT   (uint32_t)105       /* ms */

#define GROUP_ID_WPA    95
#define GROUP_ID_ANCHOR 90

#define CMD_SLEEP          20 /* 20 means sleep */


/* Get Anchor ID from Makefile */
static int  nn = BEACON;

/* Global vars */
static bool transmit_back = false;
static bool transmit_sleep_ids;

int cnt = 0;
int clen = 2;
int nlen = 1;
int nsize[2] = {8, 15};
int csize[4] = {4, 7, 8, 16};

/* Orthogonal codes */
#define MAX_NODES 					9
static uint16_t pos_code[MAX_NODES] = {65535, 43690, 52428, 39321, 61680, 42405, 50115, 38550, 65280};
static uint16_t neg_code[MAX_NODES];

/* FEC array */
uint16_t node_nr[9] = {21845, 13107, 26214, 3855, 23130, 15420, 26985, 255, 21930};

/* Payload arrays */
static uint16_t tpacket[30];  /* Location-reply packet to transmit. */
static uint16_t spacket[30];  /* sleep command packet to transmit. */
static uint16_t rpacket[30];  /* Packet to receive. */


/* ---------------------------------------------------------------------------*/
/* Functions used for initilization */
/* ---------------------------------------------------------------------------*/

/**
 * @brief Function to set shorcuts in the RADIO module to send without CPU 
 * intervention. This does a little speedup...
 */
static void set_radio_shortcuts(void)
{
	/* Enable short between READY and START */
	NRF_RADIO->SHORTS = (1UL << RADIO_SHORTS_READY_START_Pos);

	/* Enable short between END and DISABLE */
	NRF_RADIO->SHORTS |= (1UL << RADIO_SHORTS_END_DISABLE_Pos);

	/* Enable the shortcut between getting an address match in the radio and
	 * starting the RSSI. This will enable the RSSI once on every received
	 * packet. */
	NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

	/* Set the frequency */
	NRF_RADIO->FREQUENCY = BLE_FREQUENCY;
}

/**
 * This function initializes the timer 0 and set its freqency and enables the 
 * interrupt. This timer is used to stop transmitting sleep commands after some time.
 */
static void init_rtc0(void)
{
	/* Init the RTC timer! */
	/* Set the frequency to 1000 Hz. */
	NRF_RTC0->PRESCALER = 32;
	/* Enable interrupt for Compare event on CC[0] */
	NRF_RTC0->INTENSET = RTC_INTENSET_COMPARE0_Msk;
	NRF_RTC0->CC[0] = TRANSMIT_TIMEOUT;

	NRF_RTC0->TASKS_CLEAR = 1;
	NVIC_EnableIRQ(RTC0_IRQn);
}

/**
 * This function initializes the timer 1 and set it freqency and enables the 
 * interrupt.
 */
static void init_rtc1(void)
{
	/* Init the RTC timer! */
	/* Set the frequency to 100 Hz. */
	NRF_RTC1->PRESCALER = 327;
	/* Enable interrupt for Compare event on CC[0] */
	NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;
	NRF_RTC1->CC[0] = RTC_TIMEOUT;

	NRF_RTC1->TASKS_CLEAR = 1;
	NVIC_EnableIRQ(RTC1_IRQn);
}

/**
 * This function generates the "location-reply" packet.
 */
static void generate_packet(void)
{
	uint16_t node = node_nr[nn];
	uint16_t bit = 1 << (nsize[nlen] - 1); /* 15 - 1 = 14, 1 << 14 opschuiven */

	transmit_back = true;

	for(int i = 0; i<MAX_NODES; i++) {
		neg_code[i] = ~pos_code[i];
		printf("neg: %d, pos: %d\n", neg_code[i], pos_code[i]);
	}

	/* voor z van 0 tot 14 doe dit */
	for (int z = 0; z < nsize[nlen]; z++) {
		if (node & bit) {
			 tpacket[z] = pos_code[nn];
		} else {
			 tpacket[z] = neg_code[nn];
		}
		bit >>= 1; /* shift one bit right */
	}
}

/**
 * Ensure the GPIO p0.01 is set high to enable the powercaster.
 */
static void init_gpio(void)
{
	/* set AIN2 as output pin and set it HIGH */
	nrf_gpio_cfg_output(1);
	nrf_gpio_pin_set(1);
}

/**
 * This function is mainly used for debuggin purposes, it enables the RGB led on
 * on the smart BEACON kit to blink.
 */
static void blink_led(int anchor_id)
{
	int i;

	nrf_gpio_cfg_output(LED_RGB_RED);
	nrf_gpio_cfg_output(LED_RGB_GREEN);
	nrf_gpio_cfg_output(LED_RGB_BLUE);
	nrf_gpio_pin_set(LED_RGB_RED);
	nrf_gpio_pin_set(LED_RGB_GREEN);
	nrf_gpio_pin_set(LED_RGB_BLUE);

	/* Blink which anchor ID i am. */
	for(i=0; i<anchor_id; i++) {
		nrf_gpio_pin_clear(LED_RGB_RED);
		nrf_delay_ms(200);
		nrf_gpio_pin_set(LED_RGB_RED);
		nrf_delay_ms(200);
	}
	nrf_gpio_pin_clear(LED_RGB_GREEN);
}

/* This function does all the init functions for the transmit_back node!
 * It starts up the external clocks and ensures that debugging is working if enabeld.
 * An initializes timers and or the RTC.
 */
static void system_init(void)
{
	/* Start 16 MHz external crystal oscillator.  This takes 1.1mA current
	 * Takes about 800us before the STARTED event is generated.
	 * */
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART    = 1;
	/* Wait for the external oscillator to start up. */
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
	}

	/* Also startup the LFCLK for use for the RTC, This clock is switched off
	 * when going to OFF mode.
	 * The xrystal takes 1.3uA current when starting and takes about 0.3 s to start.
	 */
	NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	/* Wait for the external oscillator to start up. */
	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {
	}


#ifdef UART_DEBUG
	/* Debug info via UART when SWD is not connected. */
	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, \
		RX_PIN_NUMBER, UART_BAUDRATE_BAUDRATE_Baud115200, 12, HWFC);
#endif

	printf("\n\n-----------------------\n");
	printf("Starting Anchor Node! Node ID: %d\n", nn);
	printf("-----------------------\n");


#ifdef BOARD_PCA20006
	blink_led(nn);
#endif

	/* Init all peripherals */
	init_rtc0();
	init_rtc1(); /* Used to power cycle the powercaster */
	init_gpio();

	/* Set radio configuration parameters. */
	radio_configure();
	set_radio_shortcuts();

	/* The packet is only generated once, so if the node number changes the 
	beacon needs to be reset. */
	generate_packet();
}

/* ---------------------------------------------------------------------------*/
/* Functions used in the main program */
/* ---------------------------------------------------------------------------*/

/**
 * this function is used to turn of the powercaster and start a timer that
 * enables the powercaster again after a predefined time.
 */
static void power_cycle()
{
	nrf_gpio_pin_clear(1);
	NRF_RTC1->TASKS_START = 1;
}

/**
 * This function uses the Radio peripheral to send a packet. It can send a sleep
 * command or a location-reply packet.
 * @param
 */
static void transmit(bool send_sleep)
{
	if(send_sleep) {
		/* Set the transmit packet pointer  to the sleep command packet*/
		NRF_RADIO->PACKETPTR = (uint32_t)spacket;
	} else {
		/* Set the transmit packet pointer to the location-reply packet */
		NRF_RADIO->PACKETPTR = (uint32_t)tpacket;
	}

	/* Enable the radio and start transmission and wait for a disable event */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	NRF_RADIO->TASKS_TXEN   = 1;
	while(NRF_RADIO->EVENTS_DISABLED == 0U) {
	}
}


/**
 * This function ensures the RADIO peripheral goes into receiving mode busy-waiting.
 */
static void receive()
{
	/* Set payload pointer. */
	NRF_RADIO->PACKETPTR    = (uint32_t)rpacket;

	/* wait for a packet! */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	NRF_RADIO->TASKS_RXEN   = 1U;
	while(NRF_RADIO->EVENTS_DISABLED == 0U) {
	}

	/* If there is a CRC match the data is valid*/
	if (NRF_RADIO->CRCSTATUS == 1U)
	{
		/* Default packet! */
		if (rpacket[0] == GROUP_ID_ANCHOR) {

			/* Set the off time for the powercaster */
			NRF_RTC1->CC[0] = (uint32_t)rpacket[1];

			/* check if there is a passive wake up command */
			if ( rpacket[11] == CMD_SLEEP ) {

				spacket[0] = GROUP_ID_WPA;
				spacket[1] = CMD_SLEEP;
				spacket[2] = rpacket[12];
				//printf("Beacons to turn off: %d%d%d%d\n", (int)(rpacket[12]>>3)&0x1, (int)(rpacket[12]>>2)&0x1, (int)(rpacket[12]>>1)&0x1, (int)rpacket[12]&0x1 );

				if (rpacket[12] > 0 && rpacket[12] < 16) {
					/* only transmit sleep ID's if that is needed */
					transmit_sleep_ids = true;
				}
			}

			transmit_back = true;
		} else {
			transmit_back = false;
		}
	}
}

/**
 * This function is mainly used for debuging purposes blinks with a busy 100ms
 * delay. 
 * @param
 */
static void blink_blue(int times)
{
	int i;

	nrf_gpio_pin_set(LED_RGB_GREEN);

	for (i=0; i<times; i++) {
		nrf_gpio_pin_clear(LED_RGB_BLUE);
		nrf_delay_ms(100);
		nrf_gpio_pin_set(LED_RGB_BLUE);
	}
	nrf_gpio_pin_clear(LED_RGB_GREEN);
}

/**
 * This is the main program loop
 * @return
 */
int main(void)
{
	system_init();

	while(true) {
		printf("Receiving...\n");
		receive();
		if (transmit_back) {

			/* added a small delay otherwise the charbee is to fast for the 
			mobile node to receive. */
			nrf_delay_us(10);

			transmit(false);
			power_cycle();

			printf("\n\nStart ID based passive wakeup transmitting...\n\n");
			
			/* Start timer to disable transmitting after a pre-defined period. */
			NRF_RTC0->TASKS_START = 1;
			while(transmit_sleep_ids) {
				transmit(true);
			}
			
		} else {
			printf("Unknown packet recieved. Do nothing...!\n");
		}
	}
}

/* ---------------------------------------------------------------------------*/
/* IRQ Functions handlers*/
/* ---------------------------------------------------------------------------*/
void RTC0_IRQHandler(void)
{
	NRF_RTC0->EVENTS_COMPARE[0] = 0;
	NRF_RTC0->TASKS_STOP = 1;
	NRF_RTC0->TASKS_CLEAR = 1;

	transmit_sleep_ids = false;
}

void RTC1_IRQHandler(void)
{
	NRF_RTC1->EVENTS_COMPARE[0] = 0;
	NRF_RTC1->TASKS_STOP = 1;
	NRF_RTC1->TASKS_CLEAR = 1;
	nrf_gpio_pin_set(1);
}