/*
 * Mobile node c main file
 * made by: blinded for review
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "radio_config.h"
#include "boards.h"
#include "nrf.h"

#include "simple_uart.h"
#include "SEGGER_RTT.h"

#include <assert.h>


/* Delays */
#define WAITING_TIME            9000 /* ms */

#define WAKEUP_PERIOD          1000 /* ms */ /* t_m */
#define WAIT_FOR_TRANSMIT_TIME   10 /* ms */ /* t_c */

#define CHECK_PERIOD_MN          10 /* ms */ 
#define RX_TIMEOUT                3 /* ms */
#define PWAKE_TIMEOUT_VALUE     109 /* ms */

/* Commands used in tpackets */
#define CMD_SLEEP          20

/* Group IDs */
#define GROUP_ID_WPA       95
#define GROUP_ID_ANCHOR    90

#define BLE_FREQUENCY       0 /*2400 mHz + this value */

/* CANNOT BE LOWER THEN 4, otherwise the powercast stays on and turns RED */
#define POWER_CAST_TURN_OFF_TIME 4 /* x10 ms*/

#define TXPOWER            30 /* */

#define ADC_TRESHOLD       5  /*255*/

#define MAX_STR_LENGTH     255

#define FIRST_ROUND        0
#define SECOND_ROUND       1

/* This ifndef ensures that there is no code compiled used for debugging! */
#ifndef DEBUG
#define printf(str,...)    do{}while(0)
#endif

/* -----------------------------------------------------------------------------
 * All the gobal and local variables
 * -----------------------------------------------------------------------------
 */

/* Orthogonal codes */
#define BIT_SIZE_ENCODED_NODE_ID 	15
#define MAX_NODES 					9
static uint16_t pos_code[MAX_NODES] = {65535, 43690, 52428, 39321, 61680, 42405, 50115, 38550, 65280};
static uint16_t neg_code[MAX_NODES];

/* FEC array */
static uint16_t node_nr[MAX_NODES] = {21845, 13107, 26214, 3855, 23130, 15420, 26985, 255, 21930};


typedef enum {
	STATE_SLEEP,
	STATE_TRANSMIT_FIRST,
	STATE_RECEIVE_FIRST,
	STATE_WAIT_FOR_PWAKE,
	STATE_WAIT_FOR_TRANSMIT,
	STATE_TRANSMIT_SEC,
	STATE_RECEIVE_SEC
} wiploc_state_t;


/* interrupts flags */
static bool wait_for_lfclk;
static bool stop_receiving;
static int timer0 = 0;
static int radio_disabled = 0;
static int pwake = 0;
static int wait_for_transmit = 0;
static int pwake_timeout = 0;

/* Mesurement globals */
static int mesurement;
static int rx_round; /* 0 for roomlevel and 1 for cell level */
static uint16_t rx_something[2];
static uint16_t rx_rssi[2];
static uint16_t rx_crc_ok[2];
static uint16_t rx_result[2];

static uint16_t tpacket[PACKET_PAYLOAD_MAXSIZE/2];  /**< Packet to transmit. */
static uint16_t rpacket[PACKET_PAYLOAD_MAXSIZE/2];  /**< Packet to transmit. */

static uint16_t first_result[15];
static uint16_t second_result[15];

#define DOUT_ARRAY_SIZE 10
static int dout_values[DOUT_ARRAY_SIZE];


/* ---------------------------------------------------------------------------*/
/* Functions used for initilization */
/* ---------------------------------------------------------------------------*/

/**
 * This function generates the negative orthogonal codes from the positiv codes
 * and stores them in the neg_code.
 */
static void init_ortogonal_codes(void) {
	/* Generate negative codes from positive codes */
	for(int i = 0; i<MAX_NODES; i++) {
		neg_code[i] = ~pos_code[i];
		//printf("neg: %d, pos: %d\n", neg_code[i], pos_code[i]);
	}
}

/* this function is used to push a new value on the array */
static void push(int a[], int size, int val) 
{
	static int i = 0;
	if (i==size) i=0;
	a[i] = val;
	i++;
}

/**
 * @brief Function to set shorcuts in the RADIO module to send without CPU intervention.
 */
static void set_radio_shortcuts(void)
{
	/* Enable short between READY and START */
	NRF_RADIO->SHORTS = (1UL << RADIO_SHORTS_READY_START_Pos);

	/* Enable short between END and DISABLE */
	NRF_RADIO->SHORTS |= (1UL << RADIO_SHORTS_END_DISABLE_Pos);

	/* Enable the shortcut between getting an address match in the radio and starting the RSSI
	 * This will enable the RSSI once on every received packet */
	NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

	NRF_RADIO->INTENSET = 1<<4;
	NRF_RADIO->FREQUENCY = BLE_FREQUENCY;
	NVIC_EnableIRQ(RADIO_IRQn);
}

/**
 * This function ennsures all peripheral are turned off.
 */
static void disable_all_peripherals(void)
{
	NRF_RADIO->POWER=0;
	NRF_UART0->POWER=0;
	NRF_SPI0->POWER=0;
	NRF_TWI0->POWER=0;
	NRF_SPI1->POWER=0;
	NRF_TWI1->POWER=0;
	NRF_SPIS1->POWER=0;
	NRF_GPIOTE->POWER=0;
	NRF_ADC->POWER=0;
	NRF_TIMER0->POWER=0;
	NRF_TIMER1->POWER=0;
	NRF_TIMER2->POWER=0;
	NRF_RTC0->POWER=0;
	NRF_TEMP->POWER=0;
	NRF_RNG->POWER=0;
	NRF_ECB->POWER=0;
	NRF_AAR->POWER=0;
	NRF_CCM->POWER=0;
	NRF_WDT->POWER=0;
	NRF_RTC1->POWER=0;
	NRF_QDEC->POWER=0;
	NRF_LPCOMP->POWER=0;
}

/**
 * This function sets the digital output for controlling Dset on the powercaster
 */
static void init_gpio(void)
{
	nrf_gpio_cfg_output(2);
	nrf_gpio_pin_clear(0);
}

/**
 * This function initializes the ADC peripheral of the nRF51822 to enable
 * voltage mesurements on the Dout of the harverster.
 */
static void init_adc(void)
{
	NRF_ADC->POWER=1;

	NRF_ADC->CONFIG = 2; /* RES 10 bit resolution */
	NRF_ADC->CONFIG |= 1<<2;  /* INPSEL AnalogInputNoPrescaling */
	NRF_ADC->CONFIG |= 0<<5;  /* REFSEL VBG*/
	NRF_ADC->CONFIG |= (uint32_t)1<<10; /* PSEL AIN 2*/
	NRF_ADC->INTENSET = 1;
	NVIC_EnableIRQ(ADC_IRQn);

	NRF_ADC->ENABLE = 1;     /* READY event */
}

/**
 * This function enables two timers to do various tasks
 */
/**
 * This timer is needed for the PWAKE timeout
 */
static void init_rtc0(void)
{
	/* Also enable the RTC0 for a timeout on PWAKE */
	NRF_RTC0->POWER=1;
	/* Init the RTC timer! */
	NRF_RTC0->TASKS_STOP = 1;

	/* Set the frequency to 1000 Hz. */
	NRF_RTC0->PRESCALER = 32;

	/* Enable interrupt for Compare event on CC[0] */
	NRF_RTC0->INTENSET = 1<<16;
	NRF_RTC0->CC[0] = CHECK_PERIOD_MN;

	NRF_RTC0->TASKS_CLEAR = 1;
	NRF_RTC0->EVENTS_COMPARE[0] = 0;
	NVIC_EnableIRQ(RTC0_IRQn);
}

/**
 * This timer is needed for everyhting else
 */
static void init_rtc1(void)
{
	NRF_RTC1->POWER=1;
	/* Init the RTC timer! */
	NRF_RTC1->TASKS_STOP = 1;

	/* Set the frequency to 1000 Hz. */
	NRF_RTC1->PRESCALER = 32;

	/* Enable interrupt for Compare event on CC[0] */
	NRF_RTC1->INTENSET = 1<<16;
	NRF_RTC1->CC[0] = WAKEUP_PERIOD;
	NRF_RTC1->CC[1] = RX_TIMEOUT;
	NRF_RTC1->CC[2] = PWAKE_TIMEOUT_VALUE;
	NRF_RTC1->CC[3] = WAIT_FOR_TRANSMIT_TIME;

	/* Clear the RTC counter value for sure */
	NRF_RTC1->TASKS_CLEAR = 1;

	NRF_RTC1->EVENTS_COMPARE[0] = 0;
	NRF_RTC1->EVENTS_COMPARE[1] = 0;
	NRF_RTC1->EVENTS_COMPARE[2] = 0;
	NRF_RTC1->EVENTS_COMPARE[3] = 0;

	NVIC_EnableIRQ(RTC1_IRQn);
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
		/* Do nothing. */
	}

	/* Also startup the LFCLK for use for the RTC, This clock is switched off
	 * when going to OFF mode.
	 * The xrystal takes 1.3uA current when starting and takes about 0.3 s to start.
	 * This is a really long time so disable the CPU during this time!
	 */

	/* Set clock source! And immediately tell the clock to start! because this takes time. */
	NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
	/* Start the clock right away */
	NRF_CLOCK->TASKS_LFCLKSTART = 1;

	/* Turn off all devices OFF for sure! */
	disable_all_peripherals();

	/* now ensure that there is a interrupt generated! */
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	wait_for_lfclk = true;
	NRF_CLOCK->INTENSET = (uint32_t)1<<1;
	NVIC_EnableIRQ(POWER_CLOCK_IRQn);
	/* Wait for the external oscillator to start up. */
	while (wait_for_lfclk) {
		/* turn off CPU */
		__WFI();

		/* clear the interrupt */
		NVIC_DisableIRQ(POWER_CLOCK_IRQn);
		NRF_CLOCK->INTENCLR = (uint32_t)1<<1;
	}

	/* Debug info via UART when SWD is not connected. */
#ifdef UART_DEBUG
	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, \
		RX_PIN_NUMBER, UART_BAUDRATE_BAUDRATE_Baud115200, 12, HWFC);
#endif

	printf("-1, -----------------------\n"); /* the -1 is easy filter out this text */
	printf("-1, Starting Mobile node...\n");
	printf("-1, -----------------------\n");

	init_ortogonal_codes();

	/* Init all RADIO components */
	NRF_RADIO->POWER = 1;
	radio_configure();
	set_radio_shortcuts();

	/* Init all peripherals */
	/* Init RTC timer for PWAKE timout stuff */
	init_rtc0();

	/* Init RTC1 timer for all other stuff */
	init_rtc1();
	init_adc();
	init_gpio();
}


/* ---------------------------------------------------------------------------*/
/* Functions used in the main program */
/* ---------------------------------------------------------------------------*/

/**
 * The function wil calculate how much bit errors there are between two codes.
 * @param code1 code one
 * @param code2 code two
 */
int calc_hamming_dst(uint16_t code1, uint16_t code2) {
	uint16_t bits;
	int result;

	bits = code1 ^ code2;

	result = 0;
	/* (the number of nonzero bits) using an algorithm of Wegner (1960) that 
	repeatedly finds and clears the lowest-order nonzero bit.*/
	while(bits != 0) {
		bits &= (bits-1);
		result++;
	}
	return result;
}

/**
 * This function decodes the incomming packets with orthogonal codes.
 * @param  encoded pointer to a received location-reply packet.
 */
static uint16_t decode_pckt(uint16_t *encoded)
{
	uint16_t decoded = 0;
	uint16_t res[9] = {0,0,0,0,0,0,0,0,0};
	uint16_t result = 0;

	/* Loop over node IDs */
	for (int m = 0; m < 9; m++) {
		for (int k = 0; k < BIT_SIZE_ENCODED_NODE_ID; k++) {
			/* Do actual decoding of the packet
			 * First XOR the first value with pos_code[m] wich will result in
			 * all 0 if the code is equal.
			 */			
			decoded = encoded[k] ^ pos_code[m];
			//printf("decoded: %d, end: %d, pos_c: %d\n", decoded, encoded[k], pos_code[m]);

			int tmp = 0;
			for (int l = 0; l < 16; l++) {
				if (decoded & (1 << l))
					tmp -= 1;
				else
					tmp += 1;
			}			
			if (tmp > 0) {
				res[m] |= (1 << (BIT_SIZE_ENCODED_NODE_ID - k - 1) );
			}
		}
		
		if (res[m] != 0) {
			if (calc_hamming_dst(node_nr[m],res[m]) < 4) {
				/* encode the results in the bit poitions */
				result |= 1 << m;
			}
		}
	}
	return result;
}

/* This function stores the result on a new location */
static void save_result(uint16_t *encoded, uint8_t round)
{
	if(round == 0) {
		memcpy(first_result, encoded, 30);
	} else if (round == 1) {
		memcpy(second_result, encoded, 30);
	}
}

/* Controls the transmit messages */
static void transmit(uint16_t *this_packet)
{
	/* First set the RADIO packet pointer before enableing the RADIO. */
	NRF_RADIO->PACKETPTR = (uint32_t)this_packet;

	/* clear the DISABLED EVENT first */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	/* Enable radio and wait for ready. */
	NRF_RADIO->TASKS_TXEN   = 1U;
}

/* Sets the RADIO in receiving mode */
static void receive(void)
{
	/* Set payload pointer. before enableing the RADIO */
	NRF_RADIO->PACKETPTR = (uint32_t)rpacket;

	/* clear the DISABLED EVENT first */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	radio_disabled = 0;
	NRF_RADIO->TASKS_RXEN = 1U;
}

/**
 * Initiates the ADC check.
 */
static void check_adc(void)
{
	/* Enable Dset on the harvester to mesure Dout */
	nrf_gpio_pin_set(2);
	nrf_delay_us(45);

	/* Clear the ADC event_end for sure */
	NRF_ADC->EVENTS_END = 0;
	/* start new ADC sample */
	NRF_ADC->TASKS_START = 1;
}

static int sum(int array[], int size)
{
	int i, sum=0;
	for(i=0; i<size; i++) {
		sum = sum + array[i];
	}
	return sum;
}

static int mean(int array[], int size)
{
	if(size==0) return 0;
	return sum(array, size)/size;
}

/**
 * Creates a location request packet for anchors or WPAs
 * @param
 */
static void prepare_packet(int level)
{
	if(level == 0) {
		tpacket[0] = GROUP_ID_ANCHOR;
		tpacket[1] = POWER_CAST_TURN_OFF_TIME;

		tpacket[2] = (uint16_t)mesurement;
		tpacket[3] = rx_something[0];
		tpacket[4] = rx_rssi[0];
		tpacket[5] = rx_crc_ok[0];
		tpacket[6] = rx_result[0];

		tpacket[7] = rx_something[1];
		tpacket[8] = rx_rssi[1];
		tpacket[9] = rx_crc_ok[1];
		tpacket[10] = rx_result[1];
	} else {
		tpacket[0] = GROUP_ID_WPA;
		tpacket[1] = 0;
	}
} 


/**
 * This function determines for the state machine if you should switch depening 
 * on global vars that are updated by interrupts.
 * @return the new state(or the same)
 */
static wiploc_state_t update_state(wiploc_state_t current_state)
{
	/* check int flags and change state. */
	switch (current_state) {
		case STATE_SLEEP:
			if(timer0 > 0) {

				NRF_RTC1->INTENCLR = 1<<16;
				timer0 = 0;
				return STATE_TRANSMIT_FIRST;

			} else if(radio_disabled > 0) {

				radio_disabled = 0;
				return STATE_SLEEP;

			} else if (stop_receiving) {

				stop_receiving = false;
				return STATE_SLEEP;

			}
			break;
		case STATE_TRANSMIT_FIRST:
			if(radio_disabled > 0) {

				radio_disabled = 0;
				return STATE_RECEIVE_FIRST;

			}
			break;
		case STATE_RECEIVE_FIRST:
			if(radio_disabled > 0) {

				/* recieved something save result! */
				radio_disabled = 0;

				/* Save the results */
				rx_something[0] = 1;
				rx_rssi[0] = (uint16_t)NRF_RADIO->RSSISAMPLE;
				rx_crc_ok[0] = (uint16_t)NRF_RADIO->CRCSTATUS;
				save_result(rpacket, 0);
				
				/* Clear timeout[1] interrupt */
				NRF_RTC1->INTENCLR = 1<<17;

				return STATE_WAIT_FOR_TRANSMIT;

			} else if (stop_receiving) {

				stop_receiving = false;
				NRF_RADIO->TASKS_DISABLE = 1U;
				/* Clear timeout[1] interrupt */
				NRF_RTC1->INTENCLR = 1<<17;

				rx_something[1] = 0;
				

				return STATE_SLEEP;

			}
			break;
		case STATE_WAIT_FOR_PWAKE:
			if(radio_disabled > 0) {

				radio_disabled = 0;

				return STATE_WAIT_FOR_PWAKE;

			} else if (stop_receiving) {

				stop_receiving = false;
				return STATE_WAIT_FOR_PWAKE;

			} else if (pwake > 0) {

				pwake = 0;
				NRF_RTC1->INTENCLR = 1<<18;

				/* Pwake was detected so stop the timer and clear the interrupt */
				NRF_RTC0->INTENCLR = 1<<16;
				NRF_RTC0->TASKS_STOP = 1;

				return STATE_WAIT_FOR_TRANSMIT;

			} else if (pwake_timeout > 0) {

				pwake_timeout = 0;
				NRF_RTC1->INTENCLR = 1<<18;

				/* no pwake was detected so stop the RTC0 timer and clear the interrupt */
				NRF_RTC0->INTENCLR = 1<<16;
				NRF_RTC0->TASKS_STOP = 1;

				/* There was no PWAKE detected, asume that it is missed and contiune second round */
				return STATE_TRANSMIT_SEC;

			}
			break;
		case STATE_WAIT_FOR_TRANSMIT:
			if(wait_for_transmit>0) {

				wait_for_transmit = 0;
				NRF_RTC1->INTENCLR = 1<<19;
				return STATE_TRANSMIT_SEC;
			}
			break;
		case STATE_TRANSMIT_SEC:
			if(radio_disabled>0) {
				return STATE_RECEIVE_SEC;
			}
			break;
		case STATE_RECEIVE_SEC:
			if(radio_disabled > 0) {
				
				radio_disabled = 0;
				/* Clear timeout[1] interrupt */
				NRF_RTC1->INTENCLR = 1<<17;

				/* Save the results */
				rx_something[1] = 1;
				rx_rssi[1] = (uint16_t)NRF_RADIO->RSSISAMPLE;
				rx_crc_ok[1] = (uint16_t)NRF_RADIO->CRCSTATUS;
				save_result(rpacket, 1);

				return STATE_SLEEP;

			} else if (stop_receiving) {
				stop_receiving = false;
				NRF_RADIO->TASKS_DISABLE = 1U;
				/* Clear timeout[1] interrupt */
				NRF_RTC1->INTENCLR = 1<<17;

				rx_something[1] = 0;

				return STATE_SLEEP;

			}

			break;
	}
	return current_state;
}


/** 
 * This function handles everything when first entering a new state 
 * @param
 */
static void handle_state(wiploc_state_t current_state)
{
	static wiploc_state_t old_state = STATE_SLEEP;
	int val, value, voltage;
	int reference = 12;

	if (old_state == current_state) {
		return;
	}

	switch (current_state) {
		case STATE_SLEEP:

			/* after 20 mesurements give 10 seconds to change posistions */
			if (mesurement > 1 && mesurement%20 == 0) {
				NRF_RTC1->CC[0] = NRF_RTC1->CC[0] + WAITING_TIME;
			} else {
				NRF_RTC1->CC[0] = WAKEUP_PERIOD;
			}

			/* Enable interrupt for Compare event on CC[0] */
			if (NRF_RTC1->COUNTER < NRF_RTC1->CC[0]) {
				NRF_RTC1->INTENSET = 1<<16;
			} else {
				printf("ERROR: round took more then wake up period!\n");
				NRF_RTC1->INTENSET = 1<<16;
				NRF_RTC1->TASKS_CLEAR = 1;
			}

			if(rx_something[0] == 1) {
				rx_result[0] = decode_pckt(first_result);
			} else {
				rx_result[0] = 0;
			}
			if(rx_something[1] == 1) {
				rx_result[1] = decode_pckt(second_result);
			} else {
				rx_result[1] = 0;
			}
			
			printf("%d, %d, %d, %d, %d,  ", (int)mesurement, (int)rx_something[0], (int)rx_rssi[0], (int)rx_crc_ok[0], (int)rx_result[0]);
			printf("%d, %d, %d, %d\n", (int)rx_something[1], (int)rx_rssi[1], (int)rx_crc_ok[1], (int)rx_result[1]);
			prepare_packet(0);
			
			/* init everything for new round */
			mesurement++;
			rx_round = 0;

			rx_something[0] = 0;
			rx_rssi[0] 		= 0;
			rx_crc_ok[0] 	= 0;
			rx_result[0] 	= 0;

			rx_something[1]	= 0;
			rx_rssi[1] 		= 0;
			rx_crc_ok[1] 	= 0;
			rx_result[1] 	= 0;

			/* turn on ADC checks for dout */
			NRF_RTC0->INTENSET = 1<<16;
			NRF_RTC0->TASKS_CLEAR = 1;
			NRF_RTC0->TASKS_START = 1;
			NRF_RTC0->CC[0] = CHECK_PERIOD_MN;

			break;
		case STATE_TRANSMIT_FIRST:
			NRF_RTC0->INTENCLR = 1<<16;
			NRF_RTC0->TASKS_STOP= 1;

			val = mean(dout_values, DOUT_ARRAY_SIZE);


			value = val*100;
			voltage = value/1023*reference/2*3;
			tpacket[11] = CMD_SLEEP;			
			if (voltage < 200) {
				//printf("Activated back side.so deactivate the front\n");
				tpacket[12] = (uint16_t)0x0;
				tpacket[12] |= (uint16_t)0x1;
				tpacket[12] |= (uint16_t)0x1<<1;
			} else {
				tpacket[12] = (uint16_t)0x0;
				tpacket[12] |= (uint16_t)0x1<<2;
				tpacket[12] |= (uint16_t)0x1<<3;
			}
			tpacket[13] = voltage;
			printf("%d, %d\n", tpacket[12], tpacket[13]);
			
			transmit( tpacket );

			break;
		case STATE_RECEIVE_FIRST:

			/* Enable the interrupt for RTC_cctimer[1] */
			NRF_RTC1->CC[1] = NRF_RTC1->COUNTER + RX_TIMEOUT;
			NRF_RTC1->INTENSET = 1<<17;
			receive();

			break;
		case STATE_WAIT_FOR_PWAKE:
			/* Frist check the ADC if there is already a passive Wake UP */

			/* Enable interrupt for rtc1_cc[2] if no pwake is detected! */
			NRF_RTC1->CC[2] = NRF_RTC1->COUNTER + PWAKE_TIMEOUT_VALUE;
			NRF_RTC1->INTENSET = 1<<18;

			/* Enable interrupt for rtc0_cc[0] and let the ADC check every 10ms */
			NRF_RTC0->INTENSET = 1<<16;
			NRF_RTC0->TASKS_CLEAR = 1;
			NRF_RTC0->TASKS_START = 1;

			break;
		case STATE_WAIT_FOR_TRANSMIT:
			/* Wait for transmit so set the interrupt! */
			NRF_RTC1->CC[3] = NRF_RTC1->COUNTER + WAIT_FOR_TRANSMIT_TIME;
			NRF_RTC1->INTENSET = 1<<19;
			prepare_packet(1);

			break;
		case STATE_TRANSMIT_SEC:

			transmit( tpacket );

			break;
		case STATE_RECEIVE_SEC:

			/* Enable the interrupt for cc[1] rx timeoout */
			NRF_RTC1->CC[1] = NRF_RTC1->COUNTER + RX_TIMEOUT;
			NRF_RTC1->INTENSET = 1<<17;

			receive();

			break;
	}
	old_state = current_state;
}



/**
 * @brief Function for application main entry.
 */
int main(void)
{
	/* Set start state */
	wiploc_state_t wiploc_state = STATE_SLEEP;

	/* init all systems */
	system_init();

	/* This is a global var that stops receiving if a timeout is triggerd. */
	stop_receiving = false;

	/* This is a mesurement var that keeps track of which mesurement for logging */
	mesurement = -1;

	/* Start the RTC to enable periodically wake up (t_m)*/
	NRF_RTC1->TASKS_START = 1;

	while (true) {
		if ( !(timer0 || radio_disabled || stop_receiving || pwake || wait_for_transmit || pwake_timeout) ) {
			__WFI();
		}
		wiploc_state = update_state(wiploc_state);
		handle_state(wiploc_state);
	}
}

/* ---------------------------------------------------------------------------*/
/* IRQ Functions handlers*/
/* ---------------------------------------------------------------------------*/
void RTC1_IRQHandler(void)
{
	if(NRF_RTC1->EVENTS_COMPARE[0] == 1) {
		NRF_RTC1->EVENTS_COMPARE[0] = 0;

		/* Reset the timer to begin a new a new round */
		NRF_RTC1->TASKS_CLEAR = 1;

		timer0++;

	} else if(NRF_RTC1->EVENTS_COMPARE[1] == 1) {
		NRF_RTC1->EVENTS_COMPARE[1] = 0;

		stop_receiving= true;

	} else if(NRF_RTC1->EVENTS_COMPARE[2] == 1) {
		NRF_RTC1->EVENTS_COMPARE[2] = 0;

		pwake_timeout++;

	} else if (NRF_RTC1->EVENTS_COMPARE[3] == 1) {
		NRF_RTC1->EVENTS_COMPARE[3] = 0;

		wait_for_transmit++;
	}
}

/**
 * This functions handles the RTC0 interrupt and starts the ADC.
 */
void RTC0_IRQHandler(void)
{
	if (NRF_RTC0->EVENTS_COMPARE[0] == 1) {
		NRF_RTC0->EVENTS_COMPARE[0] = 0;

		/* Enable Dset on the Energy haverster */
		nrf_gpio_pin_set(2);
		nrf_delay_us(45);

		/* Clear the ADC event_end for sure */
		NRF_ADC->EVENTS_END = 0;
		/* start new ADC sample */
		NRF_ADC->TASKS_START = 1;
	}
}

void ADC_IRQHandler(void)
{
	NRF_ADC->EVENTS_END = 0;
	nrf_gpio_pin_clear(2);
	//printf("%d\n", NRF_ADC->RESULT);
	if (NRF_ADC->RESULT < ADC_TRESHOLD) {
		/* Set global flag */
		//pwake++;

	} else {
		push(dout_values,DOUT_ARRAY_SIZE, (int)NRF_ADC->RESULT);
	}
	NRF_RTC0->TASKS_CLEAR = 1;
}

void POWER_CLOCK_IRQHandler(void)
{
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	wait_for_lfclk = false;
}

void RADIO_IRQHandler(void)
{
	NRF_RADIO->EVENTS_DISABLED = 0;
	radio_disabled++;
}




