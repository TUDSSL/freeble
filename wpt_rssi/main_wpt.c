/**
 * This is the source file for the experiment to compare the RSS of WPT and RSSI iofBLE
 */

#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "nrf_delay.h"
#include "radio_config.h"
#include "boards.h"

#include "nordic_common.h"
#include <string.h>
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
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


#define RX_TIMEOUT   (uint32_t)5 /* ms */

#define ANCHOR_BEACON_CODE 95
#define CHARBEE_CODE       90

#define CMD_SLEEP          20 /* 20 means sleep */

#define TOTAL_MESUREMENTS_PER_F 1000
#define DELAY 6/* ms */


/* This is the Node ID or something? */
static int  nn = BEACON;
static bool keep_listening = true;
static bool wait_for_lfclk;
static int radio_disabled;
static int rx_timeout;
static int rssi;

static int mesurement = 0;


/* The spreading codes */
#define MAX_NODES 9
//static uint16_t pos_code[MAX_NODES] = {52332, 39321, 61680, 42405, 49635, 38550, 65280, 43605, 52275}; //old codes
static uint16_t pos_code[MAX_NODES] = {65535, 43690, 52428, 39321, 61680, 42405, 50115, 38550, 65280};
static uint16_t neg_code[MAX_NODES];

/* Forward error correcting layer for the ID */
static uint16_t node_id_array[9] = {21845, 13107, 26214, 3855, 23130, 15420, 26985, 255, 21930};
static uint8_t bit_size_node_id = 15;
static uint16_t bit_size_rssi = 7;

static uint8_t tpacket[PACKET_STATIC_LENGTH];  /* Packet to transmit. */
static uint8_t rpacket[PACKET_STATIC_LENGTH];  /* Packet to receive. */


static void trim_bluetooth(void)
{
	/* Before the RADIO can be used in BLE_1mbit mode there need to be some trim
	 * values overwritten.
	 */
	NRF_RADIO->OVERRIDE0 = NRF_FICR->BLE_1MBIT[0];
	NRF_RADIO->OVERRIDE1 = NRF_FICR->BLE_1MBIT[1];
	NRF_RADIO->OVERRIDE2 = NRF_FICR->BLE_1MBIT[2];
	NRF_RADIO->OVERRIDE3 = NRF_FICR->BLE_1MBIT[3];
	NRF_RADIO->OVERRIDE4 = NRF_FICR->BLE_1MBIT[4];

	/* Enable the Override registers */
	NRF_RADIO->OVERRIDE4 |= (1UL << RADIO_OVERRIDE4_ENABLE_Pos);
}

/**
 * The function wil calculate how much bit errors there are between two codes.
 * @param code1 [description]
 * @param code2 [description]
 */
int calc_hamming_dst(uint16_t code1, uint16_t code2) {
	uint16_t bits;
	int result;

	bits = code1 ^ code2;

	result = 0;
	/* (the number of nonzero bits) using an algorithm of Wegner (1960) that repeatedly finds and clears the lowest-order nonzero bit.*/
	while(bits != 0) {
		bits &= (bits-1);
		result++;
	}

	return result;
}

/**
 * Disables and turns off all the peripherals
 * 
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
 * @brief Function to set shorcuts in the RADIO module to send without CPU intervention.
 * This does a little speedup...
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
}

/* Init the RTC timer 0! */
static void init_rtc0(void)
{
	NRF_RTC0->POWER = 1;
	/* Init the RTC timer! */
	/* Set the frequency to 1000 Hz. */
	NRF_RTC0->PRESCALER = 32;
	/* Enable interrupt for Compare event on CC[0] */
	NRF_RTC0->INTENSET = RTC_INTENSET_COMPARE0_Msk;
	NRF_RTC0->CC[0] = RX_TIMEOUT;

	NRF_RTC0->TASKS_CLEAR = 1;
	NVIC_EnableIRQ(RTC0_IRQn);
}

static void generate_packet(uint8_t node_id)
{
	uint16_t *pt_packet;
	uint16_t encoded_node_id = node_id_array[node_id];
	uint16_t bit = 1 << (bit_size_node_id-1); /* 15 - 1 = 14, 1 << 14 opschuiven */

	if(node_id>=MAX_NODES){
		printf("ERROR: cannot generate spreading codes because node_id>=MAX_NODES.\n");
		return;
	}

	/* Generate negative codes from positive codes */
	for(int i = 0; i<MAX_NODES; i++) {
		neg_code[i] = ~pos_code[i];
	}

}

/* This function sets the digital output for controlling Dset on the power harverster */
static void init_gpio(void)
{
	nrf_gpio_cfg_output(2);
	nrf_gpio_pin_clear(0);
}

static void init_adc(void)
{
	NRF_ADC->POWER=1;

	NRF_ADC->CONFIG = 2; /* RES 10 bit resolution */
	NRF_ADC->CONFIG |= 1<<2;  /* INPSEL AnalogInputNoPrescaling */
	NRF_ADC->CONFIG |= 0<<5;  /* REFSEL VBG*/
	NRF_ADC->CONFIG |= (uint32_t)1<<10; /* PSEL AIN 2*/

	NRF_ADC->ENABLE = 1;     /* READY event */
}

static void blink_led(int nn)
{
	int i;

	nrf_gpio_cfg_output(LED_RGB_RED);
	nrf_gpio_cfg_output(LED_RGB_GREEN);
	nrf_gpio_cfg_output(LED_RGB_BLUE);
	nrf_gpio_pin_set(LED_RGB_RED);
	nrf_gpio_pin_set(LED_RGB_GREEN);
	nrf_gpio_pin_set(LED_RGB_BLUE);

	for(i=0; i<nn; i++) {
		nrf_gpio_pin_clear(LED_RGB_RED);
		nrf_delay_ms(200);
		nrf_gpio_pin_set(LED_RGB_RED);
		nrf_delay_ms(200);
	}
	nrf_gpio_pin_clear(LED_RGB_GREEN);
}

static void init_timer2(void)
{
	NRF_TIMER0->POWER     = 1;

	NRF_TIMER0->PRESCALER = 0; /* Ftimer = 16/(2^4) = 1.000.000 hz is 1 us. */
	NRF_TIMER0->BITMODE   = 3; /* 1=8bit, 3=32bit timer */
	NRF_TIMER0->MODE      = 0;

	/* Stop and clear the timer for sure */
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->TASKS_STOP  = 1;

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
	/* Set clock source! And immediately tell the clock to start! because this takes time. */
	NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
	/* Start the clock right away */
	

	/* Turn off all devices for sure! */
	disable_all_peripherals();

	/* now ensure that there is a interrupt generated! */
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	wait_for_lfclk = true;
	NRF_CLOCK->INTENSET = (uint32_t)1<<1;
	NVIC_EnableIRQ(POWER_CLOCK_IRQn);

	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	/* Wait for the external oscillator to start up. */
	while (wait_for_lfclk) {
		/* turn off CPU */
		__WFI();

		/* clear the interrupt */
		if(wait_for_lfclk==false) {
			NVIC_DisableIRQ(POWER_CLOCK_IRQn);
			NRF_CLOCK->INTENCLR = (uint32_t)1<<1;
		}
	}


#ifdef UART_DEBUG
	/* Debug info via UART when SWD is not connected. */
	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, \
		RX_PIN_NUMBER, UART_BAUDRATE_BAUDRATE_Baud115200, 12, HWFC);
#endif

#ifdef BOARD_PCA20006
	blink_led(nn);
#endif

	init_rtc0();
	init_gpio();
	init_adc();

	init_timer2();

	/* Set radio configuration parameters. */
	/* Init radio stuff */
	NRF_RADIO->POWER = 1;
	radio_configure();
	set_radio_shortcuts();
	NRF_RADIO->INTENSET = 1<<4;
	NVIC_EnableIRQ(RADIO_IRQn);

	/* the packet is only generated once, so if the node id changes it needs to be generated again. */
	generate_packet(nn);

	tpacket[0] = ANCHOR_BEACON_CODE; /* specify the receiver nodes group */
	tpacket[1] = 1;	/* let ANBE node know that it is a sync pulse */
	tpacket[2] = 0; /* Freqency */
	NRF_RADIO->FREQUENCY = 0;

}



static void print_array(uint16_t array[], int size)
{
	printf("{ ");
	for(int i=0; i<size; i++) {
		printf("%d ", array[i]);
	}
	printf("}\n");
}

static void print_bits(uint16_t bits, int size)
{
	printf("0b");
	for(int i=size-1; i>=0; i--) {
		printf("%d", (bits>>i)&0x1 );
	}
	printf("\n");
}

/**
 * This functions does all the decoding of the incomming packet.
 */
static void transmit()
{
	/* Set the transmit packet pointer */
	NRF_RADIO->PACKETPTR = (uint32_t)tpacket;

	/* Enable the radio and start transmission and wait for a disable event */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	radio_disabled = 0;
	NRF_RADIO->TASKS_TXEN   = 1;

	while(radio_disabled == 0) {
	}
}

static void receive()
{
	/* Set payload pointer. */
	NRF_RADIO->PACKETPTR    = (uint32_t)rpacket;

	/* start receiving */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	radio_disabled = 0;
	rx_timeout = 0;
	NRF_RTC0->TASKS_START = 1;
	NRF_RADIO->TASKS_RXEN   = 1U;

	while(radio_disabled == 0 && rx_timeout == 0) {
	}

	/* Proper shutdown of the radio. */
	if(rx_timeout) {
		NRF_RADIO->TASKS_DISABLE  = 1U;
		keep_listening = false;
		rssi = 0;
		
	} else {

		rssi = (int)NRF_RADIO->RSSISAMPLE;
	}


}

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


static uint16_t check_adc(void)
{
	/* Enable Dset on the Energy haverster */
	nrf_gpio_pin_set(2);
	nrf_delay_us(45);

	/* Clear the ADC event_end for sure */
	NRF_ADC->EVENTS_END = 0;
	/* start new ADC sample */
	NRF_ADC->TASKS_START = 1;

	while(NRF_ADC->EVENTS_END==0){}
	nrf_gpio_pin_clear(2);

	return NRF_ADC->RESULT;
}

int main(void)
{
	system_init();

	int value;

	int voltage = 0;
	int reference = 12;
	NRF_TIMER0->TASKS_START  = 1;

	while(true) {
		value = (int)check_adc()*100;
		voltage = value/1023*reference/2*3;

		transmit();
		receive();

		printf(" %d, %d, %d, %d\n", mesurement, value/100, voltage, rssi);
		mesurement++; 

		nrf_delay_ms(10);

		if(mesurement>0 && mesurement%10==0){
			printf(" -1, 4\n");
			nrf_delay_ms(1000);
			printf(" -1, 3\n");
			nrf_delay_ms(1000);
			printf(" -1, 2\n");
			nrf_delay_ms(1000);
			printf(" -1, 1\n");
			nrf_delay_ms(1000);
		}
	}
}


void RTC0_IRQHandler(void)
{
	NRF_RTC0->EVENTS_COMPARE[0] = 0;
	NRF_RTC0->TASKS_STOP = 1;
	NRF_RTC0->TASKS_CLEAR = 1;

	rx_timeout++;
}

void RTC1_IRQHandler(void)
{
	NRF_RTC1->EVENTS_COMPARE[0] = 0;
	NRF_RTC1->TASKS_STOP = 1;
	NRF_RTC1->TASKS_CLEAR = 1;
	nrf_gpio_pin_set(1);
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


