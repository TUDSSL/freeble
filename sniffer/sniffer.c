/**
 * This is the source file for the charbeacon node.
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


#define RX_TIMEOUT   (uint32_t)10 /* ms */


#define BLE_FREQUENCY       0 /*2400 mHz + this value */
#define ANCHOR_BEACON_CODE 95
#define CHARBEE_CODE       90

#define CMD_SLEEP          20 /* 20 means sleep */


/* This is the Node ID or something? */
static int  nn = BEACON;
static bool keep_listening = true;
static bool wait_for_lfclk;
static int radio_disabled;
static int rx_timeout;

/* The spreading codes */
#define MAX_NODES 9
//static uint16_t pos_code[MAX_NODES] = {52332, 39321, 61680, 42405, 49635, 38550, 65280, 43605, 52275}; //old codes
static uint16_t pos_code[MAX_NODES] = {65535, 43690, 52428, 39321, 61680, 42405, 50115, 38550, 65280};
static uint16_t neg_code[MAX_NODES];

/* Forward error correcting layer for the ID */
static uint16_t node_id_array[9] = {21845, 13107, 26214, 3855, 23130, 15420, 26985, 255, 21930};
static uint8_t bit_size_node_id = 15;
static uint8_t bit_size_rssi = 7;

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

/* Init the RTC timer 1 ! */
// static void init_rtc1(void)
// {
// 	/* Init the RTC timer! */
// 	/* Set the frequency to 100 Hz. */
// 	NRF_RTC1->PRESCALER = 327;
// 	 Enable interrupt for Compare event on CC[0] 
// 	NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;
// 	NRF_RTC1->CC[0] = RTC_TIMEOUT;

// 	NRF_RTC1->TASKS_CLEAR = 1;
// 	NVIC_EnableIRQ(RTC1_IRQn);
// }

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
		//printf("neg: %d, pos: %d\n", neg_code[i], pos_code[i]);
	}

	/* make uint16 pointer for the tpacket to store the values in two bytes */
	pt_packet = (uint16_t *)tpacket;

	/* Multiply spreading code for every bit of the encoded_node_id */
	for (int z = 0; z<bit_size_node_id; z++) {
		if (encoded_node_id & bit) {
			 pt_packet[z] = pos_code[node_id];
		} else {
			 pt_packet[z] = neg_code[node_id];
		}
		bit >>= 1; /* shift one bit right */
	}
}

static void init_gpio(void)
{
	/* set AIN2 as output pin and set it HIGH */
	nrf_gpio_cfg_output(1);
	nrf_gpio_pin_set(1);
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
	NRF_CLOCK->TASKS_LFCLKSTART = 1;

	/* Turn off all devices for sure! */
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

	printf("\n\n-1, -----------------------\n");
	printf("-1, Starting sniffer\n");
	printf("-1, -----------------------\n");

	/* Uncomment the line below if the NRF51822 chip has a revision of
	 * 2.0 or higher */
	//trim_bluetooth();

	//init_rtc0();
	//init_rtc1();

	/* Set radio configuration parameters. */
	/* Init radio stuff */
	NRF_RADIO->POWER = 1;
	radio_configure();
	set_radio_shortcuts();
	NRF_RADIO->INTENSET = 1<<4;
	NRF_RADIO->FREQUENCY = BLE_FREQUENCY;
	NVIC_EnableIRQ(RADIO_IRQn);

	/* the packet is only generated once, so if the node id changes it needs to be generated again. */
	generate_packet(nn);

	//tpacket[0] = ANCHOR_BEACON_CODE; /* specify the receiver nodes group */
	//tpacket[1] = 1;	/* let ANBE node know that it is a sync pulse */


	//init_gpio();
}

static void power_cycle()
{
	nrf_gpio_pin_clear(1);
	NRF_RTC1->TASKS_START = 1;
}

/**
 * This functions does all the decoding of the incomming packet.
 */
static void decode_packet(void)
{
	printf("Decoding...\n");
}


static void transmit()
{
	/* Set the transmit packet pointer */
	NRF_RADIO->PACKETPTR = (uint32_t)tpacket;

	//printf("tpckt: [%d, %d]\n", tpacket[0], tpacket[1]);

	/* Enable the radio and start transmission and wait for a disable event */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	radio_disabled = 0;
	NRF_RADIO->TASKS_TXEN   = 1;

	while(radio_disabled == 0) {
		//__WFI();
	}
}

static void receive()
{
	/* Set payload pointer. */
	NRF_RADIO->PACKETPTR    = (uint32_t)rpacket;

	/* start receiving */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	radio_disabled = 0;

	NRF_RADIO->TASKS_RXEN   = 1U;

	while(radio_disabled == 0 && rx_timeout == 0) {
		//printf("WFI\n");
		
		//__WFI();
	}

	/* Proper shutdown of the radio. */
	if(rx_timeout) {
		NRF_RADIO->TASKS_DISABLE  = 1U;
		keep_listening = false;
		printf("ERROR: RX_TIMEOUT!\n");
	} else {

		uint16_t *pointer;
		pointer = (uint16_t *)rpacket;

		if(NRF_RADIO->CRCSTATUS) {

			printf("%d, %d, %d, %d, %d\n", pointer[2], pointer[3], pointer[4], pointer[5], pointer[6]);

		}
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

static void set_transmit_timeout(void)
{
	/* Set global true and start transmitting */
	NRF_RTC0->TASKS_START = 1;
}

int main(void) {
	
	system_init();

	//printf("receiving...\n");
	while(true) {

		receive();
		
	}
}


void RTC0_IRQHandler(void)
{
	NRF_RTC0->EVENTS_COMPARE[0] = 0;
	NRF_RTC0->TASKS_STOP = 1;
	NRF_RTC0->TASKS_CLEAR = 1;

	rx_timeout++;
	//printf("\n\nRTC0 interrupt!!\n\n");
}

void RTC1_IRQHandler(void)
{
	NRF_RTC1->EVENTS_COMPARE[0] = 0;
	NRF_RTC1->TASKS_STOP = 1;
	NRF_RTC1->TASKS_CLEAR = 1;
	nrf_gpio_pin_set(1);
	//printf("RTC interrupt!!\n");
}

void HardFault_Handler(void)
{
	printf("ERROR!!! HARD FAULT\n");
	nrf_delay_ms(1000);
	NVIC_SystemReset();
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


