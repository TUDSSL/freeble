/**
 * This is the source file for the beacon node.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
 #include <string.h>

#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "boards.h"
#include "nordic_common.h"

#include "radio_config.h"

/* For debugging and printf() */
#include "simple_uart.h"
#include "SEGGER_RTT.h"


//#define LED_ON 1
#undef LED_ON

/* This ifndef ensures that there is no code compiled used for debugging! */
#ifndef DEBUG
#define printf(str,...)    do{}while(0)
#endif

/* This ifndef defines the beacon number */
#ifndef BEACON
#define BEACON             0
#endif

/* -----------------------------------------------------------------------------
 * All the gobal and local variables
 * -----------------------------------------------------------------------------
 */

#define BLE_TXPOWER         RADIO_TXPOWER_TXPOWER_0dBm
#define BLE_FREQUENCY       0 /*2400 mHz + this value */

#define CHECK_PERIOD_MN    2  /* ms */
#define CHECK_PERIOD       10  /* ms, period for the ADC to check for an passive wake up */
#define DOUT_SETTLE_TIME   50  /* us, This time is needed for Dout to become stable */
#define RX_TIMEOUT         (CHECK_PERIOD + CHECK_PERIOD_MN)  /* ms */

#define PAUSE_TIME          400  /* ms */
#define DISABLE_CHECK_VALUE 500  /* ms */

#define GROUP_ID_WPA 95
#define GROUP_ID_ANCHOR       90

#define ADC_TRESHOLD       6    /* from 0 to 255, 0 tot 1.2 V */

/* commands used in tpackets */
#define CMD_SLEEP          20
#define CMD_SET_TXPOWER    10

/* This is the Node ID or something? */
int nn = BEACON;

/* This global indicates if the beacon sleeps or is listening  signals.*/
static bool sleep;
static bool wait_for_lfclk;
static bool transmit_back;
static bool stop_receiving;
static bool keep_listening;
static bool send_ack;
static int radio_disabled = 0;


uint16_t mesurements;


/* Orthogonal codes */
#define MAX_NODES 9
static uint16_t pos_code[MAX_NODES] = {65535, 43690, 52428, 39321, 61680, 42405, 50115, 38550, 65280};
static uint16_t neg_code[MAX_NODES];

/* FEC array */
#define BIT_SIZE_ENCODED_NODE_ID 	15
static uint16_t node_nr[MAX_NODES] = {21845, 13107, 26214, 3855, 23130, 15420, 26985, 255, 21930};

static uint16_t tpacket[30];  /**< Packet to transmit. */
static uint16_t ack_packet[30];  /**< Packet to transmit. */
static uint16_t rpacket[30];  /**< Packet to receive. */


/* ---------------------------------------------------------------------------*/
/* Functions used for initilization */
/* ---------------------------------------------------------------------------*/

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

	/* because the anchor beacons have a smaller range we can reduce the transmit power */
	NRF_RADIO->TXPOWER = (BLE_TXPOWER << RADIO_TXPOWER_TXPOWER_Pos);
	NRF_RADIO->INTENSET = 1<<4;
	NRF_RADIO->FREQUENCY = BLE_FREQUENCY;
	NVIC_EnableIRQ(RADIO_IRQn);
}

/**
 * This function generates the "location-reply" packet.
 */
static void generate_packet(void)
{
	uint16_t node = node_nr[nn];
	uint16_t bit = 1 << (BIT_SIZE_ENCODED_NODE_ID - 1); /* 15 - 1 = 14, 1 << 14 opschuiven */

	transmit_back = true;

	/* Generate negative codes from positive codes */
	for(int i = 0; i<MAX_NODES; i++) {
		neg_code[i] = ~pos_code[i];
		//printf("neg: %d, pos: %d\n", neg_code[i], pos_code[i]);
	}

	/* voor z van 0 tot 14 doe dit */
	for (int z = 0; z < BIT_SIZE_ENCODED_NODE_ID; z++) {
		if (node & bit) {
			 tpacket[z] = pos_code[nn];
		} else {
			 tpacket[z] = neg_code[nn];
		}
		bit >>= 1; /* shift one bit right */
	}
}

/**
 * This function is mainly used for debuggin purposes, it enables the RGB led on
 * on the smart BEACON kit to blink.
 */
static void blink_led(int nn)
{
	int i;

	/* Also initializes the pins */
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
	/* these are anchor beacons, so different color! */
	nrf_gpio_pin_clear(LED_RGB_BLUE);
}

/**
 * This function sets the digital output for controlling Dset on the power harverster
 */
static void init_gpio(void)
{
	nrf_gpio_cfg_output(2);
	nrf_gpio_pin_clear(2);	
	/* NOT USE GPIOTE, this increase power consumption */
}

/**
 * This function sets up the ADC peripheral. Select which pin it has to mesure
 * And enables a READY interrupt when te ADC result is ready.
 */
static void init_adc(void)
{
	NRF_ADC->POWER=1;

	NRF_ADC->CONFIG = 0;
	NRF_ADC->CONFIG |= (uint32_t)1<<10; /* PSEL AIN 2*/
	NRF_ADC->INTENSET = 1;
	NVIC_EnableIRQ(ADC_IRQn);

	NRF_ADC->ENABLE = 1;     /* READY event */
}

/* this functions connects the rtc compare interrupt to the ADC */
static void init_ppi(void)
{
	printf("Init_ppi, enable automatic Dout check.\n");
	
	/* Set the EVENT end point and TASK */
	NRF_PPI->CH[0].EEP = (uint32_t)(&NRF_TIMER0->EVENTS_COMPARE[0]);
	NRF_PPI->CH[0].TEP = (uint32_t)(&NRF_ADC->TASKS_START);

	/* use ppi channel 0 */
	NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);
	
	/* !!! DO NOT TURN THIS LINE ON! THIS WILL INCREASE POWER CONSUMPTION WITH 3mW */
	//NRF_POWER->TASKS_CONSTLAT = 1;
}

/**
 * This function initializes the RTC_timer 1 and set it freqency and enables the 
 * interrupt.
 */
static void init_rtc1(void)
{
	NRF_RTC1->POWER=1;
	/* Init the RTC timer! */
	NRF_RTC1->TASKS_STOP = 1;

	/* Set the frequency to 1000 Hz. */
	NRF_RTC1->PRESCALER = 32;

	/* Enable interrupt for Compare event on CC[0] */
	NRF_RTC1->INTENSET = 1<<16; /* Enable the event for PPI */
	/* the rest is interrupt based. */
	NRF_RTC1->INTENSET |= 1<<17;
    NRF_RTC1->INTENSET |= 1<<18;
	NRF_RTC1->CC[0] = CHECK_PERIOD;
	NRF_RTC1->CC[1] = RX_TIMEOUT;
	NRF_RTC1->CC[2] = PAUSE_TIME;

	/* Clear the RTC counter value for sure */
	NRF_RTC1->TASKS_CLEAR = 1;

	NRF_RTC1->EVENTS_COMPARE[0] = 0;
	NRF_RTC1->EVENTS_COMPARE[1] = 0;
	NRF_RTC1->EVENTS_COMPARE[2] = 0;

	NVIC_EnableIRQ(RTC1_IRQn);
}

/**
 * This function initializes the timer 0 and set it freqency and enables the 
 * interrupt.
 */
static void init_timer0(void)
{
	NRF_TIMER0->POWER     = 1;

	NRF_TIMER0->PRESCALER = 4; /* Ftimer = 16/(2^4) = 1.000.000 hz is 1 us. */
	NRF_TIMER0->BITMODE   = 1; /* 1=8bit, 3=32bit timer */
	NRF_TIMER0->MODE      = 0;

	NRF_TIMER0->CC[0]     = DOUT_SETTLE_TIME;
	NRF_TIMER0->SHORTS    = 1;     /* enable short from compare[0] to clear */
	NRF_TIMER0->SHORTS    = 1<<8;  /* enable short from compare[0] to stop */

	/* Stop and clear the timer for sure */
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->TASKS_STOP  = 1;
	NVIC_EnableIRQ(TIMER0_IRQn);
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
		NVIC_DisableIRQ(POWER_CLOCK_IRQn);
		NRF_CLOCK->INTENCLR = (uint32_t)1<<1;
	}

#ifdef UART_DEBUG
	/* Debug info via UART when SWD is not connected. */
	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, \
		RX_PIN_NUMBER, UART_BAUDRATE_BAUDRATE_Baud115200, 12, HWFC);
#endif

	printf("\n\n-----------------------\n");
	printf("Starting anchor Beacon node ID: %d\n", nn);
	printf("-----------------------\n");

#ifdef LED_ON
	blink_led(nn);
#endif

	/* Init radio stuff */
	NRF_RADIO->POWER = 1;
	radio_configure();
	set_radio_shortcuts();


	/* the packet is only generated once, so if the node number changes the beacon needs to be reset. */
	generate_packet();

	/* Init the GPIO to digital output to enable Dout mesurement */
	/* DO not use GPIOTE then the power consumption will go up! */
	init_gpio();

	/* Init the ADC to do ADC mesurement */
	init_adc();

	/* init the RTC and timer for all timer stuff */
	init_rtc1();
	init_timer0();

	/* init the PPI to do de ADC check on the background */
	init_ppi(); /* when the RTC1 timer starts it automaticly checks the Dout every 10 ms */

}

/* ---------------------------------------------------------------------------*/
/* Functions used in the main program */
/* ---------------------------------------------------------------------------*/

/**
 * This function does the actual data transmitting.
 */
static void transmit(uint8_t return_code)
{
	if (return_code == 0) {
	/* Set transmit packet pointer. */
	NRF_RADIO->PACKETPTR = (uint32_t)tpacket;

	} else if (return_code == 1) {
		ack_packet[0] = (uint16_t)NRF_RADIO->TXPOWER;
		NRF_RADIO->PACKETPTR = (uint32_t)ack_packet;
		send_ack = false;
	}

	NRF_RADIO->EVENTS_DISABLED = 0U;
	radio_disabled = 0;
	NRF_RADIO->TASKS_TXEN   = 1;

	while(radio_disabled == 0) {
		__WFI();
	}
}


/**
 * This function ensures the RADIO peripheral goes into receiving mode busy-waiting.
 */
static void receive()
{
	/* Set payload pointer. */
	NRF_RADIO->PACKETPTR    = (uint32_t)rpacket;

	/* start receiving */
	NRF_RADIO->EVENTS_DISABLED = 0U;
	radio_disabled = 0;
	NRF_RADIO->TASKS_RXEN   = 1U;

	while(radio_disabled == 0 && stop_receiving == false ) {
		__WFI();
	}

	/* never transmit back except when the right packet is recieved */
	transmit_back = false;

	/* Proper shutdown of the radio. */
	if(stop_receiving) {
		NRF_RADIO->TASKS_DISABLE  = 1U;
		keep_listening = false;
		printf("ERROR: RX_TIMEOUT!!!\n");
	} else {

		//printf("packet received...\n");
		/* Write received data to port 1 on CRC match. */
		if (NRF_RADIO->CRCSTATUS == 1U)
		{
			//printf("rpacket: [%d %d %d]\n", (int)rpacket[0], (int)rpacket[1], (int)rpacket[2]);
			if (rpacket[0] == GROUP_ID_WPA) {

				/* always send something back unless otherwise speciefied */
				transmit_back = true;

				/* as follows, beacon_type_code, command_code, command value */
				if (rpacket[1] == CMD_SLEEP) {
					/**
					 * CMD 20 means IdBasedPasiveWakeUp, next field is the length of the message
					 * rpacket[2] the ids are encoded in the bit positions
					 */
					
					/* First ensure that the anchor node keep listening, until we say otherwise. */
					keep_listening = true;

					if( ( (rpacket[2]>>nn) & 0x1 )==1){
						/* turn back to  sleep */
						keep_listening = false;
						transmit_back = false;
						sleep = true;
					}

				} else {
					keep_listening = false;
					printf("Normal packet received! do something!\n");
				}

			} else {
				printf("ERROR: wrong packet recieved: %d\n", rpacket[0]);
			}
		} else {
			printf("ERROR: CRCERROR\n");
		}
	}


}

/**
 * This function blinks the RGB led , with a green color. As much as defined in times.
 * @param int
 */
static void blink_blue(int times)
{
	int i;

	nrf_gpio_pin_set(LED_RGB_BLUE);

	for (i=0; i<times; i++) {
		nrf_gpio_pin_clear(LED_RGB_GREEN);
		nrf_delay_ms(100);
		nrf_gpio_pin_set(LED_RGB_GREEN);
	}
	nrf_gpio_pin_clear(LED_RGB_BLUE);
}


/**
 * Disbles the periodic ADC check for a given time in ms.
 * @param value, Determines how long the periodic should be disabled in ms.
 */
static void disable_perdioc_check(uint32_t value)
{
	/* Disable compare of cc[0] */
	NRF_RTC1->INTENCLR = 1<<16;

	/* Enable the pause */
	NRF_RTC1->CC[2] = value;
	NRF_RTC1->INTENSET = 1<<18;
	NRF_RTC1->TASKS_CLEAR = 1;
}


/**
 * This is the main program loop
 * @return
 */
int main(void)
{
	system_init();

	/* set initial state */
	sleep = true;
	stop_receiving = false;
	transmit_back = false;
	send_ack = false;

	/* start the RTC to check the Dout every 10ms with PPI */
	NRF_RTC1->TASKS_START = 1;

	/* Start the program */
	while(true) {

		if (sleep) {
			__WFI();
		} else {
			/* Disable the EVENT for checking the Dout */
			NRF_RTC1->INTENCLR = 1<<16;

			/* If the ADC has a low value it decides that it has to listin for a sync pulse */
			/* this can be a maximum of 111 ms, after that just sleep. */
			keep_listening = true;
			stop_receiving = false;

			/* Clear the timer and set interrupt for rx timeout */
			NRF_RTC1->INTENSET = 1<<17;
			NRF_RTC1->TASKS_CLEAR = 1;
			while(keep_listening) {
				printf("Receiving...\n");
				receive();

			}
			NRF_RTC1->INTENCLR = 1<<17;

			if(transmit_back) {

				/* sync RTC, do this only when a valid packet is received */
				NRF_RTC1->TASKS_CLEAR = 1;
				/* Compensate according to measurements */
				NRF_RTC1->CC[0]=4;

				/* added a small delay otherwise the charbee is to fast for the mobile node */
				nrf_delay_us(10);

				if (send_ack) {
					transmit(1);
				} else {
					transmit(0);
				}

				/* blink led to indicate that is is working */
				#ifdef LED_ON
				blink_blue(1);
				#endif

				printf("Transmit! %d %d\n", tpacket[0], tpacket[1]);

				/* becasue it has transmitted something back only after a second
				 * There can be a next round so we can totally go to sleep for quite
				 * some time */
				disable_perdioc_check(PAUSE_TIME);

			} else if (sleep) {

				/* sleep is set by the receive function then sleep and disable periodic check */
				printf("SLEEP COMMAND RECIEVED :)\n");
				nrf_gpio_pin_clear(LED_RGB_RED);
				disable_perdioc_check(DISABLE_CHECK_VALUE);

			} else {

				/* Enable the interrupt for checking the Dout again */
				disable_perdioc_check(DISABLE_CHECK_VALUE);
			}

			sleep = true;
			printf("sleep again...\n");


		}
	}

}

/* ---------------------------------------------------------------------------*/
/* IRQ Functions handlers*/
/* ---------------------------------------------------------------------------*/

/* This IRQ get called when there is a new ADC result. */
void ADC_IRQHandler(void)
{
	NRF_ADC->EVENTS_END = 0;
	nrf_gpio_pin_clear(2);
	//printf("%d\n", (int)NRF_ADC->RESULT);
	if(NRF_ADC->RESULT < ADC_TRESHOLD) {
		printf("WAKE UP!!\n");
		sleep = false;
		//sleep = true;
	} else {
		sleep = true;
	}
}

/* This interrupt gets called when the RTC matches the value in CC[0] */
void RTC1_IRQHandler(void)
{
	if (NRF_RTC1->EVENTS_COMPARE[0]) {
		NRF_RTC1->EVENTS_COMPARE[0]=0;
		nrf_gpio_pin_set(2);
		NRF_TIMER0->TASKS_START=1;
		NRF_RTC1->CC[0]=CHECK_PERIOD;
		NRF_RTC1->TASKS_CLEAR=1;

	} else if (NRF_RTC1->EVENTS_COMPARE[1]) {
		/* RX timeout */

		/* Clear the interrupt */
		NRF_RTC1->EVENTS_COMPARE[1] = 0;
		stop_receiving = true;
		//printf("RTC_IRQ[1] (RX timeout)\n");


	} else if (NRF_RTC1->EVENTS_COMPARE[2] == 1) {
		/* pause time */

		/* Clear the interrupt */
		NRF_RTC1->EVENTS_COMPARE[2] = 0;
		nrf_gpio_pin_set(LED_RGB_RED);

		/* enable compare of cc[0] */
		NRF_RTC1->INTENSET = 1<<16;

		/* disable the pause */
		NRF_RTC1->INTENCLR = 1<<18;
		NRF_RTC1->TASKS_CLEAR = 1;
		printf("Enable periodic check again!\n");
	}

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
	//printf("r\n");

}
