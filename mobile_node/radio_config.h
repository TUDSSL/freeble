/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#include <stdint.h>
#include "nrf.h"

#define PACKET_BASE_ADDRESS_LENGTH       (4UL)  //!< Packet base address length field size in bytes
#define PACKET_STATIC_LENGTH             (30UL)  //!< Packet static length in bytes
#define PACKET_PAYLOAD_MAXSIZE           (PACKET_STATIC_LENGTH)  //!< Packet payload maximum size in bytes

/* These are set to zero as Shockburst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE             (0UL)  /**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE             (0UL)  /**< Packet S0 field size in bits. */
#define PACKET_LENGTH_FIELD_SIZE         (0UL)  /**< Packet length field size in bits. */

#define SET_BIT(n)      (1UL << n)
#define MAX_PDU_SIZE    (64UL)

void radio_configure(void);

#endif
