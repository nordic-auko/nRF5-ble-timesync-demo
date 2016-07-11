/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

#ifndef ADAFRUIT_PN532_CONFIG_H
#define ADAFRUIT_PN532_CONFIG_H

#include "boards.h"
 /**
 * @defgroup nrf_external_adafruit_pn532_host_config Host configuration
 * @{
 * @ingroup nrf_external_adafruit_pn532
 * @brief These defines should be specified to configure pins for the appropriate board type
 *        and choose a TWI instance.
 */

 /**
 * @brief Identifier of the TWI instance.
 */
#define MASTER_TWI_INST         0

 /**
 * @name Macros for Arduino pin mappings
 * @{
 *
 */
#define PN532_IRQ            (ARDUINO_2_PIN)
#define PN532_RESET          (ARDUINO_3_PIN)
 /* @} */

 /* @} */

#endif //ADAFRUIT_PN532_CONFIG_H
