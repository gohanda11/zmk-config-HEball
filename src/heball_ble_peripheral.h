/*
 * SPDX-License-Identifier: MIT
 * HEball BLE GATT peripheral — custom service for left-half threshold config.
 */
#pragma once

#include <zephyr/bluetooth/uuid.h>

/* Custom GATT Service UUID: 12345678-1234-5678-1234-56789abcdef0 */
#define HEBALL_SVC_UUID_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

/* Threshold Write characteristic: ..def1 (WRITE, WRITE_WITHOUT_RESPONSE) */
#define HEBALL_THRESH_WRITE_UUID_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

/* Threshold Read characteristic: ..def2 (READ, NOTIFY — responses back to central) */
#define HEBALL_THRESH_READ_UUID_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

/* ADC Notify characteristic: ..def3 (NOTIFY — streaming ADC data) */
#define HEBALL_ADC_NOTIFY_UUID_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)

/**
 * Send a response (or unsolicited notification) on the thresh-read characteristic.
 * @param data  payload bytes (CMD_ID + STATUS + optional payload)
 * @param len   number of bytes
 * @return 0 on success, negative errno on failure
 */
int heball_ble_peripheral_send_response(const uint8_t *data, uint16_t len);

/**
 * Send ADC streaming data on the adc-notify characteristic.
 * @param data  ADC payload bytes
 * @param len   number of bytes
 * @return 0 on success, negative errno on failure
 */
int heball_ble_peripheral_send_adc(const uint8_t *data, uint16_t len);
