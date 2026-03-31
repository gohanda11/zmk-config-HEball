/*
 * SPDX-License-Identifier: MIT
 * HEball BLE GATT central — right-half client that proxies commands to left half.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Check whether the BLE link to the left-half HEBALL service is ready.
 */
bool heball_ble_central_is_connected(void);

/**
 * Write a command to the left half via BLE.
 * The payload is the raw command body: [CMD_ID][payload...] (no framing).
 * @return 0 on success, negative errno on failure
 */
int heball_ble_central_write_cmd(const uint8_t *data, uint16_t len);

/**
 * Retrieve the last response received from the left half.
 * Copies up to @p max_len bytes into @p out_buf.
 * @return number of bytes copied, or -EAGAIN if no response pending
 */
int heball_ble_central_get_response(uint8_t *out_buf, uint16_t max_len);

/**
 * Check if a response from the left half is available.
 */
bool heball_ble_central_response_ready(void);

/**
 * Clear the pending response slot.
 */
void heball_ble_central_clear_response(void);

/**
 * Register a callback for ADC streaming data received from the left half.
 * The callback is invoked from the BLE RX context — keep it short.
 */
typedef void (*heball_ble_adc_cb_t)(const uint8_t *data, uint16_t len);
void heball_ble_central_set_adc_callback(heball_ble_adc_cb_t cb);
