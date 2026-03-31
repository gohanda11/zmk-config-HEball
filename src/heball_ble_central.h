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
 * Send a command to the left half and wait for a matching response.
 * Blocks the calling thread for up to @p timeout_ms milliseconds.
 * Must NOT be called from the system workqueue -- use a dedicated thread.
 * @param cmd        raw command bytes [CMD_ID][payload...]
 * @param cmd_len    length of cmd
 * @param resp       buffer for response
 * @param resp_max   max bytes to copy into resp
 * @param timeout_ms timeout in milliseconds
 * @return number of response bytes copied, or negative errno
 */
int heball_ble_central_send_and_wait(const uint8_t *cmd, uint16_t cmd_len,
                                      uint8_t *resp, uint16_t resp_max,
                                      int timeout_ms);

/**
 * Register a callback for ADC streaming data received from the left half.
 * The callback is invoked from the BLE RX context — keep it short.
 */
typedef void (*heball_ble_adc_cb_t)(const uint8_t *data, uint16_t len);
void heball_ble_central_set_adc_callback(heball_ble_adc_cb_t cb);
