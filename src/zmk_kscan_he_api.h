/*
 * SPDX-License-Identifier: MIT
 *
 * Local API shim for the zmk-driver-kscan-he module.
 *
 * The upstream driver (zmk,kscan-adc-mux) only exposes the standard Zephyr
 * kscan_driver_api.  This header declares higher-level helpers that access
 * the driver's internal data structures (he_key_state, config) to provide
 * per-key threshold get/set, ADC readout, and calibration information for
 * the HEball configurator protocol.
 */
#pragma once

#include <zephyr/device.h>
#include <stdint.h>

int zmk_kscan_he_get_num_keys(const struct device *dev, uint8_t *num_keys);

int zmk_kscan_he_get_threshold(const struct device *dev, uint8_t key_idx,
                                uint8_t *press, uint8_t *release);

int zmk_kscan_he_set_threshold(const struct device *dev, uint8_t key_idx,
                                uint8_t press, uint8_t release);

int zmk_kscan_he_get_adc_raw(const struct device *dev, uint8_t key_idx,
                              uint16_t *adc, uint8_t *dist);

int zmk_kscan_he_save_settings(const struct device *dev);

int zmk_kscan_he_reset_defaults(const struct device *dev);

int zmk_kscan_he_get_calibration(const struct device *dev, uint8_t key_idx,
                                  uint16_t *rest, uint16_t *bottom);

int zmk_kscan_he_recalibrate(const struct device *dev);
