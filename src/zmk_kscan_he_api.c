/*
 * SPDX-License-Identifier: MIT
 *
 * Local API shim — accesses the kscan-adc-mux driver internals to provide
 * per-key threshold management, ADC readout, and calibration data.
 *
 * The generic structs below mirror the layout of the macro-generated
 * he_kscan_cfg_##n / he_kscan_data_##n in the upstream driver.  They must
 * stay in sync with the driver's KSCAN_ADC_MUX_INST() macro.
 */

#include "zmk_kscan_he_api.h"
#include <he_key_state.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zmk_kscan_he_api, CONFIG_ZMK_LOG_LEVEL);

/* -------------------------------------------------------------------
 * Generic config struct — matches the beginning of he_kscan_cfg_##n.
 * We only read fields up to release_threshold.
 * ------------------------------------------------------------------- */
struct he_kscan_cfg_generic {
    const struct gpio_dt_spec *select_gpios;
    const struct adc_dt_spec  *adc_channels;
    const uint8_t             *mux_map;
    uint8_t  num_select;
    uint8_t  num_adc;
    uint8_t  num_addr;
    uint8_t  num_keys;
    uint16_t scan_period_ms;
    uint16_t idle_scan_period_ms;
    uint16_t idle_enter_ms;
    uint8_t  idle_wakeup_delta;
    uint16_t settle_us;
    bool     invert_adc;
    uint8_t  press_threshold;
    uint8_t  release_threshold;
};

/* -------------------------------------------------------------------
 * Generic data struct — matches the beginning of he_kscan_data_##n.
 * keys[] is a flexible array member; actual length = cfg->num_keys.
 * ------------------------------------------------------------------- */
struct he_kscan_data_generic {
    const struct device        *dev;
    kscan_callback_t            callback;
    struct k_work_delayable     scan_work;
    struct he_key_state         keys[];
};

/* -------------------------------------------------------------------
 * Per-key threshold overrides
 *
 * The upstream driver only stores a single global press/release
 * threshold pair.  We maintain per-key overrides here so that the
 * configurator protocol can get/set individual keys.
 * ------------------------------------------------------------------- */
#define MAX_KEYS 32

struct key_threshold {
    uint8_t press;
    uint8_t release;
};

static struct key_threshold key_thresholds[MAX_KEYS];
static bool thresholds_initialized;

static void ensure_thresholds(const struct he_kscan_cfg_generic *cfg)
{
    if (thresholds_initialized) {
        return;
    }
    for (uint8_t i = 0; i < cfg->num_keys && i < MAX_KEYS; i++) {
        key_thresholds[i].press   = cfg->press_threshold;
        key_thresholds[i].release = cfg->release_threshold;
    }
    thresholds_initialized = true;
}

/* -------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------- */

int zmk_kscan_he_get_num_keys(const struct device *dev, uint8_t *num_keys)
{
    if (!dev || !num_keys) {
        return -EINVAL;
    }
    const struct he_kscan_cfg_generic *cfg = dev->config;
    *num_keys = cfg->num_keys;
    return 0;
}

int zmk_kscan_he_get_threshold(const struct device *dev, uint8_t key_idx,
                                uint8_t *press, uint8_t *release)
{
    if (!dev || !press || !release) {
        return -EINVAL;
    }
    const struct he_kscan_cfg_generic *cfg = dev->config;
    if (key_idx >= cfg->num_keys || key_idx >= MAX_KEYS) {
        return -EINVAL;
    }
    ensure_thresholds(cfg);
    *press   = key_thresholds[key_idx].press;
    *release = key_thresholds[key_idx].release;
    return 0;
}

int zmk_kscan_he_set_threshold(const struct device *dev, uint8_t key_idx,
                                uint8_t press, uint8_t release)
{
    if (!dev) {
        return -EINVAL;
    }
    const struct he_kscan_cfg_generic *cfg = dev->config;
    if (key_idx >= cfg->num_keys || key_idx >= MAX_KEYS) {
        return -EINVAL;
    }
    ensure_thresholds(cfg);
    key_thresholds[key_idx].press   = press;
    key_thresholds[key_idx].release = release;
    return 0;
}

int zmk_kscan_he_get_adc_raw(const struct device *dev, uint8_t key_idx,
                              uint16_t *adc, uint8_t *dist)
{
    if (!dev || !adc || !dist) {
        return -EINVAL;
    }
    const struct he_kscan_cfg_generic *cfg = dev->config;
    if (key_idx >= cfg->num_keys) {
        return -EINVAL;
    }
    const struct he_kscan_data_generic *data = dev->data;
    *adc  = data->keys[key_idx].adc_filtered;
    *dist = data->keys[key_idx].distance;
    return 0;
}

int zmk_kscan_he_save_settings(const struct device *dev)
{
    /* Per-key thresholds are not yet persisted to flash.
     * A future version will use the Zephyr settings subsystem. */
    LOG_INF("save_settings: stub (not yet persisted)");
    return 0;
}

int zmk_kscan_he_reset_defaults(const struct device *dev)
{
    if (!dev) {
        return -EINVAL;
    }
    const struct he_kscan_cfg_generic *cfg = dev->config;
    for (uint8_t i = 0; i < cfg->num_keys && i < MAX_KEYS; i++) {
        key_thresholds[i].press   = cfg->press_threshold;
        key_thresholds[i].release = cfg->release_threshold;
    }
    LOG_INF("thresholds reset to defaults (press=%u release=%u)",
            cfg->press_threshold, cfg->release_threshold);
    return 0;
}

int zmk_kscan_he_get_calibration(const struct device *dev, uint8_t key_idx,
                                  uint16_t *rest, uint16_t *bottom)
{
    if (!dev || !rest || !bottom) {
        return -EINVAL;
    }
    const struct he_kscan_cfg_generic *cfg = dev->config;
    if (key_idx >= cfg->num_keys) {
        return -EINVAL;
    }
    const struct he_kscan_data_generic *data = dev->data;
    *rest   = data->keys[key_idx].adc_rest;
    *bottom = data->keys[key_idx].adc_bottom;
    return 0;
}

int zmk_kscan_he_recalibrate(const struct device *dev)
{
    /* Cannot invoke the instance-specific he_calibrate_##n() from here.
     * A full recalibration would require PM suspend/resume or a
     * driver-level API extension. */
    LOG_INF("recalibrate: stub (restart device to recalibrate)");
    return 0;
}
