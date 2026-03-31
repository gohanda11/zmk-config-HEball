/*
 * SPDX-License-Identifier: MIT
 * HEball BLE GATT peripheral — left-half custom service implementation.
 *
 * Exposes three characteristics:
 *   1. Threshold Write  — central writes commands here (same binary format as USB)
 *   2. Threshold Read   — peripheral notifies responses back to central
 *   3. ADC Notify       — peripheral streams ADC data at ≤10 Hz
 */

#include "heball_ble_peripheral.h"
#include "cmd_handler.h"
#include <zmk_kscan_he_api.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <string.h>

LOG_MODULE_REGISTER(heball_ble_periph, CONFIG_ZMK_LOG_LEVEL);

/* -----------------------------------------------------------------------
 * Notification state
 * ----------------------------------------------------------------------- */
static bool thresh_read_notify_enabled;
static bool adc_notify_enabled;
static struct bt_conn *current_conn;

/* ADC streaming state */
static bool adc_streaming_active;
static struct k_work_delayable adc_stream_work;
#define ADC_STREAM_INTERVAL_MS 100  /* 10 Hz max */

/* Forward declaration of the service for bt_gatt_notify */
static struct bt_gatt_service heball_svc;

/* -----------------------------------------------------------------------
 * Threshold Write callback — receives commands from central
 * ----------------------------------------------------------------------- */
static ssize_t heball_thresh_write_cb(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len,
                                      uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = buf;
    uint8_t cmd_id = data[0];
    const uint8_t *payload = (len > 1) ? &data[1] : NULL;
    uint8_t payload_len = (uint8_t)(len - 1);

    const struct device *kscan_dev = DEVICE_DT_GET(DT_NODELABEL(kscan_he));
    if (!device_is_ready(kscan_dev)) {
        uint8_t resp[2] = { cmd_id, HEBALL_STATUS_ERROR };
        heball_ble_peripheral_send_response(resp, 2);
        return len;
    }

    uint8_t num_keys = 0;
    zmk_kscan_he_get_num_keys(kscan_dev, &num_keys);
    if (num_keys > HEBALL_MAX_KEYS) {
        num_keys = HEBALL_MAX_KEYS;
    }

    /* Response buffer: CMD_ID + STATUS + payload (max ~128 bytes) */
    uint8_t resp[2 + HEBALL_MAX_KEYS * 4];
    uint16_t resp_len = 2;
    resp[0] = cmd_id;
    resp[1] = HEBALL_STATUS_OK;

    switch (cmd_id) {

    case CMD_GET_VERSION: {
        resp[2] = HEBALL_PROTOCOL_VERSION;
        resp[3] = HEBALL_HALF_LEFT;
        resp_len = 4;
        break;
    }

    case CMD_GET_NUM_KEYS: {
        resp[2] = num_keys;
        resp_len = 3;
        break;
    }

    case CMD_GET_THRESHOLDS: {
        for (uint8_t k = 0; k < num_keys; k++) {
            uint8_t p = 0, r = 0;
            zmk_kscan_he_get_threshold(kscan_dev, k, &p, &r);
            resp[2 + k * 2]     = p;
            resp[2 + k * 2 + 1] = r;
        }
        resp_len = 2 + num_keys * 2;
        break;
    }

    case CMD_SET_THRESHOLD: {
        if (payload_len < 3) {
            resp[1] = HEBALL_STATUS_ERROR;
            break;
        }
        /* payload[0]=key_idx (already local), payload[1]=press, payload[2]=release */
        uint8_t key_idx = payload[0];
        uint8_t press   = payload[1];
        uint8_t release = payload[2];
        int ret = zmk_kscan_he_set_threshold(kscan_dev, key_idx, press, release);
        if (ret < 0) {
            resp[1] = HEBALL_STATUS_ERROR;
        }
        break;
    }

    case CMD_SET_THRESHOLDS_BULK: {
        if (payload_len < 2) {
            resp[1] = HEBALL_STATUS_ERROR;
            break;
        }
        uint8_t start = payload[0];
        uint8_t count = payload[1];
        if (payload_len < 2 + count * 2) {
            resp[1] = HEBALL_STATUS_ERROR;
            break;
        }
        for (uint8_t i = 0; i < count; i++) {
            uint8_t press   = payload[2 + i * 2];
            uint8_t release = payload[2 + i * 2 + 1];
            int ret = zmk_kscan_he_set_threshold(kscan_dev, start + i, press, release);
            if (ret < 0) {
                resp[1] = HEBALL_STATUS_ERROR;
                break;
            }
        }
        break;
    }

    case CMD_GET_ADC_VALUES: {
        for (uint8_t k = 0; k < num_keys; k++) {
            uint16_t adc = 0;
            uint8_t  dist = 0;
            zmk_kscan_he_get_adc_raw(kscan_dev, k, &adc, &dist);
            resp[2 + k * 3]     = (uint8_t)(adc & 0xFF);
            resp[2 + k * 3 + 1] = (uint8_t)(adc >> 8);
            resp[2 + k * 3 + 2] = dist;
        }
        resp_len = 2 + num_keys * 3;
        break;
    }

    case CMD_SAVE_SETTINGS: {
        int ret = zmk_kscan_he_save_settings(kscan_dev);
        if (ret < 0) {
            resp[1] = HEBALL_STATUS_ERROR;
        }
        break;
    }

    case CMD_RESET_DEFAULTS: {
        int ret = zmk_kscan_he_reset_defaults(kscan_dev);
        if (ret < 0) {
            resp[1] = HEBALL_STATUS_ERROR;
        }
        break;
    }

    case CMD_GET_CALIBRATION: {
        for (uint8_t k = 0; k < num_keys; k++) {
            uint16_t rest = 0, bottom = 0;
            zmk_kscan_he_get_calibration(kscan_dev, k, &rest, &bottom);
            resp[2 + k * 4]     = (uint8_t)(rest & 0xFF);
            resp[2 + k * 4 + 1] = (uint8_t)(rest >> 8);
            resp[2 + k * 4 + 2] = (uint8_t)(bottom & 0xFF);
            resp[2 + k * 4 + 3] = (uint8_t)(bottom >> 8);
        }
        resp_len = 2 + num_keys * 4;
        break;
    }

    case CMD_RECALIBRATE: {
        kscan_disable_callback(kscan_dev);
        zmk_kscan_he_recalibrate(kscan_dev);
        kscan_enable_callback(kscan_dev);
        break;
    }

    case CMD_STREAM_ADC_START: {
        adc_streaming_active = true;
        k_work_reschedule(&adc_stream_work, K_MSEC(ADC_STREAM_INTERVAL_MS));
        break;
    }

    case CMD_STREAM_ADC_STOP: {
        adc_streaming_active = false;
        k_work_cancel_delayable(&adc_stream_work);
        break;
    }

    default:
        resp[1] = HEBALL_STATUS_ERROR;
        break;
    }

    heball_ble_peripheral_send_response(resp, resp_len);
    return len;
}

/* -----------------------------------------------------------------------
 * Threshold Read callback (for explicit READ requests)
 * ----------------------------------------------------------------------- */
static ssize_t heball_thresh_read_cb(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len,
                                     uint16_t offset)
{
    /* Return protocol version + half identifier on read */
    uint8_t data[2] = { HEBALL_PROTOCOL_VERSION, HEBALL_HALF_LEFT };
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(data));
}

/* -----------------------------------------------------------------------
 * CCC callbacks
 * ----------------------------------------------------------------------- */
static void heball_thresh_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    thresh_read_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Thresh-read notifications %s",
            thresh_read_notify_enabled ? "enabled" : "disabled");
}

static void heball_adc_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    adc_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("ADC notifications %s",
            adc_notify_enabled ? "enabled" : "disabled");

    if (!adc_notify_enabled && adc_streaming_active) {
        adc_streaming_active = false;
        k_work_cancel_delayable(&adc_stream_work);
    }
}

/* -----------------------------------------------------------------------
 * GATT Service Definition
 * ----------------------------------------------------------------------- */
BT_GATT_SERVICE_DEFINE(heball_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(HEBALL_SVC_UUID_VAL)),

    /* Threshold Write characteristic */
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(HEBALL_THRESH_WRITE_UUID_VAL),
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE_ENCRYPT,
        NULL, heball_thresh_write_cb, NULL),

    /* Threshold Read characteristic (READ + NOTIFY for responses) */
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(HEBALL_THRESH_READ_UUID_VAL),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        heball_thresh_read_cb, NULL, NULL),
    BT_GATT_CCC(heball_thresh_ccc_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),

    /* ADC Notify characteristic */
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(HEBALL_ADC_NOTIFY_UUID_VAL),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(heball_adc_ccc_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),
);

/* -----------------------------------------------------------------------
 * Notification send helpers
 * ----------------------------------------------------------------------- */
int heball_ble_peripheral_send_response(const uint8_t *data, uint16_t len)
{
    if (!thresh_read_notify_enabled || !current_conn) {
        return -ENOTCONN;
    }

    /* Attribute index 4 = thresh_read value attribute (after svc + write char/val + read char) */
    const struct bt_gatt_attr *attr = &heball_svc.attrs[4];
    return bt_gatt_notify(current_conn, attr, data, len);
}

int heball_ble_peripheral_send_adc(const uint8_t *data, uint16_t len)
{
    if (!adc_notify_enabled || !current_conn) {
        return -ENOTCONN;
    }

    /* Attribute index 7 = adc_notify value attribute */
    const struct bt_gatt_attr *attr = &heball_svc.attrs[7];
    return bt_gatt_notify(current_conn, attr, data, len);
}

/* -----------------------------------------------------------------------
 * ADC stream work handler (≤10 Hz)
 * ----------------------------------------------------------------------- */
static void adc_stream_work_handler(struct k_work *work)
{
    if (!adc_streaming_active || !adc_notify_enabled) {
        return;
    }

    const struct device *kscan_dev = DEVICE_DT_GET(DT_NODELABEL(kscan_he));
    if (!device_is_ready(kscan_dev)) {
        goto reschedule;
    }

    uint8_t num_keys = 0;
    zmk_kscan_he_get_num_keys(kscan_dev, &num_keys);
    if (num_keys > HEBALL_MAX_KEYS) {
        num_keys = HEBALL_MAX_KEYS;
    }

    /* Payload: [CMD_STREAM_ADC_DATA] [key_idx, adc_lo, adc_hi, dist] × N */
    uint8_t buf[1 + HEBALL_MAX_KEYS * 4];
    buf[0] = CMD_STREAM_ADC_DATA;
    for (uint8_t k = 0; k < num_keys; k++) {
        uint16_t adc = 0;
        uint8_t  dist = 0;
        zmk_kscan_he_get_adc_raw(kscan_dev, k, &adc, &dist);
        buf[1 + k * 4]     = k;
        buf[1 + k * 4 + 1] = (uint8_t)(adc & 0xFF);
        buf[1 + k * 4 + 2] = (uint8_t)(adc >> 8);
        buf[1 + k * 4 + 3] = dist;
    }

    heball_ble_peripheral_send_adc(buf, 1 + num_keys * 4);

reschedule:
    k_work_reschedule(&adc_stream_work, K_MSEC(ADC_STREAM_INTERVAL_MS));
}

/* -----------------------------------------------------------------------
 * Connection tracking
 * ----------------------------------------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_WRN("BLE connection failed (err %u)", err);
        return;
    }
    LOG_INF("BLE peer connected");
    current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE peer disconnected (reason %u)", reason);

    if (adc_streaming_active) {
        adc_streaming_active = false;
        k_work_cancel_delayable(&adc_stream_work);
    }
    thresh_read_notify_enabled = false;
    adc_notify_enabled = false;

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(heball_periph_conn_cb) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* -----------------------------------------------------------------------
 * Initialization
 * ----------------------------------------------------------------------- */
static int heball_ble_peripheral_init(void)
{
    k_work_init_delayable(&adc_stream_work, adc_stream_work_handler);
    LOG_INF("HEball BLE peripheral service registered");
    return 0;
}

SYS_INIT(heball_ble_peripheral_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY + 2);
