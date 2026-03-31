/*
 * SPDX-License-Identifier: MIT
 * HEball BLE GATT central -- right-half client implementation.
 *
 * Discovers the custom HEBALL GATT service on the left half (BLE peripheral),
 * subscribes to thresh-read and ADC notifications, and provides an API for
 * cmd_handler.c to proxy commands over BLE.
 */

#include "heball_ble_central.h"
#include "heball_ble_peripheral.h"   /* UUID definitions shared with peripheral */
#include "cmd_handler.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <string.h>

LOG_MODULE_REGISTER(heball_ble_central, CONFIG_ZMK_LOG_LEVEL);

/* -----------------------------------------------------------------------
 * Static UUIDs for discovery
 * ----------------------------------------------------------------------- */
static struct bt_uuid_128 heball_svc_uuid =
    BT_UUID_INIT_128(HEBALL_SVC_UUID_VAL);
static struct bt_uuid_128 thresh_write_uuid =
    BT_UUID_INIT_128(HEBALL_THRESH_WRITE_UUID_VAL);
static struct bt_uuid_128 thresh_read_uuid =
    BT_UUID_INIT_128(HEBALL_THRESH_READ_UUID_VAL);
static struct bt_uuid_128 adc_notify_uuid =
    BT_UUID_INIT_128(HEBALL_ADC_NOTIFY_UUID_VAL);

/* -----------------------------------------------------------------------
 * Connection & discovery state
 * ----------------------------------------------------------------------- */
static struct bt_conn *split_conn;         /* BLE connection to the left half */
static bool service_discovered;            /* true after full GATT discovery  */

static uint16_t thresh_write_handle;       /* ATT handle for write char */
static uint16_t thresh_read_handle;        /* ATT handle for read/notify char value */
static uint16_t adc_notify_handle;         /* ATT handle for ADC notify char value */

/* CCC handles discovered via descriptor discovery (Bug 3 fix) */
static uint16_t thresh_read_ccc_handle;
static uint16_t adc_notify_ccc_handle;

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params thresh_read_sub;
static struct bt_gatt_subscribe_params adc_notify_sub;

/* Response synchronization (Bug 4 fix) */
static K_MUTEX_DEFINE(ble_resp_mutex);
static K_SEM_DEFINE(ble_resp_sem, 0, 1);
static uint8_t ble_pending_cmd_id;
#define RESPONSE_BUF_SIZE 256
static uint8_t ble_response_buf[RESPONSE_BUF_SIZE];
static uint16_t ble_response_len;

/* ADC callback */
static heball_ble_adc_cb_t adc_callback;

/* Discovery phase tracking */
typedef enum {
    DISC_IDLE,
    DISC_HEBALL_SERVICE,  /* searching for our custom service */
    DISC_CHARACTERISTICS, /* enumerating characteristics */
    DISC_DESC_THRESH,     /* descriptor discovery for thresh-read CCC */
    DISC_DESC_ADC,        /* descriptor discovery for ADC CCC */
    DISC_SUBSCRIBE_THRESH,
    DISC_SUBSCRIBE_ADC,
    DISC_COMPLETE,
} disc_phase_t;

static disc_phase_t disc_phase = DISC_IDLE;

/* Discovery retry for -EBUSY (Bug 2 fix) */
#define MAX_DISCOVERY_RETRIES 5
#define DISCOVERY_RETRY_DELAY_MS 200
static uint8_t discovery_retry_count;

/* Delayed work for starting discovery after connection settles */
static struct k_work_delayable discovery_start_work;

static uint16_t svc_start_handle;
static uint16_t svc_end_handle;

/* Forward declarations */
static void start_heball_service_discovery(void);
static void start_characteristic_discovery(uint16_t start_handle, uint16_t end_handle);
static void start_desc_discovery_thresh(void);
static void start_desc_discovery_adc(void);
static void subscribe_thresh_read(void);
static void subscribe_adc_notify(void);

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */
bool heball_ble_central_is_connected(void)
{
    return split_conn != NULL && service_discovered;
}

int heball_ble_central_send_and_wait(const uint8_t *cmd, uint16_t cmd_len,
                                      uint8_t *resp, uint16_t resp_max,
                                      int timeout_ms)
{
    if (!split_conn || !service_discovered || thresh_write_handle == 0) {
        return -ENOTCONN;
    }
    if (cmd_len < 1) {
        return -EINVAL;
    }

    k_mutex_lock(&ble_resp_mutex, K_FOREVER);
    ble_pending_cmd_id = cmd[0];
    k_sem_reset(&ble_resp_sem);
    k_mutex_unlock(&ble_resp_mutex);

    int err = bt_gatt_write_without_response(split_conn, thresh_write_handle,
                                              cmd, cmd_len, false);
    if (err) {
        k_mutex_lock(&ble_resp_mutex, K_FOREVER);
        ble_pending_cmd_id = 0;
        k_mutex_unlock(&ble_resp_mutex);
        return err;
    }

    err = k_sem_take(&ble_resp_sem, K_MSEC(timeout_ms));

    k_mutex_lock(&ble_resp_mutex, K_FOREVER);
    if (err) {
        ble_pending_cmd_id = 0;
        k_mutex_unlock(&ble_resp_mutex);
        return -ETIMEDOUT;
    }

    uint16_t copy_len = (ble_response_len < resp_max) ? ble_response_len : resp_max;
    memcpy(resp, ble_response_buf, copy_len);
    ble_pending_cmd_id = 0;
    k_mutex_unlock(&ble_resp_mutex);

    return (int)copy_len;
}

void heball_ble_central_set_adc_callback(heball_ble_adc_cb_t cb)
{
    adc_callback = cb;
}

/* -----------------------------------------------------------------------
 * Notification callbacks
 * ----------------------------------------------------------------------- */
static uint8_t thresh_read_notify_cb(struct bt_conn *conn,
                                     struct bt_gatt_subscribe_params *params,
                                     const void *data, uint16_t length)
{
    if (!data) {
        LOG_INF("Thresh-read notifications unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    const uint8_t *bytes = data;

    /* Bug 4 fix: match CMD_ID and signal semaphore under mutex */
    k_mutex_lock(&ble_resp_mutex, K_FOREVER);
    if (length > 0 && length <= RESPONSE_BUF_SIZE &&
        ble_pending_cmd_id != 0 && bytes[0] == ble_pending_cmd_id) {
        memcpy(ble_response_buf, data, length);
        ble_response_len = length;
        k_sem_give(&ble_resp_sem);
    }
    k_mutex_unlock(&ble_resp_mutex);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t adc_notify_cb(struct bt_conn *conn,
                               struct bt_gatt_subscribe_params *params,
                               const void *data, uint16_t length)
{
    if (!data) {
        LOG_INF("ADC notifications unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (adc_callback && length > 0) {
        adc_callback(data, length);
    }

    return BT_GATT_ITER_CONTINUE;
}

/* -----------------------------------------------------------------------
 * GATT discovery state machine
 * ----------------------------------------------------------------------- */

/* Helper for discovery errors with -EBUSY retry (Bug 2 fix) */
static void handle_discovery_error(int err, const char *phase_name)
{
    if (err == -EBUSY && discovery_retry_count < MAX_DISCOVERY_RETRIES) {
        discovery_retry_count++;
        LOG_INF("%s: EBUSY, retry %u/%u", phase_name,
                discovery_retry_count, MAX_DISCOVERY_RETRIES);
        k_work_reschedule(&discovery_start_work, K_MSEC(DISCOVERY_RETRY_DELAY_MS));
    } else {
        LOG_ERR("%s failed (err %d)", phase_name, err);
        disc_phase = DISC_IDLE;
    }
}

/* HEBALL service discovery callback */
static uint8_t heball_svc_discover_cb(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_WRN("HEBALL custom service not found on peer -- not our split half");
        disc_phase = DISC_IDLE;
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_service_val *svc_val = attr->user_data;
    svc_start_handle = attr->handle + 1;
    svc_end_handle = svc_val->end_handle;

    LOG_INF("HEBALL service found [%u-%u]", svc_start_handle, svc_end_handle);
    start_characteristic_discovery(svc_start_handle, svc_end_handle);
    return BT_GATT_ITER_STOP;
}

/* Characteristic discovery callback */
static uint8_t char_discover_cb(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("Characteristic discovery complete: write=%u read=%u adc=%u",
                thresh_write_handle, thresh_read_handle, adc_notify_handle);

        /* Bug 3 fix: discover CCC descriptors instead of assuming handle+1 */
        if (thresh_read_handle != 0) {
            start_desc_discovery_thresh();
        } else if (adc_notify_handle != 0) {
            start_desc_discovery_adc();
        } else {
            disc_phase = DISC_COMPLETE;
            service_discovered = true;
            LOG_INF("HEBALL BLE discovery complete (no notify chars)");
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;

    if (!bt_uuid_cmp(chrc->uuid, &thresh_write_uuid.uuid)) {
        thresh_write_handle = chrc->value_handle;
        LOG_DBG("Found thresh-write handle: %u", thresh_write_handle);
    } else if (!bt_uuid_cmp(chrc->uuid, &thresh_read_uuid.uuid)) {
        thresh_read_handle = chrc->value_handle;
        LOG_DBG("Found thresh-read handle: %u", thresh_read_handle);
    } else if (!bt_uuid_cmp(chrc->uuid, &adc_notify_uuid.uuid)) {
        adc_notify_handle = chrc->value_handle;
        LOG_DBG("Found adc-notify handle: %u", adc_notify_handle);
    }

    return BT_GATT_ITER_CONTINUE;
}

/* Descriptor discovery callback for finding CCC handles (Bug 3 fix) */
static uint8_t desc_discover_cb(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    if (!attr) {
        if (disc_phase == DISC_DESC_THRESH) {
            if (thresh_read_ccc_handle == 0) {
                LOG_WRN("CCC not found for thresh-read, falling back to handle+1");
                thresh_read_ccc_handle = thresh_read_handle + 1;
            }
            if (adc_notify_handle != 0) {
                start_desc_discovery_adc();
            } else {
                subscribe_thresh_read();
            }
        } else if (disc_phase == DISC_DESC_ADC) {
            if (adc_notify_ccc_handle == 0) {
                LOG_WRN("CCC not found for adc-notify, falling back to handle+1");
                adc_notify_ccc_handle = adc_notify_handle + 1;
            }
            if (thresh_read_handle != 0) {
                subscribe_thresh_read();
            } else {
                subscribe_adc_notify();
            }
        }
        return BT_GATT_ITER_STOP;
    }

    if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC)) {
        if (disc_phase == DISC_DESC_THRESH) {
            thresh_read_ccc_handle = attr->handle;
            LOG_DBG("Found thresh-read CCC handle: %u", thresh_read_ccc_handle);
        } else if (disc_phase == DISC_DESC_ADC) {
            adc_notify_ccc_handle = attr->handle;
            LOG_DBG("Found adc-notify CCC handle: %u", adc_notify_ccc_handle);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

/* Bug 2 fix: discover HEBALL service directly (skip ZMK service check) */
static void start_heball_service_discovery(void)
{
    if (!split_conn) {
        return;
    }

    disc_phase = DISC_HEBALL_SERVICE;
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = &heball_svc_uuid.uuid;
    discover_params.func = heball_svc_discover_cb;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(split_conn, &discover_params);
    if (err) {
        handle_discovery_error(err, "HEBALL service discovery");
    }
}

static void start_characteristic_discovery(uint16_t start_handle, uint16_t end_handle)
{
    disc_phase = DISC_CHARACTERISTICS;
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = NULL;  /* discover all characteristics */
    discover_params.func = char_discover_cb;
    discover_params.start_handle = start_handle;
    discover_params.end_handle = end_handle;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(split_conn, &discover_params);
    if (err) {
        handle_discovery_error(err, "Characteristic discovery");
    }
}

/* Bug 3 fix: descriptor discovery to find actual CCC handles */
static void start_desc_discovery_thresh(void)
{
    disc_phase = DISC_DESC_THRESH;
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = NULL;
    discover_params.func = desc_discover_cb;
    discover_params.start_handle = thresh_read_handle + 1;
    /* Search up to next char declaration or end of service */
    discover_params.end_handle = (adc_notify_handle > 0)
        ? (adc_notify_handle - 2) : svc_end_handle;
    discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

    int err = bt_gatt_discover(split_conn, &discover_params);
    if (err) {
        handle_discovery_error(err, "Thresh-read descriptor discovery");
    }
}

static void start_desc_discovery_adc(void)
{
    disc_phase = DISC_DESC_ADC;
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = NULL;
    discover_params.func = desc_discover_cb;
    discover_params.start_handle = adc_notify_handle + 1;
    discover_params.end_handle = svc_end_handle;
    discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

    int err = bt_gatt_discover(split_conn, &discover_params);
    if (err) {
        handle_discovery_error(err, "ADC descriptor discovery");
    }
}

static void subscribe_thresh_read(void)
{
    disc_phase = DISC_SUBSCRIBE_THRESH;

    thresh_read_sub.notify = thresh_read_notify_cb;
    thresh_read_sub.value_handle = thresh_read_handle;
    thresh_read_sub.ccc_handle = thresh_read_ccc_handle;
    thresh_read_sub.end_handle = svc_end_handle;
    thresh_read_sub.value = BT_GATT_CCC_NOTIFY;

    int err = bt_gatt_subscribe(split_conn, &thresh_read_sub);
    if (err && err != -EALREADY) {
        LOG_ERR("Thresh-read subscribe failed (err %d)", err);
    } else {
        LOG_INF("Subscribed to thresh-read notifications");
    }

    /* Continue to ADC subscribe */
    if (adc_notify_handle != 0) {
        subscribe_adc_notify();
    } else {
        disc_phase = DISC_COMPLETE;
        service_discovered = true;
        LOG_INF("HEBALL BLE discovery complete");
    }
}

static void subscribe_adc_notify(void)
{
    disc_phase = DISC_SUBSCRIBE_ADC;

    adc_notify_sub.notify = adc_notify_cb;
    adc_notify_sub.value_handle = adc_notify_handle;
    adc_notify_sub.ccc_handle = adc_notify_ccc_handle;
    adc_notify_sub.end_handle = svc_end_handle;
    adc_notify_sub.value = BT_GATT_CCC_NOTIFY;

    int err = bt_gatt_subscribe(split_conn, &adc_notify_sub);
    if (err && err != -EALREADY) {
        LOG_ERR("ADC-notify subscribe failed (err %d)", err);
    } else {
        LOG_INF("Subscribed to ADC notifications");
    }

    disc_phase = DISC_COMPLETE;
    service_discovered = true;
    LOG_INF("HEBALL BLE discovery complete");
}

/* -----------------------------------------------------------------------
 * Delayed discovery start (allows ZMK split to settle first)
 * ----------------------------------------------------------------------- */
static void discovery_start_work_handler(struct k_work *work)
{
    if (!split_conn) {
        return;
    }
    LOG_INF("Starting HEBALL GATT discovery on split peer");
    start_heball_service_discovery();
}

/* -----------------------------------------------------------------------
 * Connection callbacks -- detect left-half peer
 * ----------------------------------------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        return;
    }

    /* Only act on connections where we are the central (initiator). */
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0) {
        return;
    }
    if (info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }

    LOG_INF("BLE central: peer connected, scheduling HEBALL discovery");

    /* Store connection -- we only support one split peer */
    if (split_conn) {
        bt_conn_unref(split_conn);
    }
    split_conn = bt_conn_ref(conn);
    service_discovered = false;
    thresh_write_handle = 0;
    thresh_read_handle = 0;
    adc_notify_handle = 0;
    thresh_read_ccc_handle = 0;
    adc_notify_ccc_handle = 0;
    discovery_retry_count = 0;

    /* Bug 2 fix: 500ms delay (was 2s) to let ZMK split settle */
    k_work_reschedule(&discovery_start_work, K_MSEC(500));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (split_conn != conn) {
        return;
    }

    LOG_INF("BLE central: split peer disconnected (reason %u)", reason);
    k_work_cancel_delayable(&discovery_start_work);

    service_discovered = false;
    thresh_write_handle = 0;
    thresh_read_handle = 0;
    adc_notify_handle = 0;
    thresh_read_ccc_handle = 0;
    adc_notify_ccc_handle = 0;
    disc_phase = DISC_IDLE;

    /* Wake any blocked send_and_wait caller */
    k_mutex_lock(&ble_resp_mutex, K_FOREVER);
    ble_pending_cmd_id = 0;
    k_sem_give(&ble_resp_sem);
    k_mutex_unlock(&ble_resp_mutex);

    bt_conn_unref(split_conn);
    split_conn = NULL;
}

BT_CONN_CB_DEFINE(heball_central_conn_cb) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* -----------------------------------------------------------------------
 * Initialization
 * ----------------------------------------------------------------------- */
static int heball_ble_central_init(void)
{
    k_work_init_delayable(&discovery_start_work, discovery_start_work_handler);
    LOG_INF("HEball BLE central initialized");
    return 0;
}

SYS_INIT(heball_ble_central_init, APPLICATION, 91);