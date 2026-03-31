/*
 * SPDX-License-Identifier: MIT
 * HEball BLE GATT central — right-half client implementation.
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

/* ZMK split BLE service UUID — used to confirm the peer is our left half */
#define ZMK_BT_SPLIT_UUID(num) \
    BT_UUID_128_ENCODE(num, 0x0096, 0x7107, 0xc967, 0xc5cfb1c2482a)
#define ZMK_SPLIT_BT_SERVICE_UUID ZMK_BT_SPLIT_UUID(0x00000000)

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

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params thresh_read_sub;
static struct bt_gatt_subscribe_params adc_notify_sub;

/* Response slot (protected by single-threaded workqueue assumption) */
#define RESPONSE_BUF_SIZE 256
static uint8_t response_buf[RESPONSE_BUF_SIZE];
static uint16_t response_len;
static volatile bool response_ready;

/* ADC callback */
static heball_ble_adc_cb_t adc_callback;

/* Discovery phase tracking */
typedef enum {
    DISC_IDLE,
    DISC_ZMK_SERVICE,     /* checking for ZMK split service */
    DISC_HEBALL_SERVICE,  /* searching for our custom service */
    DISC_CHARACTERISTICS, /* enumerating characteristics */
    DISC_SUBSCRIBE_THRESH,
    DISC_SUBSCRIBE_ADC,
    DISC_COMPLETE,
} disc_phase_t;

static disc_phase_t disc_phase = DISC_IDLE;

/* Delayed work for starting discovery after connection settles */
static struct k_work_delayable discovery_start_work;

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */
bool heball_ble_central_is_connected(void)
{
    return split_conn != NULL && service_discovered;
}

int heball_ble_central_write_cmd(const uint8_t *data, uint16_t len)
{
    if (!split_conn || !service_discovered || thresh_write_handle == 0) {
        return -ENOTCONN;
    }

    return bt_gatt_write_without_response(split_conn, thresh_write_handle,
                                          data, len, false);
}

int heball_ble_central_get_response(uint8_t *out_buf, uint16_t max_len)
{
    if (!response_ready) {
        return -EAGAIN;
    }

    uint16_t copy_len = (response_len < max_len) ? response_len : max_len;
    memcpy(out_buf, response_buf, copy_len);
    response_ready = false;
    return (int)copy_len;
}

bool heball_ble_central_response_ready(void)
{
    return response_ready;
}

void heball_ble_central_clear_response(void)
{
    response_ready = false;
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

    LOG_DBG("Thresh-read notify: %u bytes", length);

    if (length > 0 && length <= RESPONSE_BUF_SIZE) {
        memcpy(response_buf, data, length);
        response_len = length;
        response_ready = true;
    }

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

/* Forward declarations */
static void start_heball_service_discovery(void);
static void start_characteristic_discovery(uint16_t start_handle, uint16_t end_handle);
static void subscribe_thresh_read(void);
static void subscribe_adc_notify(void);

static uint16_t svc_start_handle;
static uint16_t svc_end_handle;

/* ZMK split service discovery callback — just confirms the peer is valid */
static uint8_t zmk_svc_discover_cb(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("ZMK split service not found (expected on split peer) — "
                "proceeding with HEBALL discovery anyway");
        start_heball_service_discovery();
        return BT_GATT_ITER_STOP;
    }

    LOG_INF("ZMK split service found — confirmed split peer");
    start_heball_service_discovery();
    return BT_GATT_ITER_STOP;
}

/* HEBALL service discovery callback */
static uint8_t heball_svc_discover_cb(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_WRN("HEBALL custom service not found on peer");
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

        if (thresh_read_handle != 0) {
            subscribe_thresh_read();
        } else if (adc_notify_handle != 0) {
            subscribe_adc_notify();
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

/* Start discovery for ZMK split service (peer validation) */
static void start_zmk_discovery(struct bt_conn *conn)
{
    static struct bt_uuid_128 zmk_split_uuid =
        BT_UUID_INIT_128(ZMK_SPLIT_BT_SERVICE_UUID);

    disc_phase = DISC_ZMK_SERVICE;
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = &zmk_split_uuid.uuid;
    discover_params.func = zmk_svc_discover_cb;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        LOG_ERR("ZMK service discovery failed (err %d)", err);
        disc_phase = DISC_IDLE;
    }
}

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
        LOG_ERR("HEBALL service discovery failed (err %d)", err);
        disc_phase = DISC_IDLE;
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
        LOG_ERR("Characteristic discovery failed (err %d)", err);
        disc_phase = DISC_IDLE;
    }
}

static void subscribe_thresh_read(void)
{
    disc_phase = DISC_SUBSCRIBE_THRESH;

    thresh_read_sub.notify = thresh_read_notify_cb;
    thresh_read_sub.value_handle = thresh_read_handle;
    /* CCCD handle is value_handle + 1 for standard GATT layout */
    thresh_read_sub.ccc_handle = thresh_read_handle + 1;
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
    adc_notify_sub.ccc_handle = adc_notify_handle + 1;
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
    start_zmk_discovery(split_conn);
}

/* -----------------------------------------------------------------------
 * Connection callbacks — detect left-half peer
 * ----------------------------------------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        return;
    }

    /* Only act on connections where we are the central (initiator).
     * On the right half (ZMK central), outgoing connections go to peripherals. */
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0) {
        return;
    }
    if (info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }

    LOG_INF("BLE central: peer connected, scheduling HEBALL discovery");

    /* Store connection — we only support one split peer */
    if (split_conn) {
        bt_conn_unref(split_conn);
    }
    split_conn = bt_conn_ref(conn);
    service_discovered = false;
    thresh_write_handle = 0;
    thresh_read_handle = 0;
    adc_notify_handle = 0;

    /* Wait 2 s for ZMK split handshake to settle, then start our discovery */
    k_work_reschedule(&discovery_start_work, K_SECONDS(2));
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
    response_ready = false;
    disc_phase = DISC_IDLE;

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

SYS_INIT(heball_ble_central_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY + 2);
