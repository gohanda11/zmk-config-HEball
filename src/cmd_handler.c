/*
 * SPDX-License-Identifier: MIT
 * HEball binary command handler — CDC ACM serial protocol.
 */

#include "cmd_handler.h"
#include <zmk_kscan_he_api.h>

#ifdef CONFIG_HEBALL_BLE_CENTRAL
#include "heball_ble_central.h"
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <string.h>

LOG_MODULE_REGISTER(heball_cmd, CONFIG_ZMK_LOG_LEVEL);

/* -----------------------------------------------------------------------
 * CRC-8/MAXIM: poly=0x31, refIn=true, refOut=true
 * ----------------------------------------------------------------------- */
static const uint8_t crc8_table[256] = {
    0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,
    0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,
    0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,
    0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,
    0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,
    0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,
    0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,
    0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,
    0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,
    0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,
    0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,
    0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,
    0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,
    0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,
    0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,
    0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,
    0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,
    0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,
    0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,
    0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,
    0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,
    0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,
    0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,
    0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,
    0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,
    0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,
    0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,
    0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,
    0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,
    0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,
    0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,
    0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35,
};

static uint8_t crc8_byte(uint8_t crc, uint8_t byte) {
    return crc8_table[crc ^ byte];
}

static uint8_t crc8_compute(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_byte(crc, data[i]);
    }
    return crc;
}

/* -----------------------------------------------------------------------
 * UART + ring buffer
 * ----------------------------------------------------------------------- */
#define RX_RING_BUF_SIZE 512

RING_BUF_DECLARE(rx_ring_buf, RX_RING_BUF_SIZE);

static const struct device *data_uart;

/* Shared TX frame buffer (safe: all callers run on system workqueue) */
static uint8_t tx_frame[HEBALL_MAX_FRAME_LEN];

/* -----------------------------------------------------------------------
 * Streaming state
 * ----------------------------------------------------------------------- */
static volatile bool rx_overflow;
static bool streaming_active;
static uint32_t stream_interval_ms = 50;
static struct k_work_delayable stream_work;

/* BLE response buffer size for left-half proxy */
#define RESPONSE_BUF_SIZE_CMD 256

/* -----------------------------------------------------------------------
 * Frame parser state machine
 * ----------------------------------------------------------------------- */
typedef enum {
    PARSE_WAIT_START,
    PARSE_READ_LEN,
    PARSE_READ_BODY,
} parse_state_t;

static parse_state_t parse_state = PARSE_WAIT_START;
static uint8_t  parse_len;
static uint8_t  parse_buf[HEBALL_MAX_FRAME_LEN];
static uint16_t parse_pos;
static int64_t  parse_last_byte_ms;

/* -----------------------------------------------------------------------
 * TX helpers
 * ----------------------------------------------------------------------- */
static void uart_send(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(data_uart, buf[i]);
    }
}

static void send_response(uint8_t cmd_id, uint8_t status,
                          const uint8_t *payload, uint8_t payload_len) {
    /* Frame: [0xAB][LEN][CMD_ID][STATUS][PAYLOAD...][CRC8] */
    uint8_t body_len = 2 + payload_len;
    uint16_t total_len = 5 + payload_len; /* START + LEN + CMD + STATUS + payload + CRC */

    tx_frame[0] = HEBALL_FRAME_START;
    tx_frame[1] = body_len;
    tx_frame[2] = cmd_id;
    tx_frame[3] = status;
    if (payload && payload_len > 0) {
        memcpy(&tx_frame[4], payload, payload_len);
    }
    /* CRC covers [LEN, CMD_ID, STATUS, PAYLOAD...] */
    tx_frame[4 + payload_len] = crc8_compute(&tx_frame[1], body_len + 1);
    uart_send(tx_frame, total_len);
}

static void send_ok(uint8_t cmd_id, const uint8_t *payload, uint8_t payload_len) {
    send_response(cmd_id, HEBALL_STATUS_OK, payload, payload_len);
}

static void send_error(uint8_t cmd_id) {
    send_response(cmd_id, HEBALL_STATUS_ERROR, NULL, 0);
}

/* -----------------------------------------------------------------------
 * Command dispatcher
 * ----------------------------------------------------------------------- */
static void dispatch_command(uint8_t cmd_id, const uint8_t *payload, uint8_t payload_len) {
    const struct device *kscan_dev = DEVICE_DT_GET(DT_NODELABEL(kscan_he));

    if (!device_is_ready(kscan_dev)) {
        send_error(cmd_id);
        return;
    }

    uint8_t half = (payload_len > 0) ? payload[0] : HEBALL_HALF_RIGHT;

    /* Forward left-half commands over BLE (Phase 4) */
    if (half == HEBALL_HALF_LEFT) {
#ifdef CONFIG_HEBALL_BLE_CENTRAL
        if (!heball_ble_central_is_connected()) {
            LOG_WRN("Left half BLE not connected");
            send_error(cmd_id);
            return;
        }

        /*
         * Build BLE payload: [CMD_ID][payload_after_half_byte...]
         * Strip the half byte (payload[0]) before forwarding.
         */
        uint8_t ble_buf[HEBALL_MAX_FRAME_LEN];
        ble_buf[0] = cmd_id;
        uint8_t fwd_len = 1;
        if (payload_len > 1) {
            memcpy(&ble_buf[1], &payload[1], payload_len - 1);
            fwd_len += payload_len - 1;
        }

        int err = heball_ble_central_write_cmd(ble_buf, fwd_len);
        if (err) {
            LOG_ERR("BLE write to left half failed (err %d)", err);
            send_error(cmd_id);
            return;
        }

        /*
         * Wait for the response via BLE notification (thresh-read char).
         * Poll with a timeout — recalibrate may take up to 500 ms.
         */
        int64_t deadline = k_uptime_get() + 2000;
        while (!heball_ble_central_response_ready()) {
            if (k_uptime_get() > deadline) {
                LOG_WRN("BLE response timeout from left half");
                send_error(cmd_id);
                return;
            }
            k_sleep(K_MSEC(5));
        }

        /* Response format from peripheral: [CMD_ID][STATUS][payload...] */
        uint8_t resp_buf[RESPONSE_BUF_SIZE_CMD];
        int resp_len = heball_ble_central_get_response(resp_buf, sizeof(resp_buf));
        if (resp_len < 2) {
            send_error(cmd_id);
            return;
        }

        uint8_t resp_status = resp_buf[1];
        if (resp_len > 2) {
            send_response(cmd_id, resp_status, &resp_buf[2], (uint8_t)(resp_len - 2));
        } else {
            send_response(cmd_id, resp_status, NULL, 0);
        }
        return;
#else
        send_error(cmd_id);
        return;
#endif /* CONFIG_HEBALL_BLE_CENTRAL */
    }

    uint8_t num_keys = 0;
    zmk_kscan_he_get_num_keys(kscan_dev, &num_keys);
    if (num_keys > HEBALL_MAX_KEYS) {
        num_keys = HEBALL_MAX_KEYS;
    }

    switch (cmd_id) {

    case CMD_GET_VERSION: {
        uint8_t resp[2] = { HEBALL_PROTOCOL_VERSION, HEBALL_HALF_RIGHT };
        send_ok(cmd_id, resp, 2);
        break;
    }

    case CMD_GET_NUM_KEYS: {
        send_ok(cmd_id, &num_keys, 1);
        break;
    }

    case CMD_GET_THRESHOLDS: {
        uint8_t buf[HEBALL_MAX_KEYS * 2];
        for (uint8_t k = 0; k < num_keys; k++) {
            uint8_t p = 0, r = 0;
            zmk_kscan_he_get_threshold(kscan_dev, k, &p, &r);
            buf[k * 2]     = p;
            buf[k * 2 + 1] = r;
        }
        send_ok(cmd_id, buf, num_keys * 2);
        break;
    }

    case CMD_SET_THRESHOLD: {
        if (payload_len < 4) { send_error(cmd_id); break; }
        uint8_t key_idx = payload[1];
        uint8_t press   = payload[2];
        uint8_t release = payload[3];
        int ret = zmk_kscan_he_set_threshold(kscan_dev, key_idx, press, release);
        if (ret < 0) { send_error(cmd_id); break; }
        send_ok(cmd_id, NULL, 0);
        break;
    }

    case CMD_SET_THRESHOLDS_BULK: {
        if (payload_len < 3) { send_error(cmd_id); break; }
        uint8_t start = payload[1];
        uint8_t count = payload[2];
        if (payload_len < 3 + count * 2) { send_error(cmd_id); break; }
        bool had_error = false;
        for (uint8_t i = 0; i < count; i++) {
            uint8_t press   = payload[3 + i * 2];
            uint8_t release = payload[3 + i * 2 + 1];
            int ret = zmk_kscan_he_set_threshold(kscan_dev, start + i, press, release);
            if (ret < 0) { had_error = true; break; }
        }
        if (had_error) {
            send_error(cmd_id);
        } else {
            send_ok(cmd_id, NULL, 0);
        }
        break;
    }

    case CMD_GET_ADC_VALUES: {
        uint8_t buf[HEBALL_MAX_KEYS * 3];
        for (uint8_t k = 0; k < num_keys; k++) {
            uint16_t adc = 0;
            uint8_t  dist = 0;
            zmk_kscan_he_get_adc_raw(kscan_dev, k, &adc, &dist);
            buf[k * 3]     = (uint8_t)(adc & 0xFF);
            buf[k * 3 + 1] = (uint8_t)(adc >> 8);
            buf[k * 3 + 2] = dist;
        }
        send_ok(cmd_id, buf, num_keys * 3);
        break;
    }

    case CMD_SAVE_SETTINGS: {
        int ret = zmk_kscan_he_save_settings(kscan_dev);
        if (ret < 0) { send_error(cmd_id); break; }
        send_ok(cmd_id, NULL, 0);
        break;
    }

    case CMD_RESET_DEFAULTS: {
        int ret = zmk_kscan_he_reset_defaults(kscan_dev);
        if (ret < 0) { send_error(cmd_id); break; }
        send_ok(cmd_id, NULL, 0);
        break;
    }

    case CMD_GET_CALIBRATION: {
        uint8_t buf[HEBALL_MAX_KEYS * 4];
        for (uint8_t k = 0; k < num_keys; k++) {
            uint16_t rest = 0, bottom = 0;
            zmk_kscan_he_get_calibration(kscan_dev, k, &rest, &bottom);
            buf[k * 4]     = (uint8_t)(rest & 0xFF);
            buf[k * 4 + 1] = (uint8_t)(rest >> 8);
            buf[k * 4 + 2] = (uint8_t)(bottom & 0xFF);
            buf[k * 4 + 3] = (uint8_t)(bottom >> 8);
        }
        send_ok(cmd_id, buf, num_keys * 4);
        break;
    }

    case CMD_RECALIBRATE: {
        kscan_disable_callback(kscan_dev);
        zmk_kscan_he_recalibrate(kscan_dev);
        kscan_enable_callback(kscan_dev);
        send_ok(cmd_id, NULL, 0);
        break;
    }

    case CMD_STREAM_ADC_START: {
        uint32_t interval = (payload_len >= 2) ? payload[1] : 50;
        if (interval < 50) {
            interval = 50;
        }
        stream_interval_ms = interval;
        streaming_active = true;
        k_work_reschedule(&stream_work, K_MSEC(stream_interval_ms));
        send_ok(cmd_id, NULL, 0);
        break;
    }

    case CMD_STREAM_ADC_STOP: {
        streaming_active = false;
        k_work_cancel_delayable(&stream_work);
        send_ok(cmd_id, NULL, 0);
        break;
    }

    default:
        send_error(cmd_id);
        break;
    }
}

/* -----------------------------------------------------------------------
 * ADC stream work handler
 * ----------------------------------------------------------------------- */
static void stream_work_handler(struct k_work *work) {
    if (!streaming_active) {
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

    /* Stream frame: [0xAB][LEN][0x12][key_idx, adc_lo, adc_hi, dist] × N [CRC] */
    uint8_t body_len = 1 + num_keys * 4; /* CMD_ID + payload */
    uint16_t total_len = 2 + body_len + 1; /* START + LEN + body + CRC */

    tx_frame[0] = HEBALL_FRAME_START;
    tx_frame[1] = body_len;
    tx_frame[2] = CMD_STREAM_ADC_DATA;

    for (uint8_t k = 0; k < num_keys; k++) {
        uint16_t adc = 0;
        uint8_t  dist = 0;
        zmk_kscan_he_get_adc_raw(kscan_dev, k, &adc, &dist);
        tx_frame[3 + k * 4]     = k;
        tx_frame[3 + k * 4 + 1] = (uint8_t)(adc & 0xFF);
        tx_frame[3 + k * 4 + 2] = (uint8_t)(adc >> 8);
        tx_frame[3 + k * 4 + 3] = dist;
    }

    /* CRC covers [LEN .. end of payload] */
    tx_frame[3 + num_keys * 4] = crc8_compute(&tx_frame[1], body_len + 1);
    uart_send(tx_frame, total_len);

reschedule:
    k_work_reschedule(&stream_work, K_MSEC(stream_interval_ms));
}

/* -----------------------------------------------------------------------
 * RX processing work (runs in system workqueue)
 * ----------------------------------------------------------------------- */
static struct k_work rx_process_work;

static void rx_process_work_handler(struct k_work *work) {
    uint8_t byte;

    if (rx_overflow) {
        rx_overflow = false;
        LOG_WRN("RX ring buffer overflow — bytes were dropped");
    }

    while (ring_buf_get(&rx_ring_buf, &byte, 1) == 1) {
        int64_t now = k_uptime_get();

        /* Timeout check: discard partial frame */
        if (parse_state != PARSE_WAIT_START &&
            (now - parse_last_byte_ms) > HEBALL_FRAME_TIMEOUT_MS) {
            parse_state = PARSE_WAIT_START;
            parse_pos = 0;
        }
        parse_last_byte_ms = now;

        switch (parse_state) {
        case PARSE_WAIT_START:
            if (byte == HEBALL_FRAME_START) {
                parse_state = PARSE_READ_LEN;
                parse_pos = 0;
            }
            break;

        case PARSE_READ_LEN:
            parse_len = byte;
            if (parse_len == 0 || (uint16_t)parse_len + 3 > HEBALL_MAX_FRAME_LEN) {
                parse_state = PARSE_WAIT_START;
            } else {
                parse_state = PARSE_READ_BODY;
                parse_pos = 0;
            }
            break;

        case PARSE_READ_BODY:
            parse_buf[parse_pos++] = byte;
            /* body (parse_len bytes) + CRC (1 byte) */
            if (parse_pos == (uint16_t)parse_len + 1) {
                uint8_t received_crc = parse_buf[parse_len];

                /* Compute CRC over [LEN, body...] without VLA */
                uint8_t computed_crc = 0;
                computed_crc = crc8_byte(computed_crc, parse_len);
                for (uint8_t i = 0; i < parse_len; i++) {
                    computed_crc = crc8_byte(computed_crc, parse_buf[i]);
                }

                if (received_crc == computed_crc && parse_len >= 1) {
                    uint8_t cmd_id = parse_buf[0];
                    uint8_t *cmd_payload = &parse_buf[1];
                    uint8_t plen = parse_len - 1;
                    dispatch_command(cmd_id, cmd_payload, plen);
                }
                parse_state = PARSE_WAIT_START;
                parse_pos = 0;
            }
            break;
        }
    }
}

/* -----------------------------------------------------------------------
 * UART ISR: push bytes to ring buffer, schedule work
 * ----------------------------------------------------------------------- */
static void uart_rx_isr(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    if (!uart_irq_update(dev)) {
        return;
    }
    if (!uart_irq_rx_ready(dev)) {
        return;
    }

    uint8_t buf[32];
    int count;
    while ((count = uart_fifo_read(dev, buf, sizeof(buf))) > 0) {
        int written = ring_buf_put(&rx_ring_buf, buf, count);
        if (written < count) {
            rx_overflow = true;
        }
    }
    k_work_submit(&rx_process_work);
}

/* -----------------------------------------------------------------------
 * BLE ADC forwarding — left-half ADC data received via BLE → USB
 * ----------------------------------------------------------------------- */
#ifdef CONFIG_HEBALL_BLE_CENTRAL
static void ble_adc_forward_cb(const uint8_t *data, uint16_t len)
{
    if (len < 2) {
        return;
    }

    /*
     * Data from peripheral: [CMD_STREAM_ADC_DATA][key_idx, adc_lo, adc_hi, dist] × N
     * Forward as a standard framed response over USB.
     */
    uint8_t body_len = (uint8_t)len;
    uint16_t total_len = 2 + body_len + 1; /* START + LEN + body + CRC */

    if (total_len > HEBALL_MAX_FRAME_LEN) {
        return;
    }

    tx_frame[0] = HEBALL_FRAME_START;
    tx_frame[1] = body_len;
    memcpy(&tx_frame[2], data, len);
    tx_frame[2 + len] = crc8_compute(&tx_frame[1], body_len + 1);
    uart_send(tx_frame, total_len);
}
#endif /* CONFIG_HEBALL_BLE_CENTRAL */

/* -----------------------------------------------------------------------
 * Initialization
 * ----------------------------------------------------------------------- */
int cmd_handler_init(void) {
    data_uart = DEVICE_DT_GET(DT_ALIAS(heball_data_uart));

    if (!device_is_ready(data_uart)) {
        LOG_ERR("HEball data UART not ready");
        return -ENODEV;
    }

    k_work_init(&rx_process_work, rx_process_work_handler);
    k_work_init_delayable(&stream_work, stream_work_handler);

    uart_irq_callback_user_data_set(data_uart, uart_rx_isr, NULL);
    uart_irq_rx_enable(data_uart);

#ifdef CONFIG_HEBALL_BLE_CENTRAL
    heball_ble_central_set_adc_callback(ble_adc_forward_cb);
#endif

    LOG_INF("HEball cmd handler initialized");
    return 0;
}

SYS_INIT(cmd_handler_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY + 1);
