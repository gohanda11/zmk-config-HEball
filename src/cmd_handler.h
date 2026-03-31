/*
 * SPDX-License-Identifier: MIT
 * HEball binary command protocol handler.
 */
#pragma once

#include <stdint.h>

/* Protocol constants */
#define HEBALL_FRAME_START      0xAB
#define HEBALL_STATUS_OK        0x00
#define HEBALL_STATUS_ERROR     0xFF
#define HEBALL_PROTOCOL_VERSION 1
/*
 * Maximum frame length in bytes.
 * On-wire LEN field is uint8_t (max 255), so the largest possible frame is:
 *   START(1) + LEN(1) + body(255) + CRC(1) = 258 bytes.
 * Max payload in a response = 253 (LEN encodes CMD_ID + STATUS + PAYLOAD).
 */
#define HEBALL_MAX_FRAME_LEN    258
#define HEBALL_FRAME_TIMEOUT_MS 100
#define HEBALL_MAX_KEYS         32

/* Command IDs */
#define CMD_GET_VERSION         0x01
#define CMD_GET_NUM_KEYS        0x02
#define CMD_GET_THRESHOLDS      0x03
#define CMD_SET_THRESHOLD       0x04
#define CMD_SET_THRESHOLDS_BULK 0x05
#define CMD_GET_ADC_VALUES      0x06
#define CMD_SAVE_SETTINGS       0x07
#define CMD_RESET_DEFAULTS      0x08
#define CMD_GET_CALIBRATION     0x09
#define CMD_RECALIBRATE         0x0A
#define CMD_STREAM_ADC_START    0x10
#define CMD_STREAM_ADC_STOP     0x11
#define CMD_STREAM_ADC_DATA     0x12

/* Half identifiers */
#define HEBALL_HALF_RIGHT  0x00
#define HEBALL_HALF_LEFT   0x01

/**
 * Initialize the command handler.
 * Called via SYS_INIT at APPLICATION priority.
 */
int cmd_handler_init(void);
