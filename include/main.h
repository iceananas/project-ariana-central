#pragma once

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <logging/log.h>
#include <math.h>

#include <sys/util.h>
#include <sys/reboot.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#define SCAN_INTERVAL 0x0140 /* 200 ms */
#define SCAN_WINDOW 0x0030   /* 30 ms */
#define INIT_INTERVAL 0x0010 /* 10 ms */
#define INIT_WINDOW 0x0010   /* 10 ms */
#define CONN_INTERVAL 0x00A0 /* 200 ms */
#define CONN_LATENCY 0
#define CONN_TIMEOUT MIN(MAX((CONN_INTERVAL * 125 * MAX(CONFIG_BT_MAX_CONN, 6) / 1000), 10), 3200)

#define M_PI_2		1.57079632679489661923

// BLE data structures
static struct bt_conn *bt_connection[CONFIG_BT_MAX_CONN];
static uint8_t volatile conn_count = 0;
static bool volatile is_disconnecting;

static struct bt_gatt_discover_params discover_params;
static struct bt_uuid_128 light_control_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x1d89ca61, 0x7967, 0x4bcd, 0x9fda, 0xdda8013ada2c));
static struct bt_uuid_128 ww_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x00b12566, 0xdcb3, 0x4c4f, 0xaf5c, 0x82a20d56f2b5));
static struct bt_uuid_128 cw_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x81a273e6, 0x5528, 0x4266, 0xa7d9, 0x05718297bc5c));
static uint16_t ww_handle[CONFIG_BT_MAX_CONN];
static uint16_t cw_handle[CONFIG_BT_MAX_CONN];

// Helper variables
static bool discover_completed = false;