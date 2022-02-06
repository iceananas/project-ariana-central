#pragma once

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <logging/log.h>

#include <sys/util.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

// BLE data structures
static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_uuid_128 light_control_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x1d89ca61, 0x7967, 0x4bcd, 0x9fda, 0xdda8013ada2c));
static struct bt_uuid_128 ww_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x00b12566, 0xdcb3, 0x4c4f, 0xaf5c, 0x82a20d56f2b5));
static struct bt_uuid_128 cw_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x81a273e6, 0x5528, 0x4266, 0xa7d9, 0x05718297bc5c));
static uint16_t ww_handle = 0;
static uint16_t cw_handle = 0;