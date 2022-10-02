#pragma once

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <logging/log.h>
#include <math.h>
#include <device.h>
#include <stdlib.h>

#include <sys/util.h>
#include <sys/reboot.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/led_strip.h>

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

#define M_PI_2 1.57079632679489661923

// Pixels
#define STRIP_NODE DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(DT_ALIAS(led_strip), chain_length)
static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
struct led_rgb pixels[STRIP_NUM_PIXELS];

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }
