/**
 * @file
 * @brief Driver for Focaltech FT5xx6 CTPMs
 * @author Bigbear (Shiyue Liu)
 */

#ifndef __FT5XX6_H__
#define __FT5XX6_H__

#include <device.h>
#include <zephyr/types.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FT5XX6_I2C_ADDRESS 0x38

#define FT5XX6_DEV_MODE_REGISTER 0x00
#define FT5XX6_GESTURE_REGISTER 0x01
#define FT5XX6_TOUCHX_STATUS_REGISTER 0x03
#define FT5XX6_TOUCHY_STATUS_REGISTER 0x05
#define FT5XX6_TOUCH_WEIGHT_REGISTER 0x08
#define FT5XX6_LEFT_RIGHT_OFFSET_REGISTER 0x94
#define FT5XX6_UP_DOWN_OFFSET_REGISTER 0x95
#define FT5XX6_MAX_XRES_REGISTER 0x98
#define FT5XX6_MAX_YRES_REGISTER 0x9A

#define FT5XX6_FIRMWARE_VERSION_REGISTER 0xA1
#define FT5XX6_CHIP_ID_REGISTER 0xA3
#define FT5XX6_FIRMWARE_ID_REGISTER 0xA6
#define FT5XX6_POWER_MODE_REGISTER 0xA6
#define FT5XX6_RUN_STATE_REGISTER 0xA7
#define FT5XX6_ERROR_REGISTER 0xA9

#define FT5XX6_CODE_OK 0x00

typedef enum { START = 0, END = 1, TOUCH = 2 } EventFlag;

struct point {
    uint16_t x;
    uint16_t y;
    EventFlag event;
};

typedef enum {
    CONFIGURE = 0,
    WORK = 1,
    CALIBRATION = 2,
    FACTORY = 3,
    AUTO_CALIBRATION = 4
} RunState;

typedef enum { ACTIVE = 0, MONITOR = 1, HIBERNATE = 3 } PowerMode;

typedef enum { OP = 0, SYS_INFO = 1, TEST = 4, TEST1 = 5 } DeviceMode;

struct ft5xx6_info {
    uint8_t firmware_id;
    uint16_t firmware_version;
    uint8_t chip_id;
    uint8_t error_code;
    PowerMode power_mode;
    RunState run_state;
    DeviceMode device_mode;
};

int ft5xx6_init(const struct device *i2cdev, const struct gpio_dt_spec gpiodev) {
    /* I2C checks and initializations */
    if (i2cdev == NULL) {
        printk("I2C: Device driver not found\n");
        return -1;
    }

    i2c_configure(i2cdev, I2C_SPEED_SET(I2C_SPEED_FAST));

    /* Interrupt pin checks and initializations */
    if (!device_is_ready(gpiodev.port)) {
        printk("Error: CTPM Interrupt pin %s is not ready\n", gpiodev.port->name);
        return -2;
    }

    // Configure Pins
    int ret = gpio_pin_configure_dt(&gpiodev, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n", ret, gpiodev.port->name, gpiodev.pin);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&gpiodev, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, gpiodev.port->name,
               gpiodev.pin);
        return ret;
    }

    return 0;
}

int ft5xx6_read_reg(const struct device *i2cdev, int data_register) {
    uint8_t data[1];
    int ret = i2c_reg_read_byte(i2cdev, FT5XX6_I2C_ADDRESS, data_register, data);
    if (ret) {
        printk("Error reading from Touch Panel! error code (%d)\n", ret);
        return -1;
    }
    return data[0];
}

struct ft5xx6_info ft5xx6_get_info(const struct device *i2cdev) {
    struct ft5xx6_info info;
    // Chip ID
    info.chip_id = ft5xx6_read_reg(i2cdev, FT5XX6_CHIP_ID_REGISTER);
    // Firmware ID
    info.firmware_id = ft5xx6_read_reg(i2cdev, FT5XX6_FIRMWARE_ID_REGISTER);
    // Firmware version
    int firmware_version_msb = ft5xx6_read_reg(i2cdev, FT5XX6_CHIP_ID_REGISTER);
    int firmware_version_lsb = ft5xx6_read_reg(i2cdev, FT5XX6_CHIP_ID_REGISTER + 1);
    info.firmware_version = ((uint16_t)firmware_version_msb << 8) + (uint16_t)firmware_version_lsb;
    // Power mode
    info.power_mode = ft5xx6_read_reg(i2cdev, FT5XX6_POWER_MODE_REGISTER);
    // Run state
    info.run_state = ft5xx6_read_reg(i2cdev, FT5XX6_RUN_STATE_REGISTER);
    // Error code
    info.error_code = ft5xx6_read_reg(i2cdev, FT5XX6_ERROR_REGISTER);
    // Device mode
    int device_mode_raw = ft5xx6_read_reg(i2cdev, FT5XX6_DEV_MODE_REGISTER);
    uint8_t device_mode = (device_mode_raw & 0x70) >> 4;
    info.device_mode = device_mode;

    return info;
}

struct point ft_5xx6_get_coordinates(const struct device *i2cdev) {
    struct point coords;
    uint8_t buffer[4];
    buffer[0] = ft5xx6_read_reg(i2cdev, FT5XX6_TOUCHX_STATUS_REGISTER);
    buffer[1] = ft5xx6_read_reg(i2cdev, FT5XX6_TOUCHX_STATUS_REGISTER + 1);
    buffer[2] = ft5xx6_read_reg(i2cdev, FT5XX6_TOUCHY_STATUS_REGISTER);
    buffer[3] = ft5xx6_read_reg(i2cdev, FT5XX6_TOUCHY_STATUS_REGISTER + 1);

    coords.x = (((uint16_t)buffer[0] & 0b00001111) << 8) | (uint16_t)buffer[1];
    coords.y = (((uint16_t)buffer[2] & 0b00001111) << 8) | (uint16_t)buffer[3];
    coords.event = 2;

    return coords;
}

#ifdef __cplusplus
}
#endif

#endif