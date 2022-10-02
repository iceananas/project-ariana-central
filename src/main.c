#include "main.h"
#include "ft5xx6.h"

#define LOG_LEVEL 3
LOG_MODULE_REGISTER(main);
#define SLEEP_TIME_MS 10

// Logic variables and functions
static uint8_t brightness_value = 0;
static uint8_t ww_value = 0;
static uint8_t cw_value = 0;
static uint8_t color[2] = {0, 0};
static uint8_t calculate_brightness(int x, int y);
static void calculate_color(int x, int y, uint8_t *color);

// Config and functions for CTPM
#define I2C DT_NODELABEL(i2c0)
#define INT_PIN DT_NODELABEL(ctpmint)
static const struct gpio_dt_spec ctpm_int = GPIO_DT_SPEC_GET(INT_PIN, gpios);
static struct gpio_callback ctpm_int_cb_data;
static bool ctpm_event_flag = false;
static struct point coordinates = {.x = 0, .y = 0};
static const struct device *ctpm_dev;
void on_interrupt_received(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ctpm_event_flag = true;
}

// LED Animation functions
void fadeToBlack(int ledNo, double fadeValue);
void meteorRain(struct led_rgb meteorColor, int meteorSize);
struct led_rgb meteor_color = RGB(0x70, 0x80, 0xa0);

void animationLoop0() {
    while (1) {
        meteorRain(meteor_color, 1);
    }
}

void animationLoop1() {
    while (1) {
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            for (int j = 0; j < STRIP_NUM_PIXELS; j++) {
                if (rand() % 2) {
                    if (rand() % 2) {
                        fadeToBlack(j, 8);
                    }
                }
            }
            led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
            k_msleep(20);
        }
    }
}

K_THREAD_DEFINE(animation0, 2048, animationLoop0, NULL, NULL, NULL, 8, 0, 0);
K_THREAD_DEFINE(animation1, 2048, animationLoop1, NULL, NULL, NULL, 9, 0, 0);

// BLE advertising data
uint8_t ww_data[] = {0x68, 0x68, 0x09};
uint8_t cw_data[] = {0x69, 0x69, 0x05};
static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_SVC_DATA16, ww_data, 3),
    BT_DATA(BT_DATA_SVC_DATA16, cw_data, 3),
};

// Main loop
void main(void) {
    int err;

    if (device_is_ready(strip)) {
        LOG_INF("Found LED strip device %s", strip->name);
    } else {
        LOG_ERR("LED strip device %s is not ready", strip->name);
        return;
    }

    /* Initialize BLE */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)\n", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    /* Initialize CTPM */
    ctpm_dev = device_get_binding(DT_LABEL(I2C));
    ft5xx6_init(ctpm_dev, ctpm_int);

    /* Register Callbacks */
    gpio_init_callback(&ctpm_int_cb_data, on_interrupt_received, BIT(ctpm_int.pin));
    gpio_add_callback(ctpm_int.port, &ctpm_int_cb_data);
    LOG_INF("Set up Interrupt at %s pin %d\n", ctpm_int.port->name, ctpm_int.pin);

    struct ft5xx6_info tpcm_info = ft5xx6_get_info(ctpm_dev);
    LOG_INF("FT5436 Chip ID Version: %d", tpcm_info.chip_id);
    LOG_INF("FT5436 Chip Device Mode: %d", tpcm_info.device_mode);
    LOG_INF("FT5436 Chip Firmware Version: %d", tpcm_info.firmware_version);
    LOG_INF("FT5436 Chip Run State: %d", tpcm_info.run_state);
    LOG_INF("FT5436 Chip Firmware ID: %d", tpcm_info.firmware_id);

    k_sleep(K_SECONDS(5));

    err = bt_le_adv_start(BT_LE_ADV_NCONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    while (1) {
        if (ctpm_event_flag) {
            coordinates = ft_5xx6_get_coordinates(ctpm_dev);
            brightness_value = calculate_brightness(coordinates.x, coordinates.y);
            calculate_color(coordinates.x - 255, coordinates.y - 255, color);
            ww_value = color[0] * ((float)brightness_value / 255);
            cw_value = color[1] * ((float)brightness_value / 255);

            if (ww_value < 30) {
                ww_value = 10;
            }
            if (cw_value < 30) {
                cw_value = 5;
            }
            LOG_DBG("Coordinates: x %d \t y %d", coordinates.x, coordinates.y);
            LOG_DBG("Brightness: %d", brightness_value);
            LOG_DBG("White colors: WW %d, CW %d", ww_value, cw_value);

            ctpm_event_flag = false;

            ww_data[2] = ww_value;
            cw_data[2] = cw_value;
        }

        err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            printk("Advertising failed to update (err %d)\n", err);
            return;
        }
        k_msleep(SLEEP_TIME_MS);
    }
}

// Brightnes logic: center of touchpad = lowest, outside of touchpad = highest
static uint8_t calculate_brightness(int x, int y) {
    uint8_t r = 0;
    x -= 255;
    y -= 255;
    r = sqrt(x * x + y * y);
    return r;
}

// White temperature logic: Top = cold white, Bottom = warm white
static void calculate_color(int x, int y, uint8_t *color) {
    double theta = 0;

    if (x >= 0) {
        // First quadrant
        if (y >= 0) {
            theta = atan2(y, x);
            color[0] = 255 - (255 * (theta / M_PI_2));
            color[1] = 255;
        }
        // Second quadrant
        else if (y < 0) {
            theta = -atan2(y, x);
            color[0] = 255;
            color[1] = 255 - (255 * (theta / M_PI_2));
        }
    } else {
        // Third quadrant
        if (y < 0) {
            theta = -atan2(y, -x);
            color[0] = 255;
            color[1] = 255 - (255 * (theta / M_PI_2));
        }
        // Fourth quadrant
        else if (y >= 0) {
            theta = atan2(y, -x);
            color[0] = 255 - (255 * (theta / M_PI_2));
            color[1] = 255;
        }
    }
}

// Fade out
void fadeToBlack(int ledNo, double fadeValue) {
    struct led_rgb oldColor = pixels[ledNo];
    uint8_t r, g, b;

    r = oldColor.r;
    g = oldColor.g;
    b = oldColor.b;

    oldColor.r = (r <= 10) ? 0 : (int)r - (int)ceil(((double)r * fadeValue / 256));
    oldColor.g = (g <= 10) ? 0 : (int)g - (int)ceil(((double)g * fadeValue / 256));
    oldColor.b = (b <= 10) ? 0 : (int)b - (int)ceil(((double)b * fadeValue / 256));

    memcpy(&pixels[ledNo], &oldColor, sizeof(struct led_rgb));
}

// Draw meteor
void meteorRain(struct led_rgb meteorColor, int meteorSize) {
    memset(&pixels, 0x00, sizeof(pixels));

    for (int i = 0; i < STRIP_NUM_PIXELS + STRIP_NUM_PIXELS / 2; i++) {
        for (int j = 0; j < meteorSize; j++) {
            if ((i - j < STRIP_NUM_PIXELS) && (i - j >= 0)) {
                memcpy(&pixels[i - j], &meteorColor, sizeof(struct led_rgb));
            }
        }
        led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
        k_msleep(1200);
    }
}