#include "main.h"
#include "ft5xx6.h"

#define LOG_LEVEL 3
LOG_MODULE_REGISTER(main);
#define SLEEP_TIME_MS 20
const char *ring_adresses[] = {"C3:34:B3:E9:AD:16", "E9:09:A8:54:19:5C"};

// Pixels
#define STRIP_NODE DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(DT_ALIAS(led_strip), chain_length)
static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
struct led_rgb pixels[STRIP_NUM_PIXELS];

// Logic variables and functions
static uint8_t brightness_value = 0;
static uint8_t ww_value = 0;
static uint8_t cw_value = 0;
static uint8_t color[2] = {0, 0};
static const int registered_rings = sizeof(ring_adresses) / sizeof(ring_adresses[0]);
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
    brightness_value = calculate_brightness(coordinates.x, coordinates.y);
    calculate_color(coordinates.x - 255, coordinates.y - 255, color);
    ww_value = color[0] * ((float)brightness_value / 255);
    cw_value = color[1] * ((float)brightness_value / 255);
    ctpm_event_flag = true;
}

// BLE functions and handlers
static void start_scan(void);
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params);

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                            struct net_buf_simple *ad);
static void on_connect(struct bt_conn *conn, uint8_t err);
static void on_disconnect(struct bt_conn *conn, uint8_t reason);

// Callbacks
static struct bt_conn_cb conn_callbacks = {
    .connected = on_connect,
    .disconnected = on_disconnect,
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
    bt_conn_cb_register(&conn_callbacks);
    gpio_init_callback(&ctpm_int_cb_data, on_interrupt_received, BIT(ctpm_int.pin));
    gpio_add_callback(ctpm_int.port, &ctpm_int_cb_data);
    LOG_INF("Set up Interrupt at %s pin %d\n", ctpm_int.port->name, ctpm_int.pin);

    struct ft5xx6_info tpcm_info = ft5xx6_get_info(ctpm_dev);
    LOG_INF("FT5436 Chip ID Version: %d", tpcm_info.chip_id);
    LOG_INF("FT5436 Chip Device Mode: %d", tpcm_info.device_mode);
    LOG_INF("FT5436 Chip Firmware Version: %d", tpcm_info.firmware_version);
    LOG_INF("FT5436 Chip Run State: %d", tpcm_info.run_state);
    LOG_INF("FT5436 Chip Firmware ID: %d", tpcm_info.firmware_id);

    start_scan();
    k_sleep(K_SECONDS(5));

    while (1) {
        if (ctpm_event_flag) {
            coordinates = ft_5xx6_get_coordinates(ctpm_dev);
            LOG_DBG("Coordinates: x %d \t y %d", coordinates.x, coordinates.y);
            LOG_DBG("Brightness: %d", brightness_value);
            LOG_DBG("White colors: WW %d, CW %d", ww_value, cw_value);

            for (int i = 0; i < conn_count; i++) {
                err = bt_gatt_write_without_response(bt_connection[i], ww_handle[i], &ww_value, 1,
                                                     false);
                if (err) {
                    LOG_ERR("Write data failed (err %d)", err);
                }
                err = bt_gatt_write_without_response(bt_connection[i], cw_handle[i], &cw_value, 1,
                                                     false);
                if (err) {
                    LOG_ERR("Write data failed (err %d)", err);
                }
            }
            ctpm_event_flag = false;
        }
        k_msleep(SLEEP_TIME_MS);
    }
}

// Start scan
static void start_scan(void) {
    int err;
    struct bt_le_scan_param scan_param = {
        .type = BT_HCI_LE_SCAN_PASSIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = SCAN_INTERVAL,
        .window = SCAN_WINDOW,
    };

    err = bt_le_scan_start(&scan_param, on_device_found);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)\n", err);
        return;
    }

    LOG_INF("Scanning successfully started");
}

// Handler for found devices
static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                            struct net_buf_simple *ad) {
    char addr_str[BT_ADDR_LE_STR_LEN];
    int err;
    struct bt_conn_le_create_param create_param = {
        .options = BT_CONN_LE_OPT_NONE,
        .interval = INIT_INTERVAL,
        .window = INIT_WINDOW,
        .interval_coded = 0,
        .window_coded = 0,
        .timeout = 0,
    };
    struct bt_le_conn_param conn_param = {
        .interval_min = CONN_INTERVAL,
        .interval_max = CONN_INTERVAL,
        .latency = CONN_LATENCY,
        .timeout = CONN_TIMEOUT,
    };
    if (bt_connection[conn_count]) {
        return;
    }

    /* We're only interested in connectable events */
    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("Device found: %s (RSSI %d)", log_strdup(addr_str), rssi);

    for (int i = 0; i < registered_rings; i++) {
        if (!strncmp(addr_str, ring_adresses[i], 17)) {
            LOG_INF("Ring found, trying to connect...");
            if (bt_le_scan_stop()) {
                return;
            }

            err = bt_conn_le_create(addr, &create_param, &conn_param, &bt_connection[conn_count]);

            if (err) {
                LOG_ERR("Create connection to %s failed (%u)\n", log_strdup(addr_str), err);
                start_scan();
            }
        }
    }
}

// Handler after sucessful connection
static void on_connect(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        LOG_ERR("Failed to connect to %s (%u)", log_strdup(addr), err);

        bt_conn_unref(bt_connection[conn_count]);
        bt_connection[conn_count] = NULL;

        start_scan();
        return;
    }

    LOG_INF("Connected (%u): %s", conn_count, log_strdup(addr));

    discover_params.uuid = &light_control_uuid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    discover_completed = false;
    err = bt_gatt_discover(bt_connection[conn_count], &discover_params);
    if (err) {
        LOG_ERR("Discover failed(err %d)", err);
        return;
    }
}

// Disconnect handler
static void on_disconnect(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_ERR("Disconnected: %s (reason 0x%02x)\n", log_strdup(addr), reason);
    sys_reboot(SYS_REBOOT_COLD);
}

// Discovery function
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params) {
    int err;

    if (!attr) {
        LOG_INF("Discover complete");
        (void)memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }
    LOG_INF("[ATTRIBUTE] handle %u", attr->handle);

    if (!bt_uuid_cmp(params->uuid, &light_control_uuid.uuid)) {
        LOG_INF("Light Controller Service found!");
        struct bt_gatt_service_val *service = attr->user_data;
        discover_params.uuid = &ww_uuid.uuid;
        discover_params.start_handle = attr->handle + 1;
        discover_params.end_handle = service->end_handle;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed (err %d)", err);
        }
    } else if (!bt_uuid_cmp(params->uuid, &ww_uuid.uuid)) {
        struct bt_gatt_chrc *chrc = attr->user_data;
        LOG_INF("WW Characteristic discovered at handle %u", attr->handle);

        ww_handle[conn_count] = chrc->value_handle;
        discover_params.uuid = &cw_uuid.uuid;
        discover_params.start_handle = chrc->value_handle;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed (err %d)", err);
        }
    } else if (!bt_uuid_cmp(params->uuid, &cw_uuid.uuid)) {
        struct bt_gatt_chrc *chrc = attr->user_data;
        LOG_INF("CW Characteristic discovered at handle %u", attr->handle);
        cw_handle[conn_count] = chrc->value_handle;

        conn_count++;
        if (conn_count < registered_rings) {
            start_scan();
        }
        return BT_GATT_ITER_STOP;
    }
    return BT_GATT_ITER_STOP;
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
