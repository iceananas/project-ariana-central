#include "main.h"
#include "ft5xx6.h"

#define LOG_LEVEL 3
LOG_MODULE_REGISTER(main);
#define SLEEP_TIME_MS 5
#define RINGS_ADDRESS "C3:34:B3:E9:AD:16"

// Logic variables
static bool connection_successful = false;
static uint8_t brightness_value = 0;
static uint8_t ww_value = 0;
static uint8_t cw_value = 0;

// Config and functions for CTPM
#define I2C DT_NODELABEL(i2c0)
#define INT_PIN DT_NODELABEL(ctpmint)
static const struct gpio_dt_spec ctpm_int = GPIO_DT_SPEC_GET(INT_PIN, gpios);
static struct gpio_callback ctpm_int_cb_data;
static bool ctpm_event_flag = false;
static struct point coordinates = {.x = 0, .y = 0};
static const struct device *ctpm_dev;
void on_interrupt_received(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ww_value = coordinates.x / 2;
    cw_value = coordinates.y / 2;
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

    while (!connection_successful) {
        k_msleep(100);
    }

    while (1) {
        if (ctpm_event_flag) {
            coordinates = ft_5xx6_get_coordinates(ctpm_dev);

            err = bt_gatt_write_without_response(default_conn, ww_handle, &ww_value, 1, false);
            if (err) {
                LOG_ERR("Write data failed (err %d)", err);
            }
            err = bt_gatt_write_without_response(default_conn, cw_handle, &cw_value, 1, false);
            if (err) {
                LOG_ERR("Write data failed (err %d)", err);
            }
            ctpm_event_flag = false;
        }
        k_msleep(SLEEP_TIME_MS);
    }
}

// Start scan
static void start_scan(void) {
    int err;

    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, on_device_found);
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

    if (default_conn) {
        return;
    }

    /* We're only interested in connectable events */
    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("Device found: %s (RSSI %d)", log_strdup(addr_str), rssi);

    if (!strncmp(addr_str, RINGS_ADDRESS, 17)) {
        LOG_INF("Ring found, trying to connect...");
        if (bt_le_scan_stop()) {
            return;
        }

        err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
                                &default_conn);

        if (err) {
            LOG_ERR("Create conn to %s failed (%u)\n", log_strdup(addr_str), err);
        }
    }
}

// Handler after sucessful connection
static void on_connect(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        LOG_ERR("Failed to connect to %s (%u)", log_strdup(addr), err);

        bt_conn_unref(default_conn);
        default_conn = NULL;

        start_scan();
        return;
    }

    if (conn != default_conn) {
        return;
    }
    LOG_INF("Connected: %s", log_strdup(addr));

    discover_params.uuid = &light_control_uuid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(default_conn, &discover_params);
    if (err) {
        LOG_ERR("Discover failed(err %d)", err);
        return;
    }

    connection_successful = true;
}

// Disconnect
static void on_disconnect(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    if (conn != default_conn) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_ERR("Disconnected: %s (reason 0x%02x)\n", log_strdup(addr), reason);

    bt_conn_unref(default_conn);
    default_conn = NULL;
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

        ww_handle = chrc->value_handle;
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
        cw_handle = chrc->value_handle;
        return BT_GATT_ITER_STOP;
    }
    return BT_GATT_ITER_STOP;
}
