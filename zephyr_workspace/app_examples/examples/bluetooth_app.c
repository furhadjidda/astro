/*
 * Bluetooth UART Service for RAK3312 (ESP32-S3)
 * Zephyr 4.3
 *
 * This application provides a Bluetooth Low Energy UART service
 * that allows a phone to connect and send text commands.
 */

#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_uart, LOG_LEVEL_DBG);

/* Nordic UART Service (NUS) UUIDs */
/* Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
#define BT_UUID_NUS_VAL BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

#define BT_UUID_NUS_SERVICE BT_UUID_DECLARE_128(BT_UUID_NUS_VAL)
#define BT_UUID_NUS_RX BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define BT_UUID_NUS_TX BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))

#define DEVICE_NAME "RAK3312_UART"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define COMMAND_BUFFER_SIZE 128

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static struct bt_conn* current_conn;
static bool notify_enabled;

/* Command buffer */
static char command_buffer[COMMAND_BUFFER_SIZE];
static uint16_t command_len = 0;

/* Function to process received commands */
static void process_command(const char* cmd, uint16_t len) {
    LOG_INF("Received command: %.*s", len, cmd);

    /* Add your command processing logic here */
    if (strncmp(cmd, "LED_ON", 6) == 0) {
        LOG_INF("Processing LED_ON command");
        /* Add your LED control code here */
        // gpio_pin_set_dt(&led, 1);

    } else if (strncmp(cmd, "LED_OFF", 7) == 0) {
        LOG_INF("Processing LED_OFF command");
        /* Add your LED control code here */
        // gpio_pin_set_dt(&led, 0);

    } else if (strncmp(cmd, "STATUS", 6) == 0) {
        LOG_INF("Processing STATUS command");
        /* Add your status reporting code here */

    } else if (strncmp(cmd, "HELP", 4) == 0) {
        LOG_INF("Processing HELP command");
        /* Send help information back to phone */

    } else {
        LOG_WRN("Unknown command: %.*s", len, cmd);
    }
}

/* GATT write callback for RX characteristic (data from phone) */
static ssize_t on_receive(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buf, uint16_t len,
                          uint16_t offset, uint8_t flags) {
    const char* data = (const char*)buf;

    LOG_DBG("Received data, len %u, offset %u", len, offset);

    /* Accumulate data in buffer until newline or buffer full */
    for (uint16_t i = 0; i < len; i++) {
        if (data[i] == '\n' || data[i] == '\r') {
            if (command_len > 0) {
                command_buffer[command_len] = '\0';
                process_command(command_buffer, command_len);
                command_len = 0;
            }
        } else if (command_len < COMMAND_BUFFER_SIZE - 1) {
            command_buffer[command_len++] = data[i];
        } else {
            LOG_WRN("Command buffer overflow, resetting");
            command_len = 0;
        }
    }

    return len;
}

/* GATT CCC (Client Characteristic Configuration) changed callback */
static void on_cccd_changed(const struct bt_gatt_attr* attr, uint16_t value) {
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s", notify_enabled ? "enabled" : "disabled");
}

/* GATT Service Definition */
BT_GATT_SERVICE_DEFINE(nus_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_NUS_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_NUS_TX, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
                       BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_NUS_RX, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_WRITE, NULL, on_receive, NULL), );

/* Function to send data to phone */
int bt_send_data(const char* data, uint16_t len) {
    if (!current_conn) {
        LOG_WRN("No connection");
        return -ENOTCONN;
    }

    if (!notify_enabled) {
        LOG_WRN("Notifications not enabled");
        return -EACCES;
    }

    return bt_gatt_notify(current_conn, &nus_svc.attrs[1], data, len);
}

/* Connection callbacks */
static void connected(struct bt_conn* conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    LOG_INF("Connected");

    /* Reset command buffer on new connection */
    command_len = 0;
}

static void disconnected(struct bt_conn* conn, uint8_t reason) {
    LOG_INF("Disconnected (reason %u)", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    notify_enabled = false;
    command_len = 0;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Scan response data - includes the 128-bit service UUID */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01,
                  0x00, 0x40, 0x6e),
};

/* Start advertising */
static void start_advertising(void) {
    int err;

    struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .options = BT_LE_ADV_OPT_CONN,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
        .peer = NULL,
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");
}

/* Bluetooth ready callback */
static void bt_ready(int err) {
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");
    k_sem_give(&ble_init_ok);
}

int main(void) {
    int err;

    LOG_INF("Starting Bluetooth UART service on RAK3312");

    /* Initialize Bluetooth */
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    /* Wait for Bluetooth to be ready */
    k_sem_take(&ble_init_ok, K_FOREVER);

    /* Start advertising */
    start_advertising();

    LOG_INF("System ready. Waiting for connections...");

    /* Main loop - you can add periodic tasks here */
    while (1) {
        k_sleep(K_SECONDS(1));

        /* Example: Send periodic status updates */
        if (current_conn && notify_enabled) {
            // Uncomment to send periodic messages
            // bt_send_data("STATUS: OK\n", 11);
        }
    }

    return 0;
}