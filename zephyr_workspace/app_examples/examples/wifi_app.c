#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>

LOG_MODULE_REGISTER(wifi_app, LOG_LEVEL_INF);

#define WIFI_CONNECT_TIMEOUT_S 30
#define WIFI_MAX_RETRIES 5
#define WIFI_RETRY_DELAY_S 5

static K_SEM_DEFINE(wifi_connected_sem, 0, 1);
static K_SEM_DEFINE(wifi_disconnected_sem, 0, 1);

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static struct net_if* sta_iface;

/* ── WiFi event handler ──────────────────────────────────── */
static void wifi_event_handler(struct net_mgmt_event_callback* cb, uint64_t mgmt_event, struct net_if* iface) {
    if (mgmt_event == NET_EVENT_WIFI_CONNECT_RESULT) {
        const struct wifi_status* status = (const struct wifi_status*)cb->info;

        if (status && status->status == 0) {
            LOG_INF("WiFi connected");
            k_sem_give(&wifi_connected_sem);
        } else {
            LOG_ERR("WiFi connect failed (status=%d)", status ? status->status : -1);
            k_sem_give(&wifi_disconnected_sem);
        }

    } else if (mgmt_event == NET_EVENT_WIFI_DISCONNECT_RESULT) {
        LOG_WRN("WiFi disconnected");
        k_sem_give(&wifi_disconnected_sem);
    }
}

/* ── IPv4 / DHCP event handler ───────────────────────────── */
static void ipv4_event_handler(struct net_mgmt_event_callback* cb, uint64_t mgmt_event, struct net_if* iface) {
    if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
        return;
    }

    struct net_if_ipv4* ipv4_cfg = NULL;
    if (net_if_config_ipv4_get(iface, &ipv4_cfg) < 0 || !ipv4_cfg) {
        return;
    }

    char buf[NET_IPV4_ADDR_LEN];
    for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
        struct net_if_addr_ipv4* entry = &ipv4_cfg->unicast[i];
        if (entry->ipv4.is_added) {
            LOG_INF("DHCP IP: %s", net_addr_ntop(AF_INET, &entry->ipv4.address.in_addr, buf, sizeof(buf)));
            break;
        }
    }
}

/* ── Single connection attempt ───────────────────────────── */
static int wifi_try_connect(void) {
    /* Drain any leftover semaphore state from a previous attempt */
    k_sem_reset(&wifi_connected_sem);
    k_sem_reset(&wifi_disconnected_sem);

    struct wifi_connect_req_params params = {
        .ssid = (const uint8_t*)CONFIG_WIFI_SSID,
        .ssid_length = (uint8_t)(sizeof(CONFIG_WIFI_SSID) - 1),
        .psk = (const uint8_t*)CONFIG_WIFI_PSK,
        .psk_length = (uint8_t)(sizeof(CONFIG_WIFI_PSK) - 1),
        .sae_password = NULL,
        .sae_password_length = 0,
        .channel = WIFI_CHANNEL_ANY,
        .band = WIFI_FREQ_BAND_2_4_GHZ,
        .security = WIFI_SECURITY_TYPE_PSK,
        .mfp = WIFI_MFP_OPTIONAL,
    };

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta_iface, &params, sizeof(params));
    if (ret) {
        LOG_ERR("Connect request rejected (%d)", ret);
        return ret;
    }

    /* Wait for connect or explicit failure event */
    if (k_sem_take(&wifi_connected_sem, K_SECONDS(WIFI_CONNECT_TIMEOUT_S)) == 0) {
        return 0;
    }

    LOG_WRN("Attempt timed out");
    return -ETIMEDOUT;
}

/* ── Connect with retry ──────────────────────────────────── */
static int wifi_connect(void) {
    if (!sta_iface) {
        LOG_ERR("STA interface not available");
        return -ENODEV;
    }

    for (int attempt = 1; attempt <= WIFI_MAX_RETRIES; attempt++) {
        LOG_INF("Connecting to '%s' (attempt %d/%d)...", CONFIG_WIFI_SSID, attempt, WIFI_MAX_RETRIES);

        if (wifi_try_connect() == 0) {
            LOG_INF("Connected on attempt %d", attempt);
            return 0;
        }

        if (attempt < WIFI_MAX_RETRIES) {
            LOG_WRN("Attempt %d failed. Retrying in %ds...", attempt, WIFI_RETRY_DELAY_S);
            k_sleep(K_SECONDS(WIFI_RETRY_DELAY_S));
        }
    }

    LOG_ERR("All %d attempts failed. Giving up.", WIFI_MAX_RETRIES);
    return -ETIMEDOUT;
}

/* ── Status ──────────────────────────────────────────────── */
static void wifi_print_status(void) {
    struct wifi_iface_status status = {0};
    if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, sta_iface, &status, sizeof(status)) == 0) {
        LOG_INF("-- WiFi Status -------------------");
        LOG_INF("  SSID    : %s", status.ssid);
        LOG_INF("  Band    : %s", wifi_band_txt(status.band));
        LOG_INF("  Channel : %d", status.channel);
        LOG_INF("  RSSI    : %d dBm", status.rssi);
        LOG_INF("----------------------------------");
    }
}

/* ── Main ────────────────────────────────────────────────── */
int main(void) {
    LOG_INF("=== micro-ROS WiFi -- RAK3312 (ESP32-S3) ===");
    LOG_INF("SSID: %s", CONFIG_WIFI_SSID);

    net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler,
                                 NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_cb);

    net_mgmt_init_event_callback(&ipv4_cb, ipv4_event_handler, NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&ipv4_cb);

    /* Wait for ESP32 WiFi driver to self-initialise */
    k_sleep(K_SECONDS(5));

    sta_iface = net_if_get_wifi_sta();
    if (!sta_iface) {
        LOG_ERR("No STA interface -- check CONFIG_WIFI_ESP32=y");
        return -1;
    }

    if (wifi_connect() != 0) {
        LOG_ERR("Could not connect after %d attempts -- halting", WIFI_MAX_RETRIES);
        return -1;
    }

    wifi_print_status();
    LOG_INF("WiFi ready -- initialise micro-ROS transport here");

    /*
     * TODO: initialise micro-ROS here.
     *
     *   rmw_uros_set_custom_transport(
     *       MICRO_ROS_FRAMING_REQUIRED,
     *       (void *)&default_params,
     *       zephyr_transport_open,
     *       zephyr_transport_close,
     *       zephyr_transport_write,
     *       zephyr_transport_read);
     */

    while (true) {
        k_sem_take(&wifi_disconnected_sem, K_FOREVER);
        LOG_WRN("Disconnected. Reconnecting...");
        k_sleep(K_SECONDS(WIFI_RETRY_DELAY_S));
        wifi_connect();
        wifi_print_status();
    }

    return 0;
}