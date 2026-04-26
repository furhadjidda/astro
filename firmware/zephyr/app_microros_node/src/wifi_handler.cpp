#include "wifi_handler.hpp"

#include <errno.h>
#include <string.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/wifi_mgmt.h>
#endif

#include "oled_layout.hpp"

LOG_MODULE_DECLARE(all_sensors_module, LOG_LEVEL_DBG);

#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
namespace {
constexpr int WIFI_CONNECT_TIMEOUT_S = 30;
constexpr int WIFI_DHCP_TIMEOUT_S = 20;
constexpr int WIFI_MAX_RETRIES = 5;
constexpr int WIFI_RETRY_DELAY_S = 5;

K_SEM_DEFINE(ipv4_ready_sem, 0, 1);
static struct net_if* sta_iface;

void ipv4_event_handler(struct net_mgmt_event_callback* cb, uint64_t mgmt_event, struct net_if* iface) {
    ARG_UNUSED(cb);
    ARG_UNUSED(iface);

    if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
        return;
    }

    // Keep callback minimal to avoid work in net management context.
    k_sem_give(&ipv4_ready_sem);
}
}  // namespace
#endif

int WifiHandler::initStation(OLEDLayout& oled_layout) {
#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
    static struct net_mgmt_event_callback ipv4_cb;

    net_mgmt_init_event_callback(&ipv4_cb, ipv4_event_handler, NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&ipv4_cb);

    oled_layout.display_wifi_waiting_message(CONFIG_MICROROS_WIFI_SSID);
    oled_layout.finalize_screen();

    k_sleep(K_SECONDS(2));

    sta_iface = net_if_get_wifi_sta();
    if (!sta_iface) {
        LOG_ERR("No STA interface available. Enable WiFi board support.");
        return -ENODEV;
    }

    return connectWithRetry();
#else
    ARG_UNUSED(oled_layout);
    return -ENOTSUP;
#endif
}

int WifiHandler::connectWithRetry() {
#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
    if (!sta_iface) {
        return -ENODEV;
    }

    for (int attempt = 1; attempt <= WIFI_MAX_RETRIES; ++attempt) {
        LOG_INF("Connecting to WiFi SSID '%s' (%d/%d)", CONFIG_MICROROS_WIFI_SSID, attempt, WIFI_MAX_RETRIES);
        if (tryConnect() == 0) {
            return 0;
        }

        if (attempt < WIFI_MAX_RETRIES) {
            k_sleep(K_SECONDS(WIFI_RETRY_DELAY_S));
        }
    }

    return -ETIMEDOUT;
#else
    return -ENOTSUP;
#endif
}

int WifiHandler::tryConnect() {
#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
    k_sem_reset(&ipv4_ready_sem);

    struct wifi_connect_req_params params;
    memset(&params, 0, sizeof(params));
    params.ssid = reinterpret_cast<const uint8_t*>(CONFIG_MICROROS_WIFI_SSID);
    params.ssid_length = sizeof(CONFIG_MICROROS_WIFI_SSID) - 1;
    params.psk = reinterpret_cast<const uint8_t*>(CONFIG_MICROROS_WIFI_PASSWORD);
    params.psk_length = sizeof(CONFIG_MICROROS_WIFI_PASSWORD) - 1;
    params.sae_password = NULL;
    params.sae_password_length = 0;
    params.channel = WIFI_CHANNEL_ANY;
    params.band = WIFI_FREQ_BAND_2_4_GHZ;
    params.security = (params.psk_length > 0) ? WIFI_SECURITY_TYPE_PSK : WIFI_SECURITY_TYPE_NONE;
    params.mfp = WIFI_MFP_OPTIONAL;

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta_iface, &params, sizeof(params));
    if (ret) {
        LOG_ERR("Connect request rejected (%d)", ret);
        return ret;
    }

    if (k_sem_take(&ipv4_ready_sem, K_SECONDS(WIFI_CONNECT_TIMEOUT_S + WIFI_DHCP_TIMEOUT_S)) == 0) {
        LOG_INF("WiFi connected and DHCP lease acquired");
        return 0;
    }

    LOG_WRN("WiFi connect/DHCP timeout");
    (void)net_mgmt(NET_REQUEST_WIFI_DISCONNECT, sta_iface, NULL, 0);
    return -ETIMEDOUT;
#else
    return -ENOTSUP;
#endif
}
