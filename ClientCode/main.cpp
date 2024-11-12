#include <Arduino.h>
#include <SPI.h>
#include "DW1000Ranging.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <stdio.h>
#include <HTTPClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_sntp.h"
#include "time.h"
#include "sys/time.h"
#include "esp_mac.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include <esp_timer.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include <cJSON.h>
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include <vector>

char TAG_ADDR[18];
uint8_t mac[6];
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define UWB_RST 27 // reset pin
#define UWB_IRQ 34 // irq pin
#define UWB_SS 21  // spi select pin

#define I2C_SDA 4
#define I2C_SCL 5

#define GPIO_39 39
#define GPIO_36 36
#define GPIO_22 22
#define GPIO_17 17

#define TIME_SERVER_AP "http://192.168.4.1/get-time"

struct Timestamp
{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int millisecond;
    int microsecond;
};

bool OnSendData = false;
Timestamp rtc;
bool timeSyncStatus = false;
bool wifiStatus = false;
bool uwbStatus = false;
bool rssi_OK = false;
bool post_OK = false;
bool uwb_OK = false;
float rec_csi = 0;
int TagNo = 0;
int stepOLED = 0;

uint32_t runningNumber = 1;
int value_A = 0;
int value_B = 0;
int value_C = 0;
int value_D = 0;
int counterOledTime = 0;

static void sync_time_with_ap();
unsigned long runtime = 0;
void print_current_time();
Adafruit_SSD1306 display(128, 64, &Wire, -1);
void printTime();
void get_wifi_info(char *ssid, uint8_t *ap_mac, int8_t *rssi, uint8_t *channel);
void get_ip_addresses(char *ip, char *ap_ip);
void scan_wifi(cJSON *networkNamesArray, cJSON *networkMacArray, cJSON *networkRssiArray, cJSON *networkChannelArray);
void add_csi_to_json(cJSON *informationWiFi);
uint32_t load_running_number();
void save_running_number(uint32_t number);
void display_uwb(struct Link *p);
void handleAnchorData();
void printRangeData(uint16_t currentAddress, float rxPower, float fpPower, float quality, float totalDistance, int64_t tof, float tofDist);
void printAnchorData();
int status_server = 0;
extern "C" void app_main();

//-------------------------------WiFi APSTA CSI--------------------------------
#define CONFIG_STA_WIFI_SSID "AIT-TCB"
#define CONFIG_STA_WIFI_PASSWORD "Ait@2024"
#define CONFIG_STA_CONNECT_TIMEOUT 10

#define CONFIG_WIFI_CONNECT_AP 0
#define CONFIG_WIFI_CONNECT_STA 1
#define CONFIG_WIFI_CONNECT_APSTA 0

#define CONFIG_LESS_INTERFERENCE_CHANNEL 0
#define CONFIG_SEND_FREQUENCY 100
#define CSI_DATA_SIZE 256

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

typedef struct
{
    uint8_t mac[6];             // MAC Address
    int8_t rssi;                // Received Signal Strength Indicator
    uint8_t rate;               // Data rate
    uint8_t sig_mode;           // Signal mode (HT or non-HT)
    uint8_t mcs;                // Modulation and Coding Scheme index
    uint8_t bandwidth;          // Channel Bandwidth (20MHz or 40MHz)
    uint8_t smoothing;          // Smoothing
    uint8_t not_sounding;       // Not sounding
    uint8_t aggregation;        // Aggregation
    uint8_t stbc;               // Space-Time Block Coding
    uint8_t fec_coding;         // Forward Error Correction coding
    uint8_t sgi;                // Short Guard Interval
    int8_t noise_floor;         // Noise floor
    uint8_t ampdu_cnt;          // AMPDU count
    uint8_t channel;            // Wi-Fi channel
    uint8_t secondary_channel;  // Secondary Channel
    uint32_t timestamp;         // Timestamp
    uint8_t ant;                // Antenna
    uint16_t sig_len;           // Signal length
    uint8_t rx_state;           // RX state
    uint16_t len;               // CSI buffer length
    uint8_t first_word_invalid; // First word invalid flag
    int8_t *buf;                // CSI buffer
} wifi_csi_data_t;

wifi_csi_data_t csi_data;
wifi_csi_data_t csi_data_snap;
void reset_csi_data(wifi_csi_data_t *csi_data);

typedef struct
{
    uint8_t shortAddress;
    float distance;
    float rxPower;
    uint32_t txTimestamp;
    uint32_t rxTimestamp;
    uint32_t tof;
    float range;
} UWBData_t;

UWBData_t uwb_data_set;
#define NUMBER_OF_DISTANCES 1
int64_t TIME_OVERFLOW = 0x10000000000; // 1099511627776LL
// Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
float DISTANCE_OF_RADIO = 0.0046917639786159f;
//--------------------------------------UWB------------------------------------------

#define MAX_ANCHORS 4
TaskHandle_t dw1000TaskHandle = NULL;

struct AnchorData
{
    uint16_t shortAddress;
    uint8_t channel;
    float rxPower;
    float fpPower;
    float quality;
    float range;
    int64_t tof;
    float tofDist;
    char *macAddress;
};

struct AnchorMapping
{
    uint16_t shortAddress;
    uint8_t channel;
    const char *macAddress;
};

AnchorMapping anchorMappings[] = {
    {0x73D9, 4, "D0:EF:76:35:84:B1"},
    {0x848D, 2, "D0:EF:76:35:84:8D"},
    {0x84B1, 1, "D0:EF:76:35:84:B1"},
    {0xC471, 3, "EC:64:C9:BF:C4:71"}};

uint8_t getChannelForAddress(uint16_t shortAddress)
{
    for (const auto &mapping : anchorMappings)
    {
        if (mapping.shortAddress == shortAddress)
        {
            return mapping.channel;
        }
    }
    return 0;
}

const char *getMacForAddress(uint16_t shortAddress)
{
    for (const auto &mapping : anchorMappings)
    {
        if (mapping.shortAddress == shortAddress)
        {
            return mapping.macAddress;
        }
    }
    return nullptr;
}

std::vector<AnchorData> uwbRealTimeDataList;
std::vector<AnchorData> anchorDataList;

const uint16_t requiredAnchors[] = {0x73D9, 0x848D, 0x84B1, 0xC471};
const size_t numRequiredAnchors = sizeof(requiredAnchors) / sizeof(requiredAnchors[0]);

void newRange()
{
    handleAnchorData();
}

void handleAnchorData()
{
    uint16_t currentAddress = DW1000Ranging.getDistantDevice()->getShortAddress();
    float rxPower = DW1000Ranging.getDistantDevice()->getRXPower();
    float fpPower = DW1000Ranging.getDistantDevice()->getFPPower();
    float quality = DW1000Ranging.getDistantDevice()->getQuality();
    float totalDistance = 0.0;

    for (int i = 0; i < NUMBER_OF_DISTANCES; i++)
    {
        totalDistance += DW1000Ranging.getDistantDevice()->getRange();
    }
    totalDistance /= NUMBER_OF_DISTANCES;

    int64_t tof = DW1000Ranging.getDistantDevice()->getTof();
    float tofDist = (tof % TIME_OVERFLOW) * DISTANCE_OF_RADIO;
    if (totalDistance > 0 && tof > 0 && abs(totalDistance - tofDist) <= 0.05)
    {
        bool addressFound = false;
        for (auto &anchor : anchorDataList)
        {
            if (anchor.shortAddress == currentAddress)
            {
                anchor.rxPower = rxPower;
                anchor.fpPower = fpPower;
                anchor.quality = quality;
                anchor.range = totalDistance;
                anchor.tof = tof;
                anchor.tofDist = tofDist;
                anchor.channel = getChannelForAddress(currentAddress);
                anchor.macAddress = strdup(getMacForAddress(currentAddress));
                addressFound = true;
                break;
            }
        }

        if (!addressFound)
        {
            AnchorData newAnchor;
            newAnchor.shortAddress = currentAddress;
            newAnchor.channel = getChannelForAddress(currentAddress);
            newAnchor.macAddress = strdup(getMacForAddress(currentAddress));
            newAnchor.rxPower = rxPower;
            newAnchor.fpPower = fpPower;
            newAnchor.quality = quality;
            newAnchor.range = totalDistance;
            newAnchor.tof = tof;
            newAnchor.tofDist = tofDist;
            anchorDataList.push_back(newAnchor);
        }

        if (anchorDataList.size() == numRequiredAnchors && stepOLED == 1)
        {
            Serial.println("Received data from all anchors");
            uwbRealTimeDataList = anchorDataList;
            uwb_OK = true;
        }

        printRangeData(currentAddress, rxPower, fpPower, quality, totalDistance, tof, tofDist);
    }
    // printRangeData(currentAddress, rxPower, fpPower, quality, totalDistance, tof, tofDist);
}

void printAnchorData()
{
    Serial.println("Anchor Data:");
    Serial.println("Short Address | Channel | RX Power | FP Power | Quality | Distance | ToF | ToF Distance");
    Serial.println("------------------------------------------------------------------------------");

    for (const auto &anchor : anchorDataList)
    {
        Serial.print("0x");
        Serial.print(anchor.shortAddress, HEX);
        Serial.print("       | ");
        Serial.print(anchor.channel);
        Serial.print(" | ");
        Serial.print(anchor.rxPower);
        Serial.print(" | ");
        Serial.print(anchor.fpPower);
        Serial.print(" | ");
        Serial.print(anchor.quality);
        Serial.print(" | ");
        Serial.print(anchor.range);
        Serial.print(" | ");
        Serial.print(anchor.tof);
        Serial.print(" | ");
        Serial.print(anchor.tofDist);
        Serial.println(" |");
    }

    Serial.println("------------------------------------------------------------------------------");
}

void printRangeData(uint16_t currentAddress, float rxPower, float fpPower, float quality, float totalDistance, int64_t tof, float tofDist)
{
    Serial.print("from: ");
    Serial.print(currentAddress, HEX);
    /*      Serial.print(", RX Power: ");
        Serial.print(rxPower);
        Serial.print(", FP Power: ");
        Serial.print(fpPower);
        Serial.print(", Quality: ");
        Serial.print(quality); */
    Serial.print(", Distance: ");
    Serial.print(totalDistance);
    Serial.print(" m, ToF: ");
    Serial.print(tof);
    Serial.print(" us, ToF_dis: ");
    Serial.print(tofDist);
    Serial.println(" m");
}

void newDevice(DW1000Device *device)
{
    Serial.print("Device added: ");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}

void logoshow(void)
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("ESP32"));
    display.println(F("Data"));
    display.println(F("Collect"));

    display.setTextSize(1);
    display.println(F("UWB & CSI Monitoring"));
    display.display();
    delay(500);
}

//------------------------------WiFi--------------------------------

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        Serial.println("WIFI_EVENT_STA_DISCONNECTED");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        timeSyncStatus = false;
        wifiStatus = false;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        wifiStatus = true;
        Serial.println("IP_EVENT_STA_GOT_IP");

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        Serial.print("IP Address: ");
        Serial.println(IPAddress(event->ip_info.ip.addr).toString());

        Serial.print("Gateway: ");
        Serial.println(IPAddress(event->ip_info.gw.addr).toString());
    }
}

static void initialise_wifi(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    static bool initialized = false;
    if (initialized)
    {
        return;
    }
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    initialized = true;
}

#if CONFIG_WIFI_CONNECT_STA
static bool wifi_sta(int timeout_ms)
{
    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, CONFIG_STA_WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, CONFIG_STA_WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());

    int bits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                                   pdFALSE, pdTRUE, timeout_ms / portTICK_PERIOD_MS);

    Serial.printf("bits=%x\n", bits);
    if (bits)
    {
        Serial.printf("WIFI_MODE_STA connected. SSID:%s password:%s\n",
                      CONFIG_STA_WIFI_SSID, CONFIG_STA_WIFI_PASSWORD);
    }
    else
    {
        Serial.printf("WIFI_MODE_STA can't connect. SSID:%s password:%s\n",
                      CONFIG_STA_WIFI_SSID, CONFIG_STA_WIFI_PASSWORD);
    }
    return (bits & CONNECTED_BIT) != 0;
}
#endif

void get_wifi_info(char *ssid, uint8_t *ap_mac, int8_t *rssi, uint8_t *channel)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
        strncpy(ssid, (const char *)ap_info.ssid, 32);
        ssid[31] = '\0';
        memcpy(ap_mac, ap_info.bssid, 6);
        *rssi = ap_info.rssi;
        *channel = ap_info.primary;
    }
    else
    {
        printf("Failed to get WiFi AP info\n");
    }
}

void get_ip_addresses(char *ip, char *ap_ip)
{
    esp_netif_ip_info_t ip_info;

    // Get local IP
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
    {
        snprintf(ip, 16, IPSTR, IP2STR(&ip_info.ip));
    }
    else
    {
        printf("Failed to get local IP\n");
    }

    // Get AP IP (by assuming it's the gateway)
    snprintf(ap_ip, 16, IPSTR, IP2STR(&ip_info.gw));
}

static void print_csi_data(const wifi_csi_data_t *csi_data)
{
    Serial.println("================ CSI RECV ================");
    Serial.println("MAC Address: ");
    for (int i = 0; i < 6; i++)
    {
        Serial.print(csi_data->mac[i], HEX);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();

    Serial.print("RSSI: ");
    Serial.println(csi_data->rssi);

    Serial.print("Rate: ");
    Serial.println(csi_data->rate);

    Serial.print("Signal Mode: ");
    Serial.println(csi_data->sig_mode);

    Serial.print("MCS: ");
    Serial.println(csi_data->mcs);

    Serial.print("Bandwidth: ");
    Serial.println(csi_data->bandwidth);

    Serial.print("Smoothing: ");
    Serial.println(csi_data->smoothing);

    Serial.print("Not Sounding: ");
    Serial.println(csi_data->not_sounding);

    Serial.print("Aggregation: ");
    Serial.println(csi_data->aggregation);

    Serial.print("STBC: ");
    Serial.println(csi_data->stbc);

    Serial.print("FEC Coding: ");
    Serial.println(csi_data->fec_coding);

    Serial.print("SGI: ");
    Serial.println(csi_data->sgi);

    Serial.print("Noise Floor: ");
    Serial.println(csi_data->noise_floor);

    Serial.print("AMPDU Count: ");
    Serial.println(csi_data->ampdu_cnt);

    Serial.print("Channel: ");
    Serial.println(csi_data->channel);

    Serial.print("Secondary Channel: ");
    Serial.println(csi_data->secondary_channel);

    Serial.print("Timestamp: ");
    Serial.println(csi_data->timestamp);

    Serial.print("Antenna: ");
    Serial.println(csi_data->ant);

    Serial.print("Signal Length: ");
    Serial.println(csi_data->sig_len);

    Serial.print("RX State: ");
    Serial.println(csi_data->rx_state);

    Serial.print("CSI Length: ");
    Serial.println(csi_data->len);

    Serial.print("First Word Invalid: ");
    Serial.println(csi_data->first_word_invalid);

    Serial.print("CSI Buffer: [");
    if (csi_data->len > 0)
    {
        Serial.print(csi_data->buf[0]);
        for (int i = 1; i < csi_data->len; i++)
        {
            Serial.print(", ");
            Serial.print(csi_data->buf[i]);
        }
    }
    Serial.println("]");
}

// static const uint8_t CONFIG_CSI_SEND_MAC[] = {0xd0, 0xef, 0x76, 0x35, 0x73, 0xd8};

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf)
    {
        Serial.println("Invalid CSI info");
        return;
    }

    /* if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6))
    {
        return;
    } */

    memcpy(csi_data.mac, info->mac, 6);
    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    csi_data.rssi = rx_ctrl->rssi;
    csi_data.rate = rx_ctrl->rate;
    csi_data.sig_mode = rx_ctrl->sig_mode;
    csi_data.mcs = rx_ctrl->mcs;
    csi_data.bandwidth = rx_ctrl->cwb;
    csi_data.smoothing = rx_ctrl->smoothing;
    csi_data.not_sounding = rx_ctrl->not_sounding;
    csi_data.aggregation = rx_ctrl->aggregation;
    csi_data.stbc = rx_ctrl->stbc;
    csi_data.fec_coding = rx_ctrl->fec_coding;
    csi_data.sgi = rx_ctrl->sgi;
    csi_data.noise_floor = rx_ctrl->noise_floor;
    csi_data.ampdu_cnt = rx_ctrl->ampdu_cnt;
    csi_data.channel = rx_ctrl->channel;
    csi_data.secondary_channel = rx_ctrl->secondary_channel;
    csi_data.timestamp = rx_ctrl->timestamp;
    csi_data.ant = rx_ctrl->ant;
    csi_data.sig_len = rx_ctrl->sig_len;
    csi_data.rx_state = rx_ctrl->rx_state;
    csi_data.len = info->len;
    csi_data.first_word_invalid = info->first_word_invalid;

    csi_data.buf = (int8_t *)malloc(csi_data.len * sizeof(int8_t));
    if (csi_data.buf)
    {
        memcpy(csi_data.buf, info->buf, csi_data.len);
    }

    // Print the CSI data using the print_csi_data function
    //print_csi_data(&csi_data);

    free(csi_data.buf);
    rec_csi = false;
    rssi_OK = true;
}

static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(g_wifi_radar_config->wifi_sniffer_cb));

    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = true,
        .manu_scale = false,
        .shift = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

esp_err_t client_event_get_time_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        static char time_str[64];
        if (evt->data_len < sizeof(time_str))
        {
            memcpy(time_str, evt->data, evt->data_len);
            time_str[evt->data_len] = '\0';
            printf("HTTP_EVENT_ON_DATA: %s\n", time_str);
            struct tm tm;
            if (strptime(time_str, "%Y-%m-%d %H:%M:%S", &tm) != NULL)
            {
                time_t t = mktime(&tm);
                struct timeval now = {.tv_sec = t, .tv_usec = 0};
                settimeofday(&now, NULL);
                printf("Local time updated\n");
                timeSyncStatus = true;
            }
            else
            {
                printf("Failed to parse time\n");
            }
        }
        else
        {
            printf("Received data too long\n");
        }
        break;

    default:
        break;
    }
    return ESP_OK;
}

esp_err_t client_event_get_status_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        static char status_str[64];
        if (evt->data_len < sizeof(status_str))
        {
            memcpy(status_str, evt->data, evt->data_len);
            status_str[evt->data_len] = '\0';
            status_server = atoi(status_str);
            printf("HTTP_EVENT_ON_DATA: %s\n", status_str);
        }
        else
        {
            printf("Received data too long\n");
        }
        break;

    default:
        break;
    }
    return ESP_OK;
}

static void sync_time_with_ap()
{
    esp_http_client_config_t config_get = {
        .url = "http://192.168.4.1/get-time",
        .method = HTTP_METHOD_GET,
        .event_handler = client_event_get_time_handler};
    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

static void check_status_server()
{
    esp_http_client_config_t config_get = {
        .url = "http://192.168.4.1/get-status-server",
        .method = HTTP_METHOD_GET,
        .event_handler = client_event_get_status_handler};
    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

void print_current_time()
{
    Serial.printf("T: %04d%02d%02d %02d:%02d:%02d:%06d\n",
                  rtc.year, rtc.month, rtc.day,
                  rtc.hour, rtc.minute, rtc.second,
                  rtc.microsecond);
}
static void RTC(void *pvParameter)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int64_t start_time_sec = now.tv_sec;
    int64_t start_time_us = esp_timer_get_time();

    Serial.printf("Tracking time started at: %lld sec, %lld us\n", start_time_sec, start_time_us);

    while (true)
    {
        if (timeSyncStatus == false)
        {
            sync_time_with_ap();
        }

        int64_t current_time_us = esp_timer_get_time();
        int64_t elapsed_time_ms = (current_time_us - start_time_us) / 1000;
        int64_t elapsed_time_us = current_time_us - start_time_us;
        time_t now;
        time(&now);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        timeinfo.tm_hour += 7;
        if (timeinfo.tm_hour >= 24)
        {
            timeinfo.tm_hour -= 24;
            timeinfo.tm_mday += 1;
        }
        char datetime_buf[32];
        strftime(datetime_buf, sizeof(datetime_buf), "%Y%m%d %H:%M:%S", &timeinfo);
        int milliseconds = (elapsed_time_us % 1000000) / 1000;

        rtc.year = timeinfo.tm_year + 1900;
        rtc.month = timeinfo.tm_mon + 1;
        rtc.day = timeinfo.tm_mday;
        rtc.hour = timeinfo.tm_hour;
        rtc.minute = timeinfo.tm_min;
        rtc.second = timeinfo.tm_sec;

        rtc.microsecond = current_time_us % 1000000;
        rtc.millisecond = rtc.microsecond / 1000;
        print_current_time();

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    static u_int16_t i = 0;
    u_int16_t y = 0;
    char *ptr;

    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        Serial.println("HTTP_EVENT_ERROR");
        post_OK = false;
        break;
    case HTTP_EVENT_ON_CONNECTED:
        Serial.println("HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        Serial.println("HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        Serial.printf("HTTP_EVENT_ON_HEADER, key=%s, value=%s\n", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        Serial.printf("HTTP_EVENT_ON_DATA, len=%d\n", evt->data_len);
        if (evt->data_len > 0)
        {
            // Print the received data
            Serial.printf("Data: %.*s\n", evt->data_len, (char *)evt->data);
        }
        post_OK = true;
        break;
    case HTTP_EVENT_ON_FINISH:
        Serial.println("HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        Serial.println("HTTP_EVENT_DISCONNECTED");
        break;
    default:
        break;
    }
    return ESP_OK;
}

void post_json(char *json_string)
{
    esp_http_client_config_t config = {
        .url = "http://192.168.4.1/post-json",
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .event_handler = client_event_post_handler,
        .if_name = NULL,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_err_t err = esp_http_client_set_post_field(client, json_string, strlen(json_string));
    err = esp_http_client_perform(client);

    esp_http_client_cleanup(client);
}

void displayData(const wifi_csi_data_t *csi_data)
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("======Last Send=====");
    display.print("rssi(dbm): ");
    display.println(csi_data->rssi);
    display.println("======UWB Data======");
    display.println("Ch  RxPwr ToF  Dis");

    for (const auto &anchor : uwbRealTimeDataList)
    {
        display.print(anchor.channel);
        display.print(" ");
        display.print(anchor.rxPower, 2);
        display.print(" ");
        display.print(anchor.tof);
        display.print(" ");
        display.print(anchor.range);
        display.println();
    }

    display.display();
    delay(10);
}

void displayHomePage()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("Device Status");

    display.print("TAG: ");
    display.println(TagNo);

    display.print("WiFi: ");
    display.println(wifiStatus ? "Connected" : "Disconnected");

    display.print("UWB: ");
    display.println(uwbStatus ? "OK" : "Error");

    display.print("Time Sync: ");
    display.println(timeSyncStatus ? "Sync" : "Not Sync");

    display.printf("Date: %02d/%02d/%04d\n", rtc.day, rtc.month, rtc.year);
    display.printf("Time: %02d:%02d:%02d\n", rtc.hour, rtc.minute, rtc.second);

    display.display();
}

void convertDataUWB(cJSON *macDeviceArray, cJSON *addrArray, cJSON *channelArray, cJSON *rxPowerArray, cJSON *fpPowerArray, cJSON *qualityArray, cJSON *rangeArray, cJSON *tofArray, cJSON *tofDisArray)
{
    for (const auto &anchor : anchorDataList)
    {
        char addr_str[18];

        snprintf(addr_str, sizeof(addr_str), "%04X", anchor.shortAddress);
        cJSON_AddItemToArray(addrArray, cJSON_CreateString((const char *)addr_str));
        cJSON_AddItemToArray(macDeviceArray, cJSON_CreateString((const char *)anchor.macAddress));
        cJSON_AddItemToArray(channelArray, cJSON_CreateNumber(anchor.channel));
        cJSON_AddItemToArray(rxPowerArray, cJSON_CreateNumber(anchor.rxPower));
        cJSON_AddItemToArray(fpPowerArray, cJSON_CreateNumber(anchor.fpPower));
        cJSON_AddItemToArray(qualityArray, cJSON_CreateNumber(anchor.quality));
        cJSON_AddItemToArray(rangeArray, cJSON_CreateNumber(anchor.range));
        cJSON_AddItemToArray(tofArray, cJSON_CreateNumber(anchor.tof));
        cJSON_AddItemToArray(tofDisArray, cJSON_CreateNumber(anchor.tofDist));
    }
}

static void Oled(void *pvParameter)
{
    uint64_t chipid = ESP.getEfuseMac();
    uint32_t deviceID = (uint32_t)(chipid >> 24);
    int displayCounter = 0;
    bool page = false;
    int timeOut = 0;

    while (1)
    {
        value_A = digitalRead(GPIO_39);
        value_B = digitalRead(GPIO_36);
        value_C = digitalRead(GPIO_22);

        if (value_A || value_B || value_C)
        {
            OnSendData = true;
        }

        if ((millis() - runtime) > 1000)
        {
            if (stepOLED == 1)
                timeOut += 1;

            counterOledTime += 1;
            runtime = millis();
        }

        if (OnSendData == true)
        {
            if (stepOLED == 0) // clear Data
            {
                sync_time_with_ap();
                display.clearDisplay();
                anchorDataList.clear();
                stepOLED = 1;
                rssi_OK = false;
                uwb_OK = false;
                timeOut = 0;
            }
            else if (stepOLED == 1) // Wait data
            {

                display.clearDisplay();
                display.setCursor(0, 0);
                display.setTextSize(2);
                display.printf("Device\n   Working\n");
                for (int i = 0; i < (displayCounter % 10); i++)
                {
                    display.print(".");
                }
                display.display();
                if (rssi_OK && uwb_OK)
                {
                    csi_data_snap = csi_data;
                    stepOLED = 2;
                    timeOut = 0;
                }

                if (timeOut >= 60)
                {
                    Serial.println("Time out");
                    csi_data_snap = csi_data;
                    uwbRealTimeDataList = anchorDataList;
                    stepOLED = 2;
                    timeOut = 0;
                }

                displayCounter++;
            }
            if (stepOLED == 2)
            {
                /* printAnchorData();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                print_csi_data(&csi_data);
                vTaskDelay(100 / portTICK_PERIOD_MS); */
                display.clearDisplay();
                display.setCursor(0, 0);
                display.setTextSize(2);
                display.printf("Data\nComplete\n");
                display.display();
                vTaskDelay(300 / portTICK_PERIOD_MS);
                stepOLED = 3;
                displayCounter = 0;
                display.clearDisplay();
                display.setCursor(0, 0);
                display.setTextSize(1);
                display.setCursor(0, 0);
                display.print("TAG: ");
                display.print(TagNo);
                display.print(" SQ: ");
                display.println(runningNumber);
                display.print("RSSI(dbm): ");
                display.println(csi_data.rssi);
                for (const auto &anchor : uwbRealTimeDataList)
                {
                    display.print(anchor.channel);
                    display.print(",");
                    display.print(anchor.rxPower);
                    display.print(",");
                    display.print(anchor.tof);
                    display.print(",");
                    display.print(anchor.range);
                    display.println();
                }
                display.display();
            }
            else if (stepOLED == 3)
            {
                if (displayCounter >= 60)
                {
                    stepOLED = 4;
                    displayCounter = 0;
                }
                displayCounter++;
            }
            else if (stepOLED == 4)
            {

                check_status_server();
                if (status_server == 200)
                {
                    char deviceIDStr[20];
                    snprintf(deviceIDStr, sizeof(deviceIDStr), "%lu", deviceID);

                    char sessionID[70];
                    snprintf(sessionID, sizeof(sessionID), "%04d%02d%02d%02d%02d%02d10%08lu%05lu", rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second, deviceID, runningNumber);

                    char date[16];
                    snprintf(date, sizeof(date), "%04d-%02d-%02d", rtc.year, rtc.month, rtc.day);

                    char time[24];
                    snprintf(time, sizeof(time), "%02d:%02d:%02d.%06d", rtc.hour, rtc.minute, rtc.second, rtc.microsecond);

                    char ssid[33];
                    uint8_t ap_mac[6];
                    int8_t rssi;
                    uint8_t channel;
                    get_wifi_info(ssid, ap_mac, &rssi, &channel);
                    char mac_str[18];
                    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", ap_mac[0], ap_mac[1], ap_mac[2], ap_mac[3], ap_mac[4], ap_mac[5]);
                    char ip[16], ap_ip[16];
                    get_ip_addresses(ip, ap_ip);

                    // Create JSON object
                    cJSON *root = cJSON_CreateObject();

                    // Packet ID
                    cJSON *packetId = cJSON_CreateObject();
                    cJSON_AddStringToObject(packetId, "sessionID", sessionID);
                    cJSON_AddItemToObject(root, "packetid", packetId);

                    // Information Session
                    cJSON *informationSession = cJSON_CreateObject();
                    cJSON_AddStringToObject(informationSession, "sessionID", sessionID);
                    cJSON_AddStringToObject(informationSession, "datestamp", date);
                    cJSON_AddStringToObject(informationSession, "timestamp", time);
                    cJSON_AddStringToObject(informationSession, "deviceID", deviceIDStr);
                    cJSON_AddNumberToObject(informationSession, "sequence", runningNumber);
                    cJSON_AddBoolToObject(informationSession, "timesync", timeSyncStatus);
                    cJSON_AddItemToObject(root, "information.session", informationSession);

                    // Information WiFi
                    cJSON *informationWiFi = cJSON_CreateObject();
                    cJSON_AddStringToObject(informationWiFi, "ssid", ssid);
                    cJSON_AddStringToObject(informationWiFi, "apMacAddress", mac_str);
                    cJSON_AddNumberToObject(informationWiFi, "channel", channel);
                    cJSON_AddStringToObject(informationWiFi, "ipAddress", ip);
                    cJSON_AddStringToObject(informationWiFi, "apIpAddress", ap_ip);
                    cJSON_AddNumberToObject(informationWiFi, "rssi_dBm", csi_data_snap.rssi);
                    cJSON_AddNumberToObject(informationWiFi, "noise_dBm", csi_data_snap.noise_floor);
                    cJSON_AddNumberToObject(informationWiFi, "snr", (csi_data_snap.rssi - csi_data_snap.noise_floor));
                    add_csi_to_json(informationWiFi);
                    cJSON_AddItemToObject(root, "information.wifi", informationWiFi);

                    cJSON *networkScan = cJSON_CreateObject();

                    cJSON *networkNamesArray = cJSON_CreateArray();
                    cJSON *networkMacArray = cJSON_CreateArray();
                    cJSON *networkRssiArray = cJSON_CreateArray();
                    cJSON *networkChannelArray = cJSON_CreateArray();
                    scan_wifi(networkNamesArray, networkMacArray, networkRssiArray, networkChannelArray);

                    cJSON_AddItemToObject(networkScan, "networkName", networkNamesArray);
                    cJSON_AddItemToObject(networkScan, "networkMacAddress", networkMacArray);
                    cJSON_AddItemToObject(networkScan, "networkRssi", networkRssiArray);
                    cJSON_AddItemToObject(networkScan, "networkChannel", networkChannelArray);
                    cJSON_AddItemToObject(root, "networkScan", networkScan);

                    cJSON *informationUWB = cJSON_CreateObject();

                    cJSON *addrArray = cJSON_CreateArray();
                    cJSON *channelArray = cJSON_CreateArray();
                    cJSON *rxPowerArray = cJSON_CreateArray();
                    cJSON *fpPowerArray = cJSON_CreateArray();
                    cJSON *qualityArray = cJSON_CreateArray();
                    cJSON *rangeArray = cJSON_CreateArray();
                    cJSON *tofArray = cJSON_CreateArray();
                    cJSON *tofDisArray = cJSON_CreateArray();
                    cJSON *macDeviceArray = cJSON_CreateArray();

                    convertDataUWB(macDeviceArray, addrArray, channelArray, rxPowerArray, fpPowerArray, qualityArray, rangeArray, tofArray, tofDisArray);
                    cJSON_AddItemToObject(informationUWB, "macDevice", macDeviceArray);
                    cJSON_AddItemToObject(informationUWB, "channel", channelArray);
                    cJSON_AddItemToObject(informationUWB, "fpPower(dBm)", fpPowerArray);
                    cJSON_AddItemToObject(informationUWB, "rxPower(dBm)", rxPowerArray);
                    cJSON_AddItemToObject(informationUWB, "toa_timecount", tofArray);
                    // cJSON_AddItemToObject(informationUWB, "quality ", qualityArray);
                    cJSON_AddItemToObject(informationUWB, "distance_m", tofDisArray);
                    cJSON_AddItemToObject(informationUWB, "from", addrArray);
                    cJSON_AddItemToObject(informationUWB, "range", rangeArray);

                    cJSON_AddItemToObject(root, "information.uwb", informationUWB);

                    runningNumber++;
                    if (runningNumber > 99999)
                    {
                        runningNumber = 1;
                    }
                    save_running_number(runningNumber);

                    char *json_string = cJSON_Print(root);
                    // printf("JSON String: %s\r\n", json_string);
                    post_json(json_string);
                    cJSON_Delete(root);
                    free(json_string);
                }
                Serial.print("status_server :");
                Serial.println(status_server);
                stepOLED = 5;
            }
            else if (stepOLED == 5)
            {
                if (displayCounter >= 60)
                {
                    stepOLED = 6;
                    displayCounter = 0;
                }
                displayCounter++;
                display.clearDisplay();
                display.setCursor(0, 0);
                display.setTextSize(2);
                display.printf("Uploading\n");
                for (int i = 0; i < (displayCounter % 10); i++)
                {
                    display.print(".");
                }
                display.display();
                displayCounter++;
            }
            else if (stepOLED == 6)
            {
                if (displayCounter >= 60)
                {
                    stepOLED = 7;
                    displayCounter = 0;
                }
                displayCounter++;

                display.clearDisplay();
                display.setCursor(0, 0);
                display.setTextSize(2);
                display.print("TAG: ");
                display.println(TagNo);
                display.print("SQ : ");
                display.println(runningNumber);
                display.setTextSize(1);
                display.print("REC Data :");
                display.println(post_OK ? "Complete" : "Failed");
                display.display();
            }
            else if (stepOLED == 7)
            {
                OnSendData = false;
                rssi_OK = false;
                uwb_OK = false;
                post_OK = false;
                stepOLED = 0;
                status_server = 0;
            }
        }
        else
        {
            if (counterOledTime >= 10)
            {
                page = !page;
                counterOledTime = 0;
            }
            if (page == 0)
            {
                displayHomePage();
            }
            else
            {
                displayData(&csi_data_snap);
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void add_csi_to_json(cJSON *wifiInfo)
{
    if (csi_data_snap.buf != NULL && csi_data_snap.len > 0)
    {
        int *int_array = (int *)malloc(csi_data_snap.len * sizeof(int));
        for (size_t i = 0; i < csi_data_snap.len; i++)
        {
            int_array[i] = (int)csi_data_snap.buf[i];
        }
        cJSON *csiArray = cJSON_CreateIntArray(int_array, csi_data_snap.len);
        cJSON_AddItemToObject(wifiInfo, "CSI", csiArray);
        free(int_array);
    }
    else
    {
        printf("CSI buffer is empty or not initialized.\n");
    }
}

void scan_wifi(cJSON *networkNamesArray, cJSON *networkMacArray, cJSON *networkRssiArray, cJSON *networkChannelArray)
{
    int MAX_AP_COUNT = 20;
    uint16_t ap_count = 0;
    wifi_ap_record_t apRecords[MAX_AP_COUNT];

    esp_wifi_scan_start(NULL, true);
    esp_wifi_scan_get_ap_num(&ap_count);

    if (ap_count > MAX_AP_COUNT)
    {
        ap_count = MAX_AP_COUNT;
    }

    esp_wifi_scan_get_ap_records(&ap_count, apRecords);

    for (int i = 0; i < ap_count; i++)
    {

        // เพิ่ม MAC Address
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 apRecords[i].bssid[0], apRecords[i].bssid[1], apRecords[i].bssid[2],
                 apRecords[i].bssid[3], apRecords[i].bssid[4], apRecords[i].bssid[5]);

        cJSON_AddItemToArray(networkNamesArray, cJSON_CreateString((const char *)apRecords[i].ssid));
        cJSON_AddItemToArray(networkMacArray, cJSON_CreateString(mac_str));
        cJSON_AddItemToArray(networkRssiArray, cJSON_CreateNumber(apRecords[i].rssi));
        cJSON_AddItemToArray(networkChannelArray, cJSON_CreateNumber(apRecords[i].primary));
        // printf("SSID: %s, MAC: %s, RSSI: %d, Channel: %d\n",apRecords[i].ssid, mac_str, apRecords[i].rssi, apRecords[i].primary);
    }
}

void init_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void save_running_number(uint32_t number)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK)
    {
        err = nvs_set_u32(my_handle, "run_num", number);
        ESP_ERROR_CHECK(err);

        err = nvs_commit(my_handle);
        ESP_ERROR_CHECK(err);

        nvs_close(my_handle);
    }
}

uint32_t load_running_number()
{
    nvs_handle_t my_handle;
    uint32_t runningNumber = 1;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK)
    {
        err = nvs_get_u32(my_handle, "run_num", &runningNumber);
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            runningNumber = 1;
        }
        nvs_close(my_handle);
    }
    return runningNumber;
}

static void periodic_timer_callback(void *arg)
{
    DW1000Ranging.loop();
}

extern "C" void app_main()
{
    pinMode(GPIO_39, INPUT_PULLDOWN);
    pinMode(GPIO_36, INPUT_PULLDOWN);
    pinMode(GPIO_22, INPUT_PULLDOWN);
    // pinMode(GPIO_17, INPUT);
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(1000);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }
    display.clearDisplay();
    logoshow();
    Serial.begin(115200);

    init_nvs();
    runningNumber = load_running_number();

    initialise_wifi();
    wifi_csi_init();

#if CONFIG_WIFI_CONNECT_AP
    Serial.println("Start AP Mode");
    wifi_ap();
#elif CONFIG_WIFI_CONNECT_STA
    Serial.println("Start STA Mode");
    wifi_sta(CONFIG_STA_CONNECT_TIMEOUT * 1000);
#elif CONFIG_WIFI_CONNECT_APSTA
    Serial.println("Start APSTA Mode");
    wifi_apsta(CONFIG_STA_CONNECT_TIMEOUT * 1000);
#endif

    esp_wifi_get_mac(WIFI_IF_STA, mac);
    Serial.printf("Client MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sprintf(TAG_ADDR, "%02X:%02X:%02X:%02X:%02X:%02X", mac[4], mac[5], mac[0], mac[1], mac[2], mac[3]);
    Serial.print("TAG_ADDR: ");
    Serial.println(TAG_ADDR);

    uint64_t chipid = ESP.getEfuseMac();
    uint32_t deviceID = (uint32_t)(chipid >> 24);
    Serial.print("deviceID: ");
    Serial.println(deviceID);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    delay(100);

    DW1000Ranging.initCommunication(UWB_RST, UWB_SS, UWB_IRQ);

    char device_mac[18];
    sprintf(device_mac, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    DW1000.newConfiguration();
    if (strcmp(device_mac, "D0:EF:76:35:84:B1") == 0)
    {
        DW1000.setChannel(1); // Anchor on Ch 1
        Serial.println("Selected Channel: 1 (Anchor D0:EF:76:35:84:B1)");
    }
    else if (strcmp(device_mac, "D0:EF:76:35:84:8D") == 0)
    {
        DW1000.setChannel(2); // Anchor on Ch 2
        Serial.println("Selected Channel: 2 (Anchor D0:EF:76:35:84:8D)");
    }
    else if (strcmp(device_mac, "EC:64:C9:BF:C4:71") == 0)
    {
        DW1000.setChannel(3); // Anchor on Ch 3
        Serial.println("Selected Channel: 3 (Anchor EC:64:C9:BF:C4:71)");
    }
    else if (strcmp(device_mac, "D0:EF:76:35:73:D9") == 0)
    {
        DW1000.setChannel(4); // Anchor on Ch 4
        Serial.println("Selected Channel: 4 (Anchor D0:EF:76:35:73:D9)");
    }
    else if (strcmp(device_mac, "C8:2E:18:DD:90:BC") == 0)
    {
        TagNo = 1;
        DW1000.setChannel(5); // Tag on Ch 5
        Serial.println("Selected Channel: 5 (Tag C8:2E:18:DD:90:BC)");
    }
    else if (strcmp(device_mac, "C8:2E:18:AD:9E:F0") == 0)
    {
        TagNo = 2;
        DW1000.setChannel(7); // Tag on Ch 7
        Serial.println("Selected Channel: 7 (Tag C8:2E:18:AD:9E:F0)");
    }
    else if (strcmp(device_mac, "C8:2E:18:AC:0D:4C") == 0)
    {
        TagNo = 3;
        DW1000.setChannel(1); // Tag on Ch 1
        Serial.println("Selected Channel: 1 (Tag C8:2E:18:AC:0D:4C)");
    }
    else
    {
        // Default case if no matching MAC is found
        Serial.println("Unknown MAC Address. Setting default channel to 1.");
        DW1000.setChannel(1);
    }

    DW1000.commitConfiguration();
    Serial.println("DW1000 configuration committed.");
    uwbStatus = true;
    DW1000.enableDebounceClock();
    DW1000.enableLedBlinking();
    DW1000.setGPIOMode(MSGP3, LED_MODE);

    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
    DW1000Ranging.useRangeFilter(true);

    xTaskCreate(&RTC, "RTC", 4096, NULL, 1, NULL);
    xTaskCreate(&Oled, "OLED", 8192, NULL, 2, NULL);

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1));
}
