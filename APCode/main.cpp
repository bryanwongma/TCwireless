#include <Arduino.h>
#include <SPI.h>
#include <ctime>
#include <sys/time.h>
#include "esp_sntp.h"
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <string.h>
#include <stdio.h>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_http_client.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "netif/etharp.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "freertos/semphr.h"

struct RealTimeClock
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
RealTimeClock rtc;

void time_sync_notification_cb(struct timeval *tv);
void obtain_time();
//-------------------------------WiFi APSTA CSI--------------------------------
#define CONFIG_AP_WIFI_SSID "AIT-TCB"
#define CONFIG_AP_WIFI_PASSWORD "Ait@2024"
#define CONFIG_AP_MAX_STA_CONN 4
#define CONFIG_AP_WIFI_CHANNEL 0
#define CONFIG_STA_WIFI_SSID "BrianWong"
#define CONFIG_STA_WIFI_PASSWORD "12345678"
#define CONFIG_STA_CONNECT_TIMEOUT 10
#define CONFIG_WIFI_CONNECT_APSTA 1
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
void initialize_sntp(void);
#define QUEUE_SIZE 10
#define BUFFER_SIZE 4096
//-------------------------------UWB Param-------------------------------
// Leftmost two bytes below will become the "short address"
char anchor_addr[24]; // #4
// Calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16611; // 16611

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// Connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin
esp_err_t GET_status_server();
int status_server = 0;
static QueueHandle_t json_queue;

extern "C" void app_main();

//--------------------------------------UWB------------------------------------------
#define NUMBER_OF_DISTANCES 1
int64_t TIME_OVERFLOW = 0x10000000000; // 1099511627776LL
// Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
float DISTANCE_OF_RADIO = 0.0046917639786159f;
void newRange()
{
  Serial.print("from : ");
  Serial.println(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
}

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

//----------------------------------------WiFI-------------------------------------------

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    Serial.println("WIFI_EVENT_STA_DISCONNECTED");
    esp_wifi_connect();
    xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    Serial.println("WIFI_EVENT_STA_CONNECTED");
    xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
  }
}

static void print_wifi_mac_address()
{
  uint8_t mac[6];

  // Get MAC address for Station interface (STA)
  ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
  Serial.print("STA MAC Address: ");
  for (int i = 0; i < 6; i++)
  {
    if (i > 0)
      Serial.print(":");
    Serial.print(mac[i], HEX);
  }
  Serial.println();

  // Get MAC address for Access Point interface (AP)
  ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, mac));
  Serial.print("AP MAC Address: ");
  for (int i = 0; i < 6; i++)
  {
    if (i > 0)
      Serial.print(":");
    Serial.print(mac[i], HEX);
  }
  Serial.println();
}

static void print_connected_stas()
{
  wifi_sta_list_t sta_list;
  esp_err_t err = esp_wifi_ap_get_sta_list(&sta_list);
  if (err == ESP_OK)
  {
    Serial.println("Connected STAs:");
    for (int i = 0; i < sta_list.num; i++)
    {
      Serial.print("STA ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(sta_list.sta[i].mac[0], HEX);
      Serial.print(":");
      Serial.print(sta_list.sta[i].mac[1], HEX);
      Serial.print(":");
      Serial.print(sta_list.sta[i].mac[2], HEX);
      Serial.print(":");
      Serial.print(sta_list.sta[i].mac[3], HEX);
      Serial.print(":");
      Serial.print(sta_list.sta[i].mac[4], HEX);
      Serial.print(":");
      Serial.println(sta_list.sta[i].mac[5], HEX);
    }
  }
  else
  {
    Serial.print("Failed to get STA list: ");
    Serial.println(esp_err_to_name(err));
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

#if CONFIG_WIFI_CONNECT_APSTA
static bool wifi_apsta(int timeout_ms)
{
  wifi_config_t ap_config = {0};
  strcpy((char *)ap_config.ap.ssid, CONFIG_AP_WIFI_SSID);
  strcpy((char *)ap_config.ap.password, CONFIG_AP_WIFI_PASSWORD);
  ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  ap_config.ap.ssid_len = strlen(CONFIG_AP_WIFI_SSID);
  ap_config.ap.max_connection = CONFIG_AP_MAX_STA_CONN;
  ap_config.ap.channel = CONFIG_AP_WIFI_CHANNEL;

  if (strlen(CONFIG_AP_WIFI_PASSWORD) == 0)
  {
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  wifi_config_t sta_config = {0};
  strcpy((char *)sta_config.sta.ssid, CONFIG_STA_WIFI_SSID);
  strcpy((char *)sta_config.sta.password, CONFIG_STA_WIFI_PASSWORD);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));   // Use correct constant
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config)); // Use correct constant
  ESP_ERROR_CHECK(esp_wifi_start());

  Serial.printf("WIFI_MODE_AP started. SSID:%s password:%s channel:%d\n",
                CONFIG_AP_WIFI_SSID, CONFIG_AP_WIFI_PASSWORD, CONFIG_AP_WIFI_CHANNEL);

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

void time_sync_notification_cb(struct timeval *tv)
{
  Serial.println("Time synchronized");
}
void obtain_time()
{
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_set_time_sync_notification_cb([](struct timeval *tv)
                                     { Serial.println("Time synchronized"); });
  sntp_init();
  time_t now = 0;
  struct tm timeinfo = {0};
  int retry = 0;
  const int retry_count = 10;
  while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count)
  {
    Serial.printf("Waiting for system time to be set... (%d/%d)\n", retry, retry_count);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    time(&now);
    localtime_r(&now, &timeinfo);
  }

  if (timeinfo.tm_year >= (2020 - 1900))
  {
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    Serial.printf("The current date/time is: %s\n", strftime_buf);
  }
  else
  {
    Serial.println("Failed to synchronize time.");
  }
}

void print_current_time()
{
  Serial.printf("T: %04d%02d%02d %02d:%02d:%02d:%03d\n",
                rtc.year, rtc.month, rtc.day,
                rtc.hour, rtc.minute, rtc.second,
                rtc.millisecond);
}

static void RTC(void *pvParameter)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  int64_t start_time_sec = now.tv_sec;
  int64_t start_time_us = esp_timer_get_time();

  Serial.printf("Tracking time started at: %lld sec, %lld us\n", start_time_sec, start_time_us);
  unsigned long runtime = 0;
  while (true)
  {

    if ((millis() - runtime) > 60000)
    {
      Serial.println("Send Status to Server");
      GET_status_server();
      runtime = millis();
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

esp_err_t time_get_handler(httpd_req_t *req)
{
  char time_str[64];
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);

  if (timeinfo.tm_year > (2016 - 1900))
  {
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
    httpd_resp_send(req, time_str, HTTPD_RESP_USE_STRLEN);
    printf("Time Sent to Client: %s\n", time_str); // Serial.print() equivalent
  }
  else
  {
    httpd_resp_send_500(req);
    printf("Failed to obtain time\n"); // Serial.print() equivalent
  }
  return ESP_OK;
}

esp_err_t status_get_handler(httpd_req_t *req)
{
  char str[64];
  status_server = 0;
  GET_status_server();

  if (status_server != 0)
  {
    sprintf(str,"%d",status_server);
    httpd_resp_send(req, str, HTTPD_RESP_USE_STRLEN);
    printf("Status server to Client: %s\n", str); // Serial.print() equivalent
  }
  else
  {
    httpd_resp_send_500(req);
    printf("Failed to obtain time\n"); // Serial.print() equivalent
  }
  return ESP_OK;
}

esp_err_t json_post_handler(httpd_req_t *req)
{
  char buffer[4096];
  int total_len = req->content_len;
  int cur_len = 0;
  int received = 0;

  if (total_len >= sizeof(buffer))
  {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  while (cur_len < total_len)
  {
    received = httpd_req_recv(req, buffer + cur_len, total_len - cur_len);
    if (received <= 0)
    {
      if (received == HTTPD_SOCK_ERR_TIMEOUT)
      {
        continue;
      }
      return ESP_FAIL;
    }
    cur_len += received;
  }
  buffer[total_len] = '\0';

  // Serial.print("Received JSON: ");
  // Serial.println(buffer);

  cJSON *json = cJSON_Parse(buffer);
  if (json == NULL)
  {
    const char *error_ptr = cJSON_GetErrorPtr();
    if (error_ptr != NULL)
    {
      Serial.printf("Error before: %s\n", error_ptr);
    }
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  /*     cJSON *session = cJSON_GetObjectItem(json, "packetid");
      if (session != NULL)
      {
          Serial.println("sessionID: ");
          Serial.println(cJSON_GetObjectItem(session, "sessionID")->valuestring);
      } */

  if (xQueueSend(json_queue, buffer, portMAX_DELAY) != pdTRUE)
  {
    Serial.println("Failed to add JSON to queue");
    cJSON_Delete(json);
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  cJSON_Delete(json);

  httpd_resp_send(req, "JSON received and parsed", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// Register URI handler
httpd_handle_t start_webserver(void)
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 8192;
  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) == ESP_OK)
  {
    httpd_uri_t time_uri = {
        .uri = "/get-time",
        .method = HTTP_GET,
        .handler = time_get_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &time_uri);

    httpd_uri_t status_uri = {
        .uri = "/get-status-server",
        .method = HTTP_GET,
        .handler = status_get_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &status_uri);

    httpd_uri_t json_uri = {
        .uri = "/post-json",
        .method = HTTP_POST,
        .handler = json_post_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &json_uri);
  }
  return server;
}

void initialize_sntp(void)
{
  printf("Initializing SNTP\n"); // Serial.print() equivalent
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
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

esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
{
  static u_int16_t i = 0;
  u_int16_t y = 0;
  char *ptr;

  switch (evt->event_id)
  {
  case HTTP_EVENT_ERROR:
    Serial.println("HTTP_EVENT_ERROR");
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

esp_err_t post_to_server(char *json_data)
{
  esp_http_client_config_t config = {
      .url = "https://telecomait.ngrok.app/data",
      .method = HTTP_METHOD_POST,
      .event_handler = client_event_post_handler,
      .skip_cert_common_name_check = true,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);

  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_post_field(client, json_data, strlen(json_data));

  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK)
  {
    Serial.print("POST status = ");
    Serial.println(esp_http_client_get_status_code(client));
  }
  else
  {
    Serial.print("POST request failed: ");
    Serial.println(esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);

  return err;
}

esp_err_t GET_status_server()
{
  uint64_t chipid = ESP.getEfuseMac();
  uint32_t deviceID = (uint32_t)(chipid >> 24);
  String url = String("https://telecomait.ngrok.app/access_point?DeviceID=") + String(deviceID);

  esp_http_client_config_t config = {
      .url = url.c_str(),
      .method = HTTP_METHOD_GET,
      .event_handler = client_event_get_handler,
      .skip_cert_common_name_check = true,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);

  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK)
  {
    Serial.print("GET status = ");
    status_server = esp_http_client_get_status_code(client);
    Serial.println(esp_http_client_get_status_code(client));
  }
  else
  {
    Serial.print("GET request failed: ");
    Serial.println(esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);

  return err;
}

void process_json_queue(void *pvParameters)
{
  char json_data[BUFFER_SIZE];

  while (1)
  {
    UBaseType_t queue_length = uxQueueMessagesWaiting(json_queue);
    Serial.print("Current queue length: ");
    Serial.println(queue_length);

    if (xQueueReceive(json_queue, &json_data, portMAX_DELAY) == pdTRUE)
    {
      // Serial.println("Processing JSON from queue: ");
      // Serial.println(json_data);

      if (strlen(json_data) > 0 && strlen(json_data) < BUFFER_SIZE)
      {
        if (post_to_server(json_data) == ESP_OK)
        {
          Serial.println("JSON successfully sent to server.");
          Serial.print("Items remaining in queue: ");
          Serial.println(uxQueueMessagesWaiting(json_queue));
        }
        else
        {
          Serial.println("Failed to send JSON to server.");
        }
      }
      else
      {
        Serial.println("Error: JSON data is too large or empty.");
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

static void periodic_timer_callback(void *arg)
{
  DW1000Ranging.loop();
}

void app_main()
{

  Serial.begin(115200);

  json_queue = xQueueCreate(QUEUE_SIZE, BUFFER_SIZE);

  if (json_queue == NULL)
  {
    Serial.println("Failed to create queue");
    return;
  }

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  initialise_wifi();
  print_wifi_mac_address();

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_AP, mac);
  Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sprintf(anchor_addr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[4], mac[5], mac[0], mac[1], mac[2], mac[3]);
  Serial.print("TAG_ADDR: ");
  Serial.println(anchor_addr);
  Serial.println("Start APSTA Mode");
  wifi_apsta(CONFIG_STA_CONNECT_TIMEOUT * 1000);

  uint64_t chipid = ESP.getEfuseMac();
  uint32_t deviceID = (uint32_t)(chipid >> 24);
  Serial.print("deviceID: ");
  Serial.println(deviceID);
  // Initialize the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pin

  // Set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

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
    DW1000.setChannel(5); // Tag on Ch 5
    Serial.println("Selected Channel: 5 (Tag C8:2E:18:DD:90:BC)");
  }
  else if (strcmp(device_mac, "C8:2E:18:AD:9E:F0") == 0)
  {
    DW1000.setChannel(7); // Tag on Ch 7
    Serial.println("Selected Channel: 7 (Tag C8:2E:18:AD:9E:F0)");
  }
  else if (strcmp(device_mac, "C8:2E:18:AC:0D:4C") == 0)
  {
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
  DW1000.enableDebounceClock();
  DW1000.enableLedBlinking();
  DW1000.setGPIOMode(MSGP3, LED_MODE);
  Serial.println("DW1000 configuration committed.");

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.useRangeFilter(true);
  DW1000Ranging.setRangeFilterValue(10);
  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &periodic_timer_callback,
      .name = "periodic"};
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1));

  xTaskCreate(&process_json_queue, "process_json_queue", 8192, NULL, 5, NULL);
  xTaskCreate(&RTC, "RTC", 4096, NULL, 5, NULL);
  initialize_sntp();
  GET_status_server();
  start_webserver();
}
