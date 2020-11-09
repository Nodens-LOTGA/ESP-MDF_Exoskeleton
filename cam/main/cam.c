// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "mdf_common.h"
#include "mupgrade.h"
#include "mwifi.h"

#include "driver/sdmmc_host.h"
#include "esp_camera.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "math.h"

#include "../../exosk.h"
#include "driver/rtc_io.h"

//#define MEMORY_DEBUG

#define GPIO_LED_1 GPIO_NUM_2
#define GPIO_LED_2 GPIO_NUM_4
#define GPIO_LED_3 GPIO_NUM_33
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_LED_1) | (1ULL << GPIO_LED_2))

#define REQUEST_RETRIES_COUNT 25

static const char *TAG = "exosk_cam";
static esp_netif_t *sta_netif = NULL;
static bool parent_responded = false;
static bool show_errors = true;

static char router_ssid[32] = {0}, router_password[64] = {0};
static uint8_t parent_mac[MWIFI_ADDR_LEN] = {0};
static uint16_t exosk_id = 0;
static uint32_t wake_up_gap = 5 * 1000;  // MS
static uint32_t restart_gap = 60 * 1000; // MS

static void deepsleep_set(void) {
  MDF_ERROR_ASSERT(mwifi_stop());
  MDF_ERROR_ASSERT(esp_wifi_disconnect());
  MDF_ERROR_ASSERT(esp_wifi_stop());
  MDF_ERROR_ASSERT(esp_wifi_deinit());

  gpio_set_level(GPIO_LED_1, 0);
  gpio_set_level(GPIO_LED_2, 0);
  gpio_set_level(GPIO_LED_3, 1);

  rtc_gpio_isolate(GPIO_LED_1);
  rtc_gpio_isolate(GPIO_LED_2);
  rtc_gpio_isolate(GPIO_LED_3);

  rtc_gpio_isolate(CONFIG_PWDN);
  // rtc_gpio_isolate(CONFIG_RESET);
  rtc_gpio_isolate(CONFIG_XCLK);
  rtc_gpio_isolate(CONFIG_SDA);
  rtc_gpio_isolate(CONFIG_SCL);
  rtc_gpio_isolate(CONFIG_D7);
  rtc_gpio_isolate(CONFIG_D6);
  rtc_gpio_isolate(CONFIG_D5);
  rtc_gpio_isolate(CONFIG_D4);
  rtc_gpio_isolate(CONFIG_VSYNC);

  rtc_gpio_isolate(GPIO_NUM_12);
  rtc_gpio_isolate(GPIO_NUM_13);
  rtc_gpio_isolate(GPIO_NUM_14);
  rtc_gpio_isolate(GPIO_NUM_15);

  MDF_LOGI("esp_sleep_enable_timer_wakeup, internal: %d ms", wake_up_gap);
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wake_up_gap * 1000));
  MDF_LOGI("Entering deep sleep");
  esp_deep_sleep_start();
}

static void startup_handle(void) {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  esp_reset_reason_t reason = esp_reset_reason();

  if (reason == ESP_RST_SW) {
    MDF_LOGI("ESP_RST_SW");
    show_errors = true;
  } else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    MDF_LOGI("ESP_SLEEP_WAKEUP_TIMER");
    show_errors = false;
    esp_restart();
  }
}

static void init_gpio(void) {
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);
}

static void error_handle(exosk_error_t error) {
  if (show_errors) {
    uint32_t level = 1;
    uint32_t last_tick = 0, tick_elapsed = 0;
    init_gpio();
    if (error == EXOSK_ERROR_WIFI) {
      gpio_set_level(GPIO_LED_1, level);
      gpio_set_level(GPIO_LED_2, level);
    } else {
      while (1) {
        if (error == EXOSK_ERROR_SD_CARD)
          gpio_set_level(GPIO_LED_1, level);
        else if (error == EXOSK_ERROR_CAMERA) {
          gpio_set_level(GPIO_LED_2, level);
          if (tick_elapsed > restart_gap / portTICK_RATE_MS)
            esp_restart();
          tick_elapsed += xTaskGetTickCount() - last_tick;
          last_tick = xTaskGetTickCount();
        }
        level ^= 1;
        vTaskDelay(500 / portTICK_RATE_MS);
      }
    }
  } else {
    esp_restart();
  }
}

static camera_config_t camera_config = {
    .pin_pwdn = CONFIG_PWDN,
    .pin_reset = CONFIG_RESET,
    .pin_xclk = CONFIG_XCLK,
    .pin_sscb_sda = CONFIG_SDA,
    .pin_sscb_scl = CONFIG_SCL,

    .pin_d7 = CONFIG_D7,
    .pin_d6 = CONFIG_D6,
    .pin_d5 = CONFIG_D5,
    .pin_d4 = CONFIG_D4,
    .pin_d3 = CONFIG_D3,
    .pin_d2 = CONFIG_D2,
    .pin_d1 = CONFIG_D1,
    .pin_d0 = CONFIG_D0,
    .pin_vsync = CONFIG_VSYNC,
    .pin_href = CONFIG_HREF,
    .pin_pclk = CONFIG_PCLK,

    // XCLK 20MHz or 10MHz
    .xclk_freq_hz = CONFIG_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,   // QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, // 0-63 lower number means higher quality
    .fb_count = 1       // if more than one, i2s runs in continuous mode. Use only with JPEG
};

static void init_camera(void) {
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
    error_handle(EXOSK_ERROR_CAMERA);
  }
}

static mdf_err_t wifi_init(void) {
  mdf_err_t ret = nvs_flash_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    MDF_ERROR_ASSERT(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  MDF_ERROR_ASSERT(ret);

  MDF_ERROR_ASSERT(esp_netif_init());
  MDF_ERROR_ASSERT(esp_event_loop_create_default());
  MDF_ERROR_ASSERT(esp_netif_create_default_wifi_mesh_netifs(&sta_netif, NULL));
  MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
  MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
  MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
  MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
  MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
  MDF_ERROR_ASSERT(esp_wifi_start());

  return MDF_OK;
}

static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx) {
  mesh_addr_t parent_bssid = {0};

  MDF_LOGI("event_loop_cb, event: %d", event);

  switch (event) {
  case MDF_EVENT_MWIFI_STARTED:
    MDF_LOGI("MESH is started");
    break;

  case MDF_EVENT_MWIFI_STOPPED:
    MDF_LOGI("MESH is stopped");
    break;

  case MDF_EVENT_MWIFI_PARENT_CONNECTED: {
    parent_responded = 0;
    esp_mesh_get_parent_bssid(&parent_bssid);
    gpio_set_level(GPIO_LED_1, 0);
    gpio_set_level(GPIO_LED_2, 0);
    break;
  }

  case MDF_EVENT_MWIFI_NO_PARENT_FOUND:
    MDF_LOGI("Parent not found");
    break;

  case MDF_EVENT_MWIFI_PARENT_DISCONNECTED: {
    mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)ctx;
    MDF_LOGI("<MESH_EVENT_PARENT_DISCONNECTED>reason:%d", disconnected->reason);
    break;
  }

  case MDF_EVENT_MWIFI_LAYER_CHANGE:
    MDF_LOGI("Layer change");
    break;

  case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
  case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
    MDF_LOGI("total_num: %d", esp_mesh_get_total_node_num());
    break;

  default:
    break;
  }

  return MDF_OK;
}

static void init_mesh(void) {
  mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
  mwifi_config_t config = {.mesh_id = CONFIG_MESH_ID, .mesh_password = CONFIG_MESH_PASSWORD, .mesh_type = MWIFI_MESH_LEAF};
  strcpy(config.router_ssid, router_ssid);
  strcpy(config.router_password, router_password);
  MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
  MDF_ERROR_ASSERT(wifi_init());
  MDF_ERROR_ASSERT(mwifi_init(&cfg));
  MDF_ERROR_ASSERT(mwifi_set_config(&config));

  const uint8_t group_id_list[MWIFI_ADDR_LEN] = EXOSK_GROUP_ID_CAM;
  MDF_ERROR_ASSERT(esp_mesh_set_group_id((mesh_addr_t *)group_id_list, sizeof(group_id_list) / sizeof(group_id_list[0])));

  MDF_ERROR_ASSERT(mwifi_start());
  error_handle(EXOSK_ERROR_WIFI);
}

static void init_sd(void) {
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 3,
  };
  sdmmc_card_t *card;

  ESP_LOGI(TAG, "Mounting SD card...");
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "SD card mount successfully!");
  } else {
    ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
    error_handle(EXOSK_ERROR_SD_CARD);
  }

  char buffer[128];
  ESP_LOGI(TAG, "Opening file");
  FILE *f = fopen("/sdcard/config.txt", "r");
  if (f == NULL) {
    ESP_LOGE(TAG, "Failed to open file for reading");
    error_handle(EXOSK_ERROR_SD_CARD);
  }
  ESP_LOGI(TAG, "Reading config...");
  while (!feof(f)) {
    fgets(buffer, sizeof(buffer), f);
    ESP_LOGI(TAG, "Read from file: \"%s\"", buffer);
    if (sscanf(buffer, "ssid=%s", router_ssid) == 1)
      continue;
    if (sscanf(buffer, "pass=%s", router_password) == 1)
      continue;
    if (sscanf(buffer, "gap=%d", &wake_up_gap) == 1)
      continue;
    if (sscanf(buffer, "mac=%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx", &parent_mac[0], &parent_mac[1], &parent_mac[2], &parent_mac[3], &parent_mac[4],
               &parent_mac[5]) == 6)
      continue;
    if (sscanf(buffer, "res_gap=%d", &restart_gap) == 1)
      continue;
  }
  fclose(f);
  ESP_LOGI(TAG, "Router SSID: %s", router_ssid);
  ESP_LOGI(TAG, "Router Password: %s", router_password);
  ESP_LOGI(TAG, "Wake up gap: %d", wake_up_gap);
  ESP_LOGI(TAG, "Parent MAC: \"" MACSTR "\"", MAC2STR(parent_mac));
  ESP_LOGI(TAG, "Restart gap: %d", restart_gap);
  if (parent_mac[0] == 0) {
    ESP_LOGE(TAG, "Parent mac not found");
    error_handle(EXOSK_ERROR_SD_CARD);
  }
  if (router_ssid[0] == 0) {
    ESP_LOGE(TAG, "Router SSID is incorrect");
    error_handle(EXOSK_ERROR_SD_CARD);
  }
  if (router_password[0] == 0) {
    ESP_LOGE(TAG, "Router password is incorrect");
    error_handle(EXOSK_ERROR_SD_CARD);
  }
  esp_vfs_fat_sdcard_unmount("/sdcard", card);
  ESP_LOGI(TAG, "Card unmounted");
}

static void node_read_task(void *arg) {
  mdf_err_t ret = MDF_OK;
  char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
  size_t size = MWIFI_PAYLOAD_LEN;
  mwifi_data_type_t data_type = {0x0};
  exosk_wifi_data_type_t exosk_data_type = {0x0};
  uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};

  MDF_LOGI("Node read task is running");

  for (;;) {
    if (!mwifi_is_connected()) {
      vTaskDelay(500 / portTICK_RATE_MS);
      continue;
    }

    size = MWIFI_PAYLOAD_LEN;
    memset(data, 0, MWIFI_PAYLOAD_LEN);
    ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
    MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
    MDF_LOGD("Node receive: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
    src_addr[5]++;
    MDF_LOGD("Parent MAC: " MACSTR, MAC2STR(parent_mac));
    memcpy(&exosk_data_type, &data_type.custom, sizeof(data_type.custom));
    MDF_LOGD("Type: %d, Format: %d", exosk_data_type.type, exosk_data_type.format);
    if (memcmp(src_addr, parent_mac, MWIFI_ADDR_LEN) == 0) {
      MDF_LOGD("Parent found");
      if (exosk_data_type.type == EXOSK_WIFI_TYPE_BOARD && exosk_data_type.format == EXOSK_WIFI_FORMAT_JSON) {
        if (strcmp(data, EXOSK_TO_CAM_RESPONSE) == 0) {
          exosk_id = exosk_data_type.exosk_id;
          ESP_LOGI(TAG, "Exoskeleton ID: %d", exosk_id);
          parent_responded = true;
          break;
        }
      }
    }
    vTaskDelay(500 / portTICK_RATE_MS);
  }

  MDF_LOGW("Node read task is exit");

  MDF_FREE(data);
  vTaskDelete(NULL);
}

static void node_write_task(void *arg) {
  int pic_frag = 0, size = 0, len = 0, last_len = 0;
  char *data = NULL;
  mdf_err_t ret = MDF_OK;
  mwifi_data_type_t data_type = {0};
  exosk_wifi_data_type_t exosk_data_type = {.type = EXOSK_WIFI_TYPE_CAM};
  uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
  uint8_t attempts = 1;

  MDF_LOGI("Node task is running");

  esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
  for (;;) {
    if (!mwifi_is_connected()) {
      vTaskDelay(500 / portTICK_RATE_MS);
      continue;
    }
    if (parent_responded) {
      ESP_LOGI(TAG, "Taking picture...");
      camera_fb_t *pic = esp_camera_fb_get();
      if (!pic) {
        ESP_LOGE(TAG, "Frame buffer could not be acquired");
        error_handle(EXOSK_ERROR_CAMERA);
      }
      exosk_data_type.exosk_id = exosk_id;
      exosk_data_type.format = EXOSK_WIFI_FORMAT_JSON;
      exosk_data_type.data = (int)ceil((float)pic->len / MWIFI_PAYLOAD_LEN);
      size = asprintf(&data, "{\"src\":\"" MACSTR "\",\"id\":%d,\"img\":{\"type\":\"jpeg\",\"size\":%d,\"frag\":%d}}", MAC2STR(sta_mac), exosk_id, pic->len,
                      exosk_data_type.data);
      ret = mwifi_write(NULL, &data_type, data, size, true);
      MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
      size = pic->len;
      ESP_LOGI(TAG, "Pic size:%d", size);
      len = 0;
      last_len = 0;
      pic_frag = 1;
      exosk_data_type.format = EXOSK_WIFI_FORMAT_JPEG;
      gpio_set_level(GPIO_LED_2, 1);
      while (size > 0) {
        if (size > MWIFI_PAYLOAD_LEN)
          len = MWIFI_PAYLOAD_LEN;
        else
          len = size;
        MDF_LOGD("Node send, size: %d", len);
        exosk_data_type.data = pic_frag;
        memcpy(&data_type.custom, &exosk_data_type, sizeof(data_type.custom));
        ret = mwifi_write(NULL, &data_type, pic->buf + last_len, len, true);
        last_len += len;
        size -= len;
        pic_frag++;
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
      }
      MDF_FREE(data);
      esp_camera_fb_return(pic);
      deepsleep_set();
    } else {
      if (attempts > REQUEST_RETRIES_COUNT)
        error_handle(EXOSK_ERROR_WIFI);
      size = asprintf(&data, EXOSK_CAM_REQUEST);
      MDF_LOGD("Node send, size: %d, data: %s", size, data);
      exosk_data_type.format = EXOSK_WIFI_FORMAT_JSON;
      memcpy(&data_type.custom, &exosk_data_type, sizeof(data_type.custom));
      ret = mwifi_write(parent_mac, &data_type, data, size, true);
      MDF_FREE(data);
      MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
      attempts++;
      vTaskDelay(1000);
    }
  }

  MDF_FREE(data);
  MDF_LOGW("Node task is exit");

  vTaskDelete(NULL);
}

static void print_system_info_timercb(void *timer) {
  uint8_t primary = 0;
  wifi_second_chan_t second = 0;
  mesh_addr_t parent_bssid = {0};
  uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
  mesh_assoc_t mesh_assoc = {0x0};
  wifi_sta_list_t wifi_sta_list = {0x0};

  esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  esp_wifi_get_channel(&primary, &second);
  esp_wifi_vnd_mesh_get(&mesh_assoc);
  esp_mesh_get_parent_bssid(&parent_bssid);

  MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
           ", parent rssi: %d, node num: %d, free heap: %u, my_type: %i",
           primary, esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr), mesh_assoc.rssi, esp_mesh_get_total_node_num(),
           esp_get_free_heap_size(), esp_mesh_get_type());

  for (int i = 0; i < wifi_sta_list.num; i++) {
    MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
  }

#ifdef MEMORY_DEBUG

  if (!heap_caps_check_integrity_all(true)) {
    MDF_LOGE("At least one heap is corrupt");
  }

  mdf_mem_print_heap();
  mdf_mem_print_record();
  mdf_mem_print_task();
#endif
}

void app_main(void) {
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set(TAG, ESP_LOG_DEBUG);

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << GPIO_LED_3);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);
  gpio_set_level(GPIO_LED_3, 0);

  startup_handle();

  init_camera();
  init_sd();
  init_mesh();

  xTaskCreate(node_write_task, "node_write_task", 16 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
  xTaskCreate(node_read_task, "node_read_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

  TimerHandle_t timer = xTimerCreate("print_system_info", 2000 / portTICK_RATE_MS, true, NULL, print_system_info_timercb);
  xTimerStart(timer, 0);
}
