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
#include "esp_system.h"
#include "mdf_common.h"
#include "mwifi.h"

#include "../../exosk.h"

#define REQUEST_RETRIES_COUNT 25

//#define MEMORY_DEBUG

#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024
#define MSG_BUFFER_SIZE 64 + 3

static QueueHandle_t uart_queue, node_write_queue;

static const char *TAG = "exosk_screen";
static esp_netif_t *sta_netif = NULL;
static bool parent_responded = false;

static screen_config_t screen_config;

static const char uart_config_request[] = "global.config_req.val=%hhu";
static const char uart_mac_error[] = "global.error_mac.val=%hhu";
static const char uart_mesh_error[] = "global.error_mesh.val=%hhu";
static const char uart_exists_error[] = "global.error_exists.val=%hhu";
static const char uart_disconnect_error[] = "global.error_disconnect.val=%hhu";

static const char uart_send_mac[] = "global.mac_number.txt=\"%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx\"";
static const char uart_send_ssid[] = "global.ssid_number.txt=\"%s\"";
static const char uart_send_pass[] = "global.pass_number.txt=\"%s\"";
static const char uart_send_exosk_id[] = "global.exosk_id.txt=\"%hu\"";

static const char uart_recive_mac[] = "mac=%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx";
static const char uart_recive_ssid[] = "ssid=%s";
static const char uart_recive_pass[] = "pass=%s";
static const char uart_recive_reboot[] = "reboot";
static const char uart_recive_erase_config[] = "erase_config";

static const char uart_data_temperature[] = "global.temperature.txt=\"%.1f\"";
static const char uart_data_illumination[] = "global.illumination.txt=\"%.1f\"";
static const char uart_data_co[] = "global.co.txt=\"%d\"";
static const char uart_data_co2[] = "global.co2.txt=\"%d\"";
static const char uart_data_ch4[] = "global.ch4.txt=\"%d\"";
static const char uart_data_lpg[] = "global.lpg.txt=\"%d\"";
static const char uart_data_humidity[] = "global.humidity.txt=\"%d\"";
static const char uart_data_atmo_pressure[] = "global.atmo_pressure.txt=\"%d\"";
static const char uart_data_t_radiation[] = "global.t_radiation.txt=\"%d\"";
static const char uart_data_smoke[] = "global.smoke.txt=\"%d\"";
static const char uart_data_op_weight[] = "global.op_weight.txt=\"%.1f\"";
static const char uart_data_cargo_weight[] = "global.cargo_weight.txt=\"%.1f\"";
static const char uart_data_battery_charge[] = "global.battery_charge.txt=\"%d\"";

static screen_sensors_data_t sensors_data;

inline static esp_err_t save_config(void) { return mdf_info_save(TAG, &screen_config, sizeof(screen_config_t)); }

inline static int send_to_screen(char *buffer, size_t len) {
  memset(buffer + len, 0xFF, 3);
  ESP_LOGI(TAG, "Send to Screen: %s", buffer);
  return uart_write_bytes(UART_NUM_2, buffer, len + 3);
}

inline static int read_from_screen(char *buffer, uint32_t len, TickType_t ticks_to_wait) {
  int size = uart_read_bytes(UART_NUM_2, (uint8_t *)buffer, len, ticks_to_wait);
  ESP_LOGI(TAG, "Read from Screen: %s", buffer);
  return size;
}

static void send_data_to_screen(void) {
  char buffer[MSG_BUFFER_SIZE];
  size_t len = 0;
  len = sprintf(buffer, uart_data_temperature, sensors_data.temperature);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_illumination, sensors_data.illumination);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_co, sensors_data.co);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_co2, sensors_data.co2);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_ch4, sensors_data.ch4);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_lpg, sensors_data.lpg);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_humidity, sensors_data.humidity);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_atmo_pressure, sensors_data.atmo_pressure);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_t_radiation, sensors_data.t_radiation);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_smoke, sensors_data.smoke);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_op_weight, sensors_data.op_weight);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_cargo_weight, sensors_data.cargo_weight);
  send_to_screen(buffer, len);
  len = sprintf(buffer, uart_data_battery_charge, sensors_data.battery_charge);
  send_to_screen(buffer, len);
}

static mdf_err_t wifi_init(void) {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

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
  char *buffer = NULL;
  size_t len = 0;

  MDF_LOGI("event_loop_cb, event: %d", event);

  switch (event) {
  case MDF_EVENT_MWIFI_STARTED:
    MDF_LOGI("MESH is started");
    break;

  case MDF_EVENT_MWIFI_STOPPED:
    MDF_LOGI("MESH is stopped");
    break;

  case MDF_EVENT_MWIFI_PARENT_CONNECTED: {
    parent_responded = false;
    buffer = MDF_MALLOC(32);
    len = sprintf(buffer, uart_mesh_error, 0);
    send_to_screen(buffer, len);
    len = sprintf(buffer, uart_disconnect_error, 0);
    send_to_screen(buffer, len);
    MDF_FREE(buffer);
    break;
  }

  case MDF_EVENT_MWIFI_NO_PARENT_FOUND:
    MDF_LOGI("Parent not found");
    buffer = MDF_MALLOC(32);
    len = sprintf(buffer, uart_mesh_error, 1);
    send_to_screen(buffer, len);
    MDF_FREE(buffer);
    break;

  case MDF_EVENT_MWIFI_PARENT_DISCONNECTED: {
    mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)ctx;
    MDF_LOGI("<MESH_EVENT_PARENT_DISCONNECTED>reason:%d", disconnected->reason);
    buffer = MDF_MALLOC(32);
    len = sprintf(buffer, uart_disconnect_error, 1);
    send_to_screen(buffer, len);
    MDF_FREE(buffer);
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
  strcpy(config.router_ssid, screen_config.router_ssid);
  strcpy(config.router_password, screen_config.router_password);
  MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
  MDF_ERROR_ASSERT(wifi_init());
  MDF_ERROR_ASSERT(mwifi_init(&cfg));
  MDF_ERROR_ASSERT(mwifi_set_config(&config));

  const uint8_t group_id_list[MWIFI_ADDR_LEN] = EXOSK_GROUP_ID_SCREEN;
  MDF_ERROR_ASSERT(esp_mesh_set_group_id((mesh_addr_t *)group_id_list, sizeof(group_id_list) / sizeof(group_id_list[0])));

  MDF_ERROR_ASSERT(mwifi_start());
}

static void init_uart(void) {
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 2 * RX_BUFFER_SIZE, 2 * TX_BUFFER_SIZE, 10, &uart_queue, 0));

  bool cfg_found = true;
  mdf_info_init();
  if (mdf_info_load(TAG, &screen_config, sizeof(screen_config_t)) == ESP_ERR_NVS_NOT_FOUND) {
    cfg_found = false;
    MDF_LOGI("Config not found");
  }

  uart_event_t event;
  char *buffer = MDF_MALLOC(RX_BUFFER_SIZE);
  char *msg = MDF_MALLOC(MSG_BUFFER_SIZE);
  char *ptkn = NULL;
  int len = 0;

  memset(buffer, 0, RX_BUFFER_SIZE);
  if (!cfg_found) {
    len = sprintf(buffer, uart_config_request, 1);
    send_to_screen(buffer, len);
    while (!cfg_found) {
      memset(buffer, 0, RX_BUFFER_SIZE);
      if (xQueueReceive(uart_queue, (void *)&event, 60000 / portTICK_RATE_MS)) {
        if (event.type == UART_DATA) {
          read_from_screen(buffer, event.size, portMAX_DELAY);
          ptkn = strtok(buffer, ";");
          while (ptkn != NULL) {
            memset(msg, 0, MSG_BUFFER_SIZE);
            strcpy(msg, ptkn);
            if (sscanf(msg, uart_recive_mac, &screen_config.parent_mac[0], &screen_config.parent_mac[1], &screen_config.parent_mac[2],
                       &screen_config.parent_mac[3], &screen_config.parent_mac[4], &screen_config.parent_mac[5]) == 6 ||
                sscanf(msg, uart_recive_ssid, screen_config.router_ssid) == 1 || sscanf(msg, uart_recive_pass, screen_config.router_password) == 1)
              save_config();
            ptkn = strtok(NULL, ";");
          }
          if (screen_config.parent_mac[0] != 0 && strlen(screen_config.router_ssid) != 0 && strlen(screen_config.router_password) != 0) {
            cfg_found = 1;
            len = sprintf(buffer, uart_config_request, 0);
            send_to_screen(buffer, len);
            break;
          }
        }
      } else {
        len = sprintf(buffer, uart_config_request, 1);
        send_to_screen(buffer, len);
      }
    }
    save_config();
  } else {
    len = sprintf(buffer, uart_send_mac, MAC2STR(screen_config.parent_mac));
    send_to_screen(buffer, len);
    len = sprintf(buffer, uart_send_ssid, screen_config.router_ssid);
    send_to_screen(buffer, len);
    len = sprintf(buffer, uart_send_pass, screen_config.router_password);
    send_to_screen(buffer, len);
  }
  ESP_LOGI(TAG, "Router SSID: %s", screen_config.router_ssid);
  ESP_LOGI(TAG, "Router Password: %s", screen_config.router_password);
  ESP_LOGI(TAG, "Parent MAC: " MACSTR, MAC2STR(screen_config.parent_mac));
  MDF_FREE(buffer);
  MDF_FREE(msg);
}

static void uart_event_task(void *arg) {
  uart_event_t event;
  char *buffer = MDF_MALLOC(RX_BUFFER_SIZE);
  char *msg = MDF_MALLOC(MSG_BUFFER_SIZE);
  char *ptkn = NULL;
  float ref_weight = 0;

  for (;;) {
    if (xQueueReceive(uart_queue, (void *)&event, (portTickType)portMAX_DELAY)) {
      memset(buffer, 0, RX_BUFFER_SIZE);
      ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_2);
      switch (event.type) {
      case UART_DATA:
        ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
        read_from_screen(buffer, event.size, portMAX_DELAY);

        ptkn = strtok(buffer, ";");
        while (ptkn != NULL) {
          memset(msg, 0, MSG_BUFFER_SIZE);
          strcpy(msg, ptkn);
          ESP_LOGI(TAG, "Token: %s", msg);
          if (strcmp(msg, EXOSK_CALIBRATE_STRAIN_ZERO) == 0 || sscanf(msg, EXOSK_CALIBRATE_STRAIN_REFERENCE, &ref_weight) == 1)
            xQueueSend(node_write_queue, msg, portMAX_DELAY);
          else if (sscanf(msg, uart_recive_mac, &screen_config.parent_mac[0], &screen_config.parent_mac[1], &screen_config.parent_mac[2],
                          &screen_config.parent_mac[3], &screen_config.parent_mac[4], &screen_config.parent_mac[5]) == 6 ||
                   sscanf(msg, uart_recive_ssid, screen_config.router_ssid) == 1 || sscanf(msg, uart_recive_pass, screen_config.router_password) == 1)
            save_config();
          else if (strcmp(msg, uart_recive_reboot) == 0)
            esp_restart();
          else if (strcmp(msg, uart_recive_erase_config) == 0)
            mdf_info_erase(TAG);
          ptkn = strtok(NULL, ";");
        }
        break;
      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow");
        uart_flush_input(UART_NUM_2);
        xQueueReset(uart_queue);
        break;
      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full");
        uart_flush_input(UART_NUM_2);
        xQueueReset(uart_queue);
        break;
      case UART_BREAK:
        ESP_LOGI(TAG, "uart rx break");
        break;
      case UART_PARITY_ERR:
        ESP_LOGI(TAG, "uart parity error");
        break;
      case UART_FRAME_ERR:
        ESP_LOGI(TAG, "uart frame error");
        break;
      default:
        ESP_LOGI(TAG, "uart event type: %d", event.type);
        break;
      }
    }
  }
  MDF_FREE(buffer);
  MDF_FREE(msg);
  vTaskDelete(NULL);
}

static void node_read_task(void *arg) {
  mdf_err_t ret = MDF_OK;
  char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
  size_t size = MWIFI_PAYLOAD_LEN;
  mwifi_data_type_t data_type = {0x0};
  exosk_wifi_data_type_t exosk_data_type = {0x0};
  uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
  char buffer[MSG_BUFFER_SIZE];
  size_t len = 0;

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
    memcpy(&exosk_data_type, &data_type.custom, sizeof(data_type.custom));
    MDF_LOGD("Type: %d, Format: %d", exosk_data_type.type, exosk_data_type.format);
    MDF_LOGD("Parent MAC: " MACSTR, MAC2STR(screen_config.parent_mac));
    if (memcmp(src_addr, screen_config.parent_mac, MWIFI_ADDR_LEN) == 0) {
      if (exosk_data_type.type == EXOSK_WIFI_TYPE_BOARD) {
        if (exosk_data_type.format == EXOSK_WIFI_FORMAT_JSON) {
          if (strcmp(data, EXOSK_TO_SCREEN_RESPONSE) == 0) {
            MDF_LOGD("Parent found");
            screen_config.exosk_id = exosk_data_type.exosk_id;
            ESP_LOGI(TAG, "Exoskeleton ID: %d", screen_config.exosk_id);
            parent_responded = true;
            len = sprintf(buffer, uart_exists_error, 0);
            send_to_screen(buffer, len);
            len = sprintf(buffer, uart_mac_error, 0);
            send_to_screen(buffer, len);
            len = sprintf(buffer, uart_send_exosk_id, screen_config.exosk_id);
            send_to_screen(buffer, len);
          } else if (strcmp(data, EXOSK_SCREEN_EXISTS_ERROR)) {
            parent_responded = false;
            len = sprintf(buffer, uart_exists_error, 1);
            send_to_screen(buffer, len);
          }
        } else if (exosk_data_type.format == EXOSK_WIFI_FORMAT_DATA) {
          memcpy(&sensors_data, data, sizeof(sensors_data));
          send_data_to_screen();
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
  size_t len = 0;
  char buffer[MSG_BUFFER_SIZE];
  mdf_err_t ret = MDF_OK;
  mwifi_data_type_t data_type = {0};
  exosk_wifi_data_type_t exosk_data_type = {.type = EXOSK_WIFI_TYPE_SCREEN};
  uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
  uint8_t attempts = 1;

  node_write_queue = xQueueCreate(1, MSG_BUFFER_SIZE * sizeof(char));

  MDF_LOGI("Node task is running");

  esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
  for (;;) {
    if (!mwifi_is_connected()) {
      vTaskDelay(500 / portTICK_RATE_MS);
      continue;
    }
    if (parent_responded) {
      memset(buffer, 0, MSG_BUFFER_SIZE);
      if (xQueueReceive(node_write_queue, buffer, 500 / portTICK_RATE_MS)) {
        exosk_data_type.exosk_id = screen_config.exosk_id;
        exosk_data_type.format = EXOSK_WIFI_FORMAT_JSON;
        exosk_data_type.data = 0;
        len = strlen(buffer);
        MDF_LOGD("Node send, size: %d, data: %s", len, buffer);
        ret = mwifi_write(screen_config.parent_mac, &data_type, buffer, len, true);
        if (ret != MDF_OK)
          parent_responded = false;
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
      }
    } else {
      len = sprintf(buffer, EXOSK_SCREEN_REQUEST);
      MDF_LOGD("Node send, size: %d, data: %s", len, buffer);
      exosk_data_type.format = EXOSK_WIFI_FORMAT_JSON;
      memcpy(&data_type.custom, &exosk_data_type, sizeof(data_type.custom));
      ret = mwifi_write(screen_config.parent_mac, &data_type, buffer, len, true);
      vTaskDelay(1000);
      if (attempts == REQUEST_RETRIES_COUNT) {
        len = sprintf(buffer, uart_mac_error, 1);
        send_to_screen(buffer, len);
        attempts = 1;
        vTaskDelay(30000 / portTICK_RATE_MS);
      }
      attempts++;
      MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
    }
  }

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
#endif /**< MEMORY_DEBUG */
}

void app_main(void) {
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set(TAG, ESP_LOG_DEBUG);

  init_uart();
  init_mesh();

  xTaskCreate(uart_event_task, "uart_event_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY + 1, NULL);
  xTaskCreate(node_write_task, "node_write_task", 16 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
  xTaskCreate(node_read_task, "node_read_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

  TimerHandle_t timer = xTimerCreate("print_system_info", 2000 / portTICK_RATE_MS, true, NULL, print_system_info_timercb);
  xTimerStart(timer, 0);
}
