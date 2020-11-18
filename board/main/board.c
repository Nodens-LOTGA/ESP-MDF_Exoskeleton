#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "driver/uart.h"
#include "mdf_common.h"
#include "mwifi.h"

#include "../../exosk.h"
#include "nmea_parser.h"

//#define MAKE_JSON_ON_ROOT

#define YEAR_BASE 2000

#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024
#define MSG_BUFFER_SIZE 64

#define RPI_MISO 12
#define RPI_MOSI 13
#define RPI_SCLK 14
#define RPI_CS 15
#define RPI_HOST HSPI_HOST
#define RPI_DMA_CHAN 0

static QueueHandle_t uart_queue = NULL;
static nmea_parser_handle_t nmea_hdl = NULL;

static int g_sockfd = -1;
static const char *TAG = "exosk_board";
static esp_netif_t *netif_sta = NULL;

static board_config_t board_config;

static uint8_t screen_mac[MWIFI_ADDR_LEN];

static const char uart_config_request[] = "config;";

static const char uart_recive_config[] = "config=";
static const char uart_recive_data[] = "data=";

static const char spi_data[] = "hr=%hhu;steps=%hu;";

static sensors_data_t sensors_data;
static gps_data_t gps_data;
static band_data_t band_data;

#define JSON_FORMAT_ARGS(x, mac)                                                                                                                               \
  MAC2STR(mac), x.exosk_id, x.screen_sensors_data.temperature, x.screen_sensors_data.illumination, x.screen_sensors_data.co, x.screen_sensors_data.co2,        \
      x.screen_sensors_data.ch4, x.screen_sensors_data.lpg, x.screen_sensors_data.humidity, x.screen_sensors_data.atmo_pressure,                               \
      x.screen_sensors_data.t_radiation, x.screen_sensors_data.smoke, x.screen_sensors_data.op_weight, x.screen_sensors_data.cargo_weight,                     \
      x.screen_sensors_data.battery_charge, x.gps_data.latitude, x.gps_data.longitude, x.gps_data.year, x.gps_data.month, x.gps_data.day, x.gps_data.hour,     \
      x.gps_data.minute, x.gps_data.second, x.band_data.num_of_step, x.band_data.heart_rate, x.vibration, x.dust, x.noise, x.num_of_lift, x.cpu_temperature,   \
      x.battery_voltage, x.total_weight
static const char json_format[] = "{\"src\":\"" MACSTR "\","
                                  "\"id\":%hu,"
                                  "\"data\":{"
                                  "\"temperature\":%f,"
                                  "\"illumination\":%f,"
                                  "\"co\":%hhu,"
                                  "\"co2\":%hhu,"
                                  "\"ch4\":%hhu,"
                                  "\"lpg\":%hhu,"
                                  "\"humidity\":%hhu,"
                                  "\"atmo_pressure\":%u,"
                                  "\"t_radiation\":%hhu,"
                                  "\"smoke\":%hhu,"
                                  "\"op_weight\":%f,"
                                  "\"cargp_weight\":%f,"
                                  "\"battery_charge\":%hhu,"
                                  "\"latitude\":%f,"
                                  "\"longitude\":%f,"
                                  "\"date_time\":\"%04hu-%02hhu-%02hhuT%02hhu:%02hhu:%02hhu\","
                                  "\"num_of_step\":%hu,"
                                  "\"heart_rate\":%hhu,"
                                  "\"vibration\":%hu,"
                                  "\"dust\":%hu,"
                                  "\"noise\":%hu,"
                                  "\"num_of_lifts\":%hu,"
                                  "\"cpu_temperature\":%f,"
                                  "\"battery_voltage\":%f,"
                                  "\"total_weight\":%u}}";

inline static int send_to_stm32(char *buffer, size_t len) {
  ESP_LOGI(TAG, "Send to STM32:");
  esp_log_buffer_hex(TAG, buffer, len);
  return uart_write_bytes(UART_NUM_2, buffer, len);
}

inline static int read_from_stm32(char *buffer, uint32_t len, TickType_t ticks_to_wait) {
  int size = uart_read_bytes(UART_NUM_2, (uint8_t *)buffer, len, ticks_to_wait);
  ESP_LOGI(TAG, "Read from STM32:");
  esp_log_buffer_hex(TAG, buffer, len);
  return size;
}

static bool read_data_from_stm32(void) {
  uart_event_t event;
  char buffer[RX_BUFFER_SIZE];
  char *pch = NULL;
  if (xQueueReceive(uart_queue, (void *)&event, 2 * board_config.gap / portTICK_RATE_MS)) {
    if (event.type == UART_DATA) {
      read_from_stm32(buffer, event.size, portMAX_DELAY);
      pch = strstr(buffer, uart_recive_data);
      if (pch != NULL)
        if (pch[sizeof(uart_recive_data) - 1 + sizeof(sensors_data_t)] == ';') {
          memcpy(&sensors_data, pch + sizeof(uart_recive_data) - 1, sizeof(sensors_data_t));
          return true;
        }
    }
  }
  return false;
}

inline static mdf_err_t write_while_enomem(int s, const void *dataptr, size_t len) {
  mdf_err_t ret;
  while (true) {
    ret = write(s, dataptr, len);
    if (errno == ENOMEM)
      vTaskDelay(10 / portTICK_RATE_MS);
    else
      break;
  }
  return ret;
}

static mdf_err_t wifi_init(void) {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  MDF_ERROR_ASSERT(esp_netif_init());
  MDF_ERROR_ASSERT(esp_event_loop_create_default());
  ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
  MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
  MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
  MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
  MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
  MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
  MDF_ERROR_ASSERT(esp_wifi_start());

  return MDF_OK;
}

static int socket_udp_client_create(const char *ip, uint16_t port) {
  MDF_PARAM_CHECK(ip);

  MDF_LOGI("Create a UDP client, ip: %s, port: %d", ip, port);

  mdf_err_t ret = ESP_OK;
  int sockfd = -1;
  struct sockaddr_in server_addr = {
      .sin_family = AF_INET,
      .sin_port = htons(port),
      .sin_addr.s_addr = inet_addr(ip),
  };

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  MDF_ERROR_GOTO(sockfd < 0, ERR_EXIT, "socket create, sockfd: %d", sockfd);

  ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
  MDF_ERROR_GOTO(ret < 0, ERR_EXIT, "socket connect, ret: %d, ip: %s, port: %d", ret, ip, port);
  return sockfd;

ERR_EXIT:

  if (sockfd != -1) {
    close(sockfd);
  }

  return -1;
}

void udp_client_write_task(void *arg) {
  mdf_err_t ret = MDF_OK;
  char *data = MDF_CALLOC(1, MWIFI_PAYLOAD_LEN);
  char *udp_data = MDF_CALLOC(1, 2 * MWIFI_PAYLOAD_LEN);
  size_t size = MWIFI_PAYLOAD_LEN;
  size_t udp_size = 0;
  uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
  mwifi_data_type_t data_type = {0x0};
  exosk_wifi_data_type_t exosk_data_type = {0x0};

#ifdef MAKE_JSON_ON_ROOT
  sensors_data_t *d = NULL;
#endif

  MDF_LOGI("UDP client write task is running");

  while (mwifi_is_connected()) {
    if (g_sockfd == -1) {
      g_sockfd = socket_udp_client_create(board_config.ip, board_config.port);

      if (g_sockfd == -1) {
        vTaskDelay(500 / portTICK_RATE_MS);
        continue;
      }
    }

    size = MWIFI_PAYLOAD_LEN;
    udp_size = 2 * MWIFI_PAYLOAD_LEN;
    memset(data, 0, MWIFI_PAYLOAD_LEN);
    memset(udp_data, 0, 2 * MWIFI_PAYLOAD_LEN);
    ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
    MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
    memcpy(&exosk_data_type, &data_type.custom, sizeof(data_type.custom));
    switch (exosk_data_type.format) {
    case EXOSK_WIFI_FORMAT_JPEG:
      udp_size = sprintf(udp_data, "img:%d:%d:", exosk_data_type.exosk_id, exosk_data_type.data);
      memcpy(udp_data + udp_size, data, size);
      udp_size += size;
      udp_size += sprintf(udp_data + udp_size, ":img");
      MDF_LOGI("UDP jpeg write, img:%d:%d:<%d bytes>:img", exosk_data_type.exosk_id, exosk_data_type.data, udp_size);
      esp_log_buffer_hex(TAG, udp_data, udp_size);
      ret = write(g_sockfd, udp_data, udp_size);
      break;
    case EXOSK_WIFI_FORMAT_JSON:
      MDF_LOGI("UDP json write, size: %d, data: %s", size, data);
      ret = write(g_sockfd, data, size);
      break;
    case EXOSK_WIFI_FORMAT_DATA:
#ifdef MAKE_JSON_ON_ROOT
      d = (sensors_data_t *)data;
      udp_size = snprintf(udp_data, udp_size, json_format, JSON_FORMAT_ARGS((*d), src_addr));
      MDF_LOGI("UDP json from data write, size: %d, data: %s", udp_size, udp_data);
#else
      MDF_LOGI("UDP data write, size: %d, data:", udp_size);
      esp_log_buffer_hex(TAG, udp_data, udp_size);
#endif
      ret = write(g_sockfd, udp_data, udp_size);
      break;
    }
    if (ret <= 0) {
      MDF_LOGW("<%s> UDP write", strerror(errno));
      close(g_sockfd);
      g_sockfd = -1;
      continue;
    }
  }

  MDF_LOGI("UDP client write task is exit");

  close(g_sockfd);
  g_sockfd = -1;
  MDF_FREE(udp_data);
  MDF_FREE(data);
  vTaskDelete(NULL);
}

static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx) {
  MDF_LOGI("event_loop_cb, event: %d", event);

  switch (event) {
  case MDF_EVENT_MWIFI_STARTED:
    MDF_LOGI("MESH is started");
    break;

  case MDF_EVENT_MWIFI_PARENT_CONNECTED:
    MDF_LOGI("Parent is connected on station interface");

    if (esp_mesh_is_root()) {
      esp_netif_dhcpc_start(netif_sta);
    }

    break;

  case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
    MDF_LOGI("Parent is disconnected on station interface");
    break;

  case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
  case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
    MDF_LOGI("total_num: %d", esp_mesh_get_total_node_num());
    break;

  case MDF_EVENT_MWIFI_ROOT_GOT_IP: {
    MDF_LOGI("Root obtains the IP address. It is posted by LwIP stack "
             "automatically");
    xTaskCreate(udp_client_write_task, "udp_client_write_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    break;
  }

  default:
    break;
  }

  return MDF_OK;
}

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  gps_t *gps = NULL;
  switch (event_id) {
  case GPS_UPDATE:
    gps = (gps_t *)event_data;
    ESP_LOGI(TAG,
             "%d/%d/%d %d:%d:%d => \r\n"
             "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
             "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
             "\t\t\t\t\t\taltitude   = %.02fm\r\n"
             "\t\t\t\t\t\tspeed      = %fm/s",
             gps->date.year + YEAR_BASE, gps->date.month, gps->date.day, gps->tim.hour + board_config.time_zone, gps->tim.minute, gps->tim.second,
             gps->latitude, gps->longitude, gps->altitude, gps->speed);
    gps_data.latitude = gps->altitude;
    gps_data.longitude = gps->longitude;
    gps_data.day = gps->date.day;
    gps_data.month = gps->date.month;
    gps_data.year = gps->date.year;
    gps_data.hour = gps->tim.hour + board_config.time_zone;
    gps_data.minute = gps->tim.minute;
    gps_data.second = gps->tim.second;
    break;
  case GPS_UNKNOWN:
    ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
    break;
  default:
    break;
  }
}

static void init_uart(void) {
  // strcpy(board_config.router_ssid, "RPI");
  // strcpy(board_config.router_password, "43215678");
  // board_config.exosk_id = 1;
  // strcpy(board_config.ip, "192.168.254.3");
  // board_config.port = 55555;
  // board_config.time_zone = 4;
  // board_config.gap = 5000;

  // ESP_LOGI(TAG, "Config: ");
  // esp_log_buffer_hex(TAG, &board_config, sizeof(board_config));

  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 2 * RX_BUFFER_SIZE, 2 * TX_BUFFER_SIZE, 16, &uart_queue, 0));

  uart_event_t event;
  char *buffer = MDF_MALLOC(RX_BUFFER_SIZE);
  char *pch = NULL;
  int len = 0;

  memset(buffer, 0, RX_BUFFER_SIZE);
  len = sprintf(buffer, uart_config_request);
  send_to_stm32(buffer, len);
  while (1) {
    memset(buffer, 0, RX_BUFFER_SIZE);
    if (xQueueReceive(uart_queue, (void *)&event, 10000 / portTICK_RATE_MS)) {
      if (event.type == UART_DATA) {
        read_from_stm32(buffer, event.size, portMAX_DELAY);
        pch = strstr(buffer, uart_recive_config);
        if (pch != NULL)
          if (pch[sizeof(uart_recive_config) - 1 + sizeof(board_config_t)] == ';')
            memcpy(&board_config, pch + sizeof(uart_recive_config) - 1, sizeof(board_config_t));
        if (strlen(board_config.router_ssid) != 0 && strlen(board_config.router_password) != 0 && board_config.exosk_id != 0 && strlen(board_config.ip) != 0 &&
            board_config.port != 0) {
          len = sprintf(buffer, uart_config_request);
          break;
        }
      }
    } else {
      len = sprintf(buffer, uart_config_request);
      send_to_stm32(buffer, len);
    }
  }

  ESP_LOGI(TAG, "Router SSID: %s", board_config.router_ssid);
  ESP_LOGI(TAG, "Router Password: %s", board_config.router_password);
  ESP_LOGI(TAG, "Exoskeleton ID: %hu", board_config.exosk_id);
  ESP_LOGI(TAG, "IP: %s", board_config.ip); //
  ESP_LOGI(TAG, "Port: %hu", board_config.port);
  ESP_LOGI(TAG, "Timezone: %hhi", board_config.time_zone);
  ESP_LOGI(TAG, "Gap: %u", board_config.gap);
  MDF_FREE(buffer);
}

static void init_spi(void) {
  spi_bus_config_t buscfg = {
      .mosi_io_num = RPI_MOSI,
      .miso_io_num = RPI_MISO,
      .sclk_io_num = RPI_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };
  spi_slave_interface_config_t slvcfg = {.mode = 0, .spics_io_num = RPI_CS, .queue_size = 3, .flags = 0};

  gpio_set_pull_mode(RPI_MOSI, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(RPI_SCLK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(RPI_CS, GPIO_PULLUP_ONLY);

  ESP_ERROR_CHECK(spi_slave_initialize(RPI_HOST, &buscfg, &slvcfg, RPI_DMA_CHAN));
}

static void init_mesh(void) {
  mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
  mwifi_config_t config = {
      .mesh_id = CONFIG_MESH_ID,
      .mesh_password = CONFIG_MESH_PASSWORD,
  };
  strcpy(config.router_ssid, board_config.router_ssid);
  strcpy(config.router_password, board_config.router_password);
  MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
  MDF_ERROR_ASSERT(wifi_init());
  MDF_ERROR_ASSERT(mwifi_init(&cfg));
  MDF_ERROR_ASSERT(mwifi_set_config(&config));

  const uint8_t group_id_list[MWIFI_ADDR_LEN] = EXOSK_GROUP_ID_BOARD;
  MDF_ERROR_ASSERT(esp_mesh_set_group_id((mesh_addr_t *)group_id_list, sizeof(group_id_list) / sizeof(group_id_list[0])));

  MDF_ERROR_ASSERT(mwifi_start());
}

static void init_gps(void) {
  nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
  nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
  ESP_ERROR_CHECK(nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL));
}

static void node_read_task(void *arg) {
  mdf_err_t ret = MDF_OK;
  char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
  size_t size = MWIFI_PAYLOAD_LEN;
  mwifi_data_type_t data_type = {0x0};
  exosk_wifi_data_type_t exosk_data_type = {0x0};
  uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
  float ref_weight = 0;

  MDF_LOGI("Note read task is running");

  for (;;) {
    if (!mwifi_is_connected()) {
      vTaskDelay(500 / portTICK_RATE_MS);
      continue;
    }

    size = MWIFI_PAYLOAD_LEN;
    memset(data, 0, MWIFI_PAYLOAD_LEN);
    ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
    MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
    MDF_LOGI("Node receive: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
    memcpy(&exosk_data_type, &data_type.custom, sizeof(data_type.custom));
    if (exosk_data_type.format == EXOSK_WIFI_FORMAT_JSON) {
      exosk_data_type.exosk_id = board_config.exosk_id;
      exosk_data_type.type = EXOSK_WIFI_TYPE_BOARD;
      memcpy(&data_type.custom, &exosk_data_type, sizeof(data_type.custom));
      if (strcmp(data, EXOSK_CAM_REQUEST) == 0) {
        strcpy(data, EXOSK_TO_CAM_RESPONSE);
        ESP_LOGI(TAG, "Send ID to camera: %d", board_config.exosk_id);
        ret = mwifi_write(src_addr, &data_type, data, strlen(data), true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
      } else if (strcmp(data, EXOSK_SCREEN_REQUEST) == 0) {
        if (screen_mac[0] == 0)
          memcpy(screen_mac, src_addr, MWIFI_ADDR_LEN);
        if (memcmp(src_addr, screen_mac, MWIFI_ADDR_LEN) == 0) {
          strcpy(data, EXOSK_TO_SCREEN_RESPONSE);
          ESP_LOGI(TAG, "Send ID to screen: %d", board_config.exosk_id);
          ret = mwifi_write(src_addr, &data_type, data, strlen(data), true);
          MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
        } else {
          strcpy(data, EXOSK_SCREEN_EXISTS_ERROR);
          ESP_LOGI(TAG, "Send screen exists error to screen");
          ret = mwifi_write(src_addr, &data_type, data, strlen(data), true);
          MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
        }
      } else if (strcmp(data, EXOSK_CALIBRATE_STRAIN_ZERO) == 0 || sscanf(data, EXOSK_CALIBRATE_STRAIN_REFERENCE, &ref_weight) == 1) {
        ESP_LOGI(TAG, "Send strain calibrate request to stm32");
        send_to_stm32(data, size);
      }
    }
  }

  MDF_LOGW("Note read task is exit");

  MDF_FREE(data);
  vTaskDelete(NULL);
}

static void node_write_task(void *arg) {
  size_t len = 0;
  char *buffer = MDF_MALLOC(2 * MWIFI_PAYLOAD_LEN);
  mdf_err_t ret = MDF_OK;
  mwifi_data_type_t data_type = {0};
  exosk_wifi_data_type_t exosk_data_type = {.type = EXOSK_WIFI_TYPE_BOARD, .format = EXOSK_WIFI_FORMAT_DATA};
  uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};

  MDF_LOGI("Node task is running");

  esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
  for (;;) {
    if (!mwifi_is_connected()) {
      vTaskDelay(500 / portTICK_RATE_MS);
      continue;
    }
    memset(buffer, 0, 2 * MWIFI_PAYLOAD_LEN);
    len = sprintf(buffer, "gps=");
    memcpy(buffer + len, &gps_data, sizeof(gps_data_t));
    len += sizeof(gps_data_t);
    buffer[len++] = ';';
    len += sprintf(buffer + len, "band=");
    memcpy(buffer + len, &band_data, sizeof(band_data_t));
    len += sizeof(band_data_t);
    buffer[len++] = ';';
    send_to_stm32(buffer, len);
    if (read_data_from_stm32()) {
      if (screen_mac[0] != 0) {
        exosk_data_type.format = EXOSK_WIFI_FORMAT_DATA;
        memcpy(&data_type.custom, &exosk_data_type, sizeof(data_type.custom));
        len = sizeof(sensors_data.screen_sensors_data);
        MDF_LOGI("Send data to screen. Data:");
        esp_log_buffer_hex(TAG, (char *)&sensors_data.screen_sensors_data, len);
        ret = mwifi_write(screen_mac, &data_type, &sensors_data.screen_sensors_data, len, true);
        if (ret != MDF_OK)
          memset(screen_mac, 0, MWIFI_ADDR_LEN);
      }
#ifdef MAKE_JSON_ON_ROOT
      exosk_data_type.format = EXOSK_WIFI_FORMAT_DATA;
      memcpy(&data_type.custom, &exosk_data_type, sizeof(data_type.custom));
      len = sizeof(sensors_data);
      MDF_LOGI("Send data to root. Data:");
      esp_log_buffer_hex(TAG, (char *)&sensors_data, len);
      ret = mwifi_write(NULL, &data_type, &sensors_data, len, true);
      MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
#else
      exosk_data_type.format = EXOSK_WIFI_FORMAT_JSON;
      memcpy(&data_type.custom, &exosk_data_type, sizeof(data_type.custom));
      len = snprintf(buffer, 2 * MWIFI_PAYLOAD_LEN, json_format, JSON_FORMAT_ARGS(sensors_data, sta_mac));
      MDF_LOGI("Send json to root. Data:\n%s", buffer);
      ret = mwifi_write(NULL, &data_type, buffer, len, true);
      MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
#endif
    }
  }

  MDF_FREE(buffer);
  MDF_LOGW("Node task is exit");

  vTaskDelete(NULL);
}

static void rpi_task(void *arg) {
  char sendbuf[32] = "";
  char recvbuf[32] = "";
  spi_slave_transaction_t t = {0};
  while (1) {
    memset(recvbuf, 0, 32);
    sprintf(sendbuf, "%04hu-%02hhu-%02hhuT%02hhu:%02hhu:%02hhu;", gps_data.year, gps_data.month, gps_data.day,
            gps_data.hour, gps_data.minute, gps_data.second);
    memset(&t, 0, sizeof(t));
    t.length = 32 * 8;
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;
    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_slave_transmit(RPI_HOST, &t, portMAX_DELAY));
    ESP_LOGI(TAG, "Received from RPI: %s", recvbuf);
    ESP_LOGI(TAG, "HEX:");
    esp_log_buffer_hex(TAG, recvbuf, 32);

    if (sscanf(recvbuf, spi_data, &band_data.heart_rate, &band_data.num_of_step) == 2) {
      ESP_LOGI(TAG, "RPI: Recive Data: HR %hhu, Steps %hu", band_data.heart_rate, band_data.num_of_step);
    }
    vTaskDelay(100 / portTICK_RATE_MS);
  }
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

  MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR ", parent rssi: %d, node num: %d, free heap: %u", primary,
           esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr), mesh_assoc.rssi, esp_mesh_get_total_node_num(), esp_get_free_heap_size());

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

  mdf_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    MDF_ERROR_ASSERT(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  MDF_ERROR_ASSERT(ret);

  init_uart();
  init_spi();
  init_mesh();
  // init_gps();

  xTaskCreate(node_write_task, "node_write_task", 16 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
  xTaskCreate(node_read_task, "node_read_task", 16 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
  xTaskCreate(rpi_task, "rpi_task", 16 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

  TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS, true, NULL, print_system_info_timercb);
  xTimerStart(timer, 0);
}
