#include "stdint.h"

#define EXOSK_GROUP_ID_BOARD {0x01, 0x00, 0x5e, 0x01, 0x01, 0x01}
#define EXOSK_GROUP_ID_CAM {0x01, 0x00, 0x5e, 0x01, 0x01, 0x02}
#define EXOSK_GROUP_ID_SCREEN  {0x01, 0x00, 0x5e, 0x01, 0x01, 0x03}
#define EXOSK_CAM_REQUEST "cam_req;"
#define EXOSK_TO_CAM_RESPONSE "my_id;"
#define EXOSK_SCREEN_REQUEST "screen_req;"
#define EXOSK_TO_SCREEN_RESPONSE "my_id;"
#define EXOSK_CALIBRATE_STRAIN_ZERO "calibrate_strain_zero;"
#define EXOSK_CALIBRATE_STRAIN_REFERENCE "calibrate_strain_reference=%f;"
#define EXOSK_SCREEN_EXISTS_ERROR "screen_exists;"

typedef enum { EXOSK_WIFI_TYPE_BOARD = 0, EXOSK_WIFI_TYPE_CAM, EXOSK_WIFI_TYPE_SCREEN } exosk_wifi_type_t;

typedef enum { EXOSK_WIFI_FORMAT_NONE = 0, EXOSK_WIFI_FORMAT_JSON, EXOSK_WIFI_FORMAT_JPEG, EXOSK_WIFI_FORMAT_DATA } exosk_wifi_format_t;

typedef struct {
  uint16_t exosk_id : 16;
  uint8_t type : 2;
  uint8_t format : 2;
  uint16_t data : 12;
} exosk_wifi_data_type_t;

typedef enum {EXOSK_ERROR_SD_CARD = 0, EXOSK_ERROR_CAMERA, EXOSK_ERROR_WIFI} exosk_error_t;

typedef struct {
  float temperature;
  float illumination;
  float op_weight;
  float cargo_weight;
  uint32_t atmo_pressure;
  uint8_t co;
  uint8_t co2;
  uint8_t ch4;
  uint8_t lpg;
  uint8_t humidity;
  uint8_t t_radiation;
  uint8_t smoke;
  uint8_t battery_charge;
} screen_sensors_data_t;

typedef struct {
  float latitude;
  float longitude;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;      
  uint8_t minute;    
  uint8_t second;  
} gps_data_t;

typedef struct {
  uint16_t num_of_step;
  uint8_t heart_rate;
} band_data_t;

typedef struct {
  screen_sensors_data_t screen_sensors_data;
  gps_data_t gps_data;
  band_data_t band_data;
  float cpu_temperature;
  float battery_voltage;
  uint32_t total_weight; 
  uint16_t vibration;
  uint16_t dust;
  uint16_t num_of_lift;
  uint16_t exosk_id;
  uint16_t noise;
} sensors_data_t;

typedef struct {
  char router_ssid[33];
  char router_password[33];
  char ip[16];
  uint16_t port;
  uint16_t exosk_id;
  uint32_t gap;
  int8_t time_zone;
} board_config_t;

typedef struct {
  char router_ssid[33];
  char router_password[33];
  uint8_t parent_mac[MWIFI_ADDR_LEN];
  uint16_t exosk_id;
} screen_config_t;