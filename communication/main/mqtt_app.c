#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT_COM";
static esp_mqtt_client_handle_t s_client = NULL;

void mqtt_app_start(void) {
  esp_mqtt_client_config_t mqtt_config = {
    .uri = CONFIG_BROKER_URL,
  };

  s_client = esp_mqtt_client_init(&mqtt_config);

  esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, s_client);
  esp_mqtt_client_start(s_client);

}