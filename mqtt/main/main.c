#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h" //broadcast events
#include "esp_wifi.h" // wifi
#include "nvs_flash.h" //access to the permanent memory
#include "mqtt_client.h"
#include "dht11.h"


//Log
#define TAG_WIFI "WIFI"
#define TAG_MQTT "MQTT"
#define TAG_LED "LED"
#define TAG_SENSOR "SENSOR"

//WIFI
#define WIFI_SSID   "lababerto"
#define WIFI_PASS   "lababerto"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAXIMUM_RETRY  10

//MQTT
#define BROKER_URL "mqtt://broker.hivemq.com"

//Led
#define LED_BLUE_BOARD 2
#define LED_RED 23
#define DATA_SENSOR 22
#define PINS_SEL  ((1ULL<<LED_BLUE_BOARD) | (1ULL<<LED_RED))

//Topics
#define BOOTCAMP_HEALTH "bootcamp/+/health"
#define LED_BLUE_SET "bootcamp/joana/led/blue/set"
#define LED_RED_SET "bootcamp/joana/led/red/set"
#define TEMPERATURE_SET "bootcamp/joana/sensor/temperature/status"
#define HUMIDITY_SET "bootcamp/joana/sensor/humidity/status"

//event queue
static EventGroupHandle_t eventGroup;
static int retryNum = 0; //static store on the static memory
static struct MQTTMessage message;
static esp_mqtt_client_handle_t client;

struct MQTTMessage {
    char payload[1024];
    char topic[128];
};

void setLed(int led_pin, int value){
    gpio_set_level(led_pin, value);
}

void processMessage(struct MQTTMessage message) {
    ESP_LOGI(TAG_LED, "My message topic(%s) payload(%s)", message.topic, message.payload);
    if(strcmp(message.topic, LED_BLUE_SET)==0){
        setLed(LED_BLUE_BOARD, atoi(message.payload));
    }
    if(strcmp(message.topic, LED_RED_SET)==0){
        setLed(LED_RED, atoi(message.payload));
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data) {

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retryNum < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            retryNum++;
            ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
        } else {
            ESP_LOGI(TAG_WIFI,"connect to the AP fail");
            xEventGroupSetBits(eventGroup, WIFI_FAIL_BIT);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        retryNum = 0;
        xEventGroupSetBits(eventGroup, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data) {

    ESP_LOGI(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%d", event_base, event_id);

    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_publish(client, "bootcamp/joana/health", "ON", 0, 0, 0);
        esp_mqtt_client_subscribe(client, LED_BLUE_SET, 0);
        esp_mqtt_client_subscribe(client, LED_RED_SET, 0);
        esp_mqtt_client_subscribe(client, BOOTCAMP_HEALTH, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG_MQTT, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
        ESP_LOGI(TAG_MQTT, "DATA=%.*s\r\n", event->data_len, event->data);
        if(event->data_len < sizeof(message.payload) && event -> topic_len < sizeof(message.topic)){
            strncpy(message.payload, event->data, event->data_len);
            strncpy(message.topic, event->topic, event->topic_len);
            message.payload[event->data_len] = '\0';
            message.topic[event->topic_len] = '\0';
            processMessage(message);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG_MQTT,"reported from esp-tls %d", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(TAG_MQTT,"reported from tls stack %d", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(TAG_MQTT,"captured as transport's socket errno %d",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGE(TAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}

void initWifi(void)
{
    ESP_LOGI(TAG_WIFI, "Start wifi config connection");
    ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK verifica erros e reinicializa
    esp_netif_create_default_wifi_sta();

    //hardware config for wifi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    //instancia de estrutura de eventos a que vamos registar
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    //define that is a station and not ap
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_WIFI, "wifi config connection finished.");

    //fica à escuta dos eventos que definimos posteriormente - Vai ficar à espera pelos dois eventos, se nenhum destes eventos acontecer o código fica bloqueado
    EventBits_t bits = xEventGroupWaitBits(eventGroup,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(eventGroup);
}

void initMqtt(void){

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, &mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void initLed() {
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = PINS_SEL;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void initSensor(){
    DHT11_init(DATA_SENSOR);
}

void logDht11Values(struct dht11_reading readings){
    if(readings.status == DHT11_OK){
        ESP_LOGI(TAG_SENSOR, "Sensor readings temperature: %d, humidity: %d", readings.temperature, readings.humidity);
    }
}

void publishSensorValue(char* topic, int value){

  //  sprintf(convertValue, "%d", value);
   //
}

void vTaskDht11(void * pvParameters){
    initSensor();
    char temperature[3] = {};
    char humidity[3] = {};
    while(true){
        struct dht11_reading readings = DHT11_read();
        logDht11Values(readings);
        if(client != NULL){
            sprintf(temperature, "%d", readings.temperature);
            sprintf(humidity, "%d", readings.humidity);
            esp_mqtt_client_publish(client, TEMPERATURE_SET, temperature, 0, 0, 0);
            esp_mqtt_client_publish(client, HUMIDITY_SET, humidity, 0, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TaskHandle_t xHandle = NULL;

void app_main(void)
{
    xTaskCreate(vTaskDht11,
        "SENSOR_READING",
        2048,
        NULL,
        configMAX_PRIORITIES-1,
    //if I want to delete I just need to pass the handle
        NULL
    );

    //Initialize NVS - flash
    esp_err_t ret = nvs_flash_init();
    //initializing the flash may fail and we need to detected the error
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //   ESP_ERROR_CHECK(nvs_flash_erase());
    //   ret = nvs_flash_init();
    // }
    //check errors
    ESP_ERROR_CHECK(ret);

    eventGroup = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    initWifi();
    initLed();
    initMqtt();


}
