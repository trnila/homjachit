#include <esp_log.h>
#include <nvs_flash.h>
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "camera.h"
#include "stream.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"



esp_err_t httpd_get_index(httpd_req_t *req){
  httpd_resp_sendstr(req, "<h1>oVerEnginEEred automatic feeder</h1><img id='camera'><img id='camera'><script>document.getElementById('camera').src = `http://${document.location.host}:81/`;</script>");
  return ESP_OK;
}

httpd_uri_t uri_get = {
  .uri      = "/",
  .method   = HTTP_GET,
  .handler  = httpd_get_index,
  .user_ctx = NULL
};

static char stats[1024];

#define LED_PIN 21
#define BTN_PIN 22
#define STEP_PIN 4
#define IO_SPI_CS 5
#define IO_STEPPER_DIR 6
#define IO_STEPPER_EN 7
int i2c_master_port = 0;
uint8_t pins_state = 0;


void set_pin(uint8_t pin, bool state) {
  if(state) {
    pins_state |= 1U << pin;
  } else {
    pins_state &= ~(1U << pin);
  }

  uint8_t write_buf[] = {0x01, pins_state};
  ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, 0x18, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS));
}

static void cs_high(spi_transaction_t* t) {
  set_pin(IO_SPI_CS, 0);
}

static void cs_low(spi_transaction_t* t) {
  set_pin(IO_SPI_CS, 1);
}

void app_main() {
  gpio_config_t gpio_led = {};
  gpio_led.mode = GPIO_MODE_OUTPUT;
  gpio_led.pin_bit_mask = 1ULL << LED_PIN;
  ESP_ERROR_CHECK(gpio_config(&gpio_led));

  memset(&gpio_led, 0, sizeof(gpio_led));
  gpio_led.mode = GPIO_MODE_INPUT;
  gpio_led.pin_bit_mask = 1ULL << BTN_PIN;
  ESP_ERROR_CHECK(gpio_config(&gpio_led));


  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 15,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = 2,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
    .clk_flags = 0,
  };
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));

  // set all pins to output
  uint8_t write_buf[] = {0x03, 0x00};
  ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, 0x18, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS));

  spi_bus_config_t buscfg={
    .miso_io_num = 5,
    .mosi_io_num = 18,
    .sclk_io_num = 19,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 40,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  spi_device_handle_t spi_handle;
  spi_device_interface_config_t devcfg={
    .command_bits = 0,
    .clock_speed_hz = 1000000,
    .mode = 0,
    .spics_io_num = -1,
    .queue_size = 1,
    .flags = SPI_DEVICE_POSITIVE_CS,
    .pre_cb = cs_high,
    .post_cb = cs_low,
    .input_delay_ns = 100,
  };
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));

  set_pin(IO_STEPPER_DIR, 0);
  set_pin(IO_STEPPER_EN, 0);

  memset(&gpio_led, 0, sizeof(gpio_led));
  gpio_led.mode = GPIO_MODE_OUTPUT;
  gpio_led.pin_bit_mask = 1ULL << STEP_PIN;
  ESP_ERROR_CHECK(gpio_config(&gpio_led));

  int i = 0;
  bool led_state = false; 
  for(;;) {
  //  ESP_LOGI("test", "cus kaktus %d %d", i++, gpio_get_level(BTN_PIN));
    gpio_set_level(LED_PIN, led_state);
    led_state = !led_state;

    uint8_t write_buf[] = {0x01, led_state ? 0xFF : 0x00};
    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, 0x18, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS));
    
    set_pin(2, led_state);

    //esp_rom_delay_us(100);

    gpio_set_level(STEP_PIN, led_state);


    uint8_t tx[5] = {
      0x04,
    };
    uint8_t rx[5] = {};
    spi_transaction_t t = {};
    t.length = 40;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &t));

    ESP_LOG_BUFFER_HEXDUMP("spi", rx, sizeof(rx), ESP_LOG_ERROR);
  }

  nvs_flash_init();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {
    .sta = {
      .ssid = "user",
      .password = "pass",
    },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_connect());

  ESP_ERROR_CHECK(camera_start());
  ESP_ERROR_CHECK(stream_start());

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &uri_get);
  }

  for(;;) {
    vTaskGetRunTimeStats(stats);
    printf("%s\n", stats);
    vTaskDelay(1000);
  }
}
