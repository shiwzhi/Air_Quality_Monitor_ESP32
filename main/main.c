#include <stdio.h>

#include "bmp2.h"
#include "bmp280_port.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "scd4x_i2c.h"
#include "sensirion_gas_index_algorithm.h"
#include "sgp40_i2c.h"
#include "u8g2.h"
#include "u8g2_port.h"

static const char* TAG = "main";

static float g_co2, g_temp, g_hum, g_pressure, g_voc_index, pm1, pm25, pm10;

static esp_err_t init_gpio() {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = 1 << i2c_sensor_pwr_pin;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  if (ESP_OK != gpio_config(&io_conf)) {
    return ESP_FAIL;
  }

  return gpio_set_level(i2c_sensor_pwr_pin, 1);
}

static esp_err_t init_i2c() {
  static const i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = i2c_sda_pin,  // select SDA GPIO specific to your project
      .sda_pullup_en = GPIO_PULLUP_DISABLE,
      .scl_io_num = i2c_cl_pin,  // select SCL GPIO specific to your project
      .scl_pullup_en = GPIO_PULLUP_DISABLE,
      .master.clk_speed = 1000000  // select frequency specific to your project

  };
  if (ESP_OK != i2c_param_config(i2c_master_port, &conf)) {
    return ESP_FAIL;
  }
  return i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0);
}

void u8g2_task() {
  u8g2_t u8g2;
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(
      &u8g2, U8G2_R0, u8x8_byte_arduino_hw_i2c, u8x8_gpio_and_delay_template);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_SetFont(&u8g2, u8g2_font_fur17_tr);

  while (1) {
    u8g2_ClearBuffer(&u8g2);
    char str[50];

    sprintf(str, "%.0f  %.0f  %.0f", pm1, pm25, pm10);
    u8g2_DrawStr(&u8g2, 0, 19, str);

    sprintf(str, "%.0f   %.0fV", g_co2, g_voc_index);
    u8g2_DrawStr(&u8g2, 0, 41, str);

    sprintf(str, "%.1fC %.1f%%", g_temp, g_hum);
    u8g2_DrawStr(&u8g2, 0, 64, str);

    u8g2_SendBuffer(&u8g2);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void scd4x_task() {
  ESP_LOGI(TAG, "scd4x: %i", scd4x_wake_up());
  ESP_LOGI(TAG, "scd4x: %i", scd4x_stop_periodic_measurement());
  //   ESP_LOGI(TAG, "scd4x: %i", scd4x_perform_factory_reset());
  ESP_LOGI(TAG, "scd4x: %i", scd4x_reinit());
  ESP_LOGI(TAG, "scd4x: %i", scd4x_set_temperature_offset(6200));
  ESP_LOGI(TAG, "scd4x: %i", scd4x_start_periodic_measurement());

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "scd4x: %i", scd4x_set_ambient_pressure(g_pressure));
    bool ready = false;
    scd4x_get_data_ready_flag(&ready);
    if (ready) {
      uint16_t co2;
      int32_t temp, hum;
      scd4x_read_measurement(&co2, &temp, &hum);
      ESP_LOGI(TAG, "co2: %u temp: %.2f hum: %.2f", co2,
               (float)(temp / 1000.0f), (float)(hum / 1000.0f));

      g_co2 = co2;
      g_temp = temp / 1000.0f;
      g_hum = hum / 1000.f;
    }
  }
}

void bmp280_task() {
  int8_t rslt;
  uint32_t meas_time;
  struct bmp2_dev dev;
  struct bmp2_config conf;
  dev.read = bmp2_i2c_read;
  dev.write = bmp2_i2c_write;
  dev.intf = BMP2_I2C_INTF;
  dev.intf_ptr = NULL;
  dev.delay_us = bmp2_delay_us;
  rslt = bmp2_init(&dev);
  bmp2_get_config(&conf, &dev);
  conf.filter = BMP2_FILTER_COEFF_16;
  conf.os_mode = BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION;
  conf.odr = BMP2_ODR_0_5_MS;
  rslt = bmp2_set_config(&conf, &dev);
  rslt = bmp2_set_power_mode(BMP2_POWERMODE_FORCED, &conf, &dev);
  rslt = bmp2_compute_meas_time(&meas_time, &conf, &dev);
  struct bmp2_status status;
  struct bmp2_data comp_data;
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(meas_time / 1000 + 5000));

    rslt = bmp2_get_status(&status, &dev);
    if (status.measuring == BMP2_MEAS_DONE) {
      rslt = bmp2_get_sensor_data(&comp_data, &dev);
      printf(" Temperature: %.4lf deg C	Pressure: %.4lf Pa\n",
             comp_data.temperature, comp_data.pressure);
      g_pressure = comp_data.pressure / 100;
    }

    rslt = bmp2_set_power_mode(BMP2_POWERMODE_FORCED, &conf, &dev);
  }
}

void sgp40_task() {
  vTaskDelay(pdMS_TO_TICKS(5000));
  int16_t error = 0;
  uint16_t sraw_voc;
  int32_t voc_index_value;
  GasIndexAlgorithmParams params;
  GasIndexAlgorithm_init(&params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    error = sgp40_measure_raw_signal(g_hum * 65535 / 100,
                                     (g_temp + 45) * 65535 / 175, &sraw_voc);
    if (!error) {
      GasIndexAlgorithm_process(&params, sraw_voc, &voc_index_value);
      g_voc_index = voc_index_value;
      ESP_LOGI(TAG, "Raw VOC: %u Index: %li", sraw_voc, voc_index_value);
    }
  }
}

void pms_task() {
  const uart_port_t uart_num = UART_NUM_1;
  const int uart_buffer_size = (1024 * 2);
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
                                      uart_buffer_size, 10, &uart_queue, 0));

  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(
      uart_set_pin(uart_num, 1, 0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  while (1) {
    uint8_t data[32];
    int length = 0;
    length = uart_read_bytes(uart_num, data, 32, pdMS_TO_TICKS(1200));
    if (length == 32 && data[0] == 0x42 && data[1] == 0x4d) {
      pm1 = data[10] * 256 + data[11];
      pm25 = data[12] * 256 + data[13];
      pm10 = data[14] * 256 + data[15];
      ESP_LOGI(TAG, "uart rx: %i pm1: %.0f pm25: %.0f pm10: %.0f", length, pm1,
               pm25, pm10);
    }
  }
}


void app_main(void) {
  ESP_LOGI(TAG, "App started");
  if (ESP_OK != init_gpio()) {
    esp_restart();
  }
  if (ESP_OK != init_i2c()) {
    esp_restart();
  }

  xTaskCreate(pms_task, "pms_task", 4096, NULL, 0, NULL);
  xTaskCreate(u8g2_task, "u8g2_task", 4096, NULL, 0, NULL);
  xTaskCreate(bmp280_task, "bmp_task", 4096, NULL, 0, NULL);
  vTaskDelay(pdMS_TO_TICKS(10000));
  xTaskCreate(scd4x_task, "scd4x_task", 4096, NULL, 0, NULL);
  xTaskCreate(sgp40_task, "sgp40_task", 4096, NULL, 3, NULL);


  while (1) {
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
