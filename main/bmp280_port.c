#include "bmp280_port.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "board_config.h"

BMP2_INTF_RET_TYPE bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                 uint32_t length, const void *intf_ptr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_READ, true);

  if (length > 1) {
    i2c_master_read(cmd, reg_data, length - 1, I2C_MASTER_ACK);
  }

  i2c_master_read_byte(cmd, reg_data + length - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret =
      i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return BMP2_E_COM_FAIL;
  }

  return BMP2_OK;
}

BMP2_INTF_RET_TYPE bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                  uint32_t length, const void *intf_ptr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write(cmd, reg_data, length, true);
  i2c_master_stop(cmd);

  esp_err_t ret =
      i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return BMP2_E_COM_FAIL;
  }

  return BMP2_OK;
}

void bmp2_delay_us(uint32_t period_us, void *intf_ptr) {
  if (period_us > 1000) {
    vTaskDelay(pdMS_TO_TICKS(period_us / 1000));
  }
}
