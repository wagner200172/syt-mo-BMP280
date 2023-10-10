#include "../include/board.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char* TAG = "MAIN";

void initI2C() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_GPIO_SDA,  // select SDA GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_GPIO_SCL,  // select SCL GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK_SPEED,  // select frequency specific to your project
        .clk_flags = 0,     // optional; you can use I2C_SCLK_SRC_FLAG_* flags to
                          // choose i2c source clock here
    };

    i2c_param_config(I2C_NUM_0, &conf);
    esp_err_t espRc = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    if (espRc == ESP_OK) {
        ESP_LOGD(TAG, "success: i2c installation");
    } else {
        ESP_LOGD(TAG, "failed: i2c installation");
    }
}


int8_t userWrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t dev_addr = dev_id;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);


    i2c_master_write(cmd, reg_data, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

 

int8_t userRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t dev_addr = dev_id;
    esp_err_t espRc;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id<< 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, reg_data, len-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data+len-1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void app_main() {
    ESP_LOGD(TAG, "First Program");
    initI2C();
    userRead();
}



uint8_t dev_addr = BMP280_I2C_ADDR_PRIM;
rslt =bmp280