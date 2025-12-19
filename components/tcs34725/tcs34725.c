
#include "tcs34725.h"
#include "esp_log.h"

static const char *TAG = "TCS34725";
void i2c_address_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t dummy;
        esp_err_t res = i2c_master_read_from_device(I2C_MASTER_NUM, addr, &dummy, 1, 10 / portTICK_PERIOD_MS);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan completed.");
}
void tcs34725_init(tcs34725_handler *handler)
{
    handler->r = 0;
    handler->g = 0;
    handler->b = 0;
    // init i2c
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);

    // check sensor ID
    uint8_t id = 0;
    tcs_read(handler, REG_ID, &id, 1);
    ESP_LOGI(TAG, "Sensor ID = 0x%02X", id);

    // enable the sensor
    tcs_write(REG_ENABLE, ENABLE_PON);
    vTaskDelay(pdMS_TO_TICKS(3));
    tcs_write(REG_ENABLE, ENABLE_PON | ENABLE_AEN);

    // configure integration time and gain
    tcs_write(REG_ATIME, 0xD6);   // ~100 ms
    tcs_write(REG_CONTROL, 0x01); // Gain = 4x

    vTaskDelay(pdMS_TO_TICKS(200)); // chờ cảm biến đo lần đầu
    // Initialization code can be added here if needed
    ESP_LOGI(TAG, "TCS34725 initialized");
}
void tcs34725_deinit(void)
{
    // Disable the sensor
    tcs_write(REG_ENABLE, 0x00);
    // Delete I2C driver
    i2c_driver_delete(I2C_MASTER_NUM);
    ESP_LOGI(TAG, "TCS34725 deinitialized");
}
esp_err_t tcs_read(tcs34725_handler *handler, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t cmd = TCS_CMD_BIT | TCS_CMD_AUTO_INC | reg; //
    return i2c_master_write_read_device(I2C_MASTER_NUM, TCS34725_ADDR,
                                        &cmd, 1, data, len, pdMS_TO_TICKS(1000));
}
esp_err_t tcs_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { TCS_CMD_BIT | reg, value };
    return i2c_master_write_to_device(I2C_MASTER_NUM, TCS34725_ADDR,
                                      data, sizeof(data), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}
void tcs34725_reader(tcs34725_handler *handler)
{
    //ir_sensor_counter(&ir_handler, &ir_event);
        uint8_t bufc[2];
        uint8_t bufr[2];
        uint8_t bufg[2];
        uint8_t bufb[2];
        if (tcs_read(handler, REG_CDATAL, bufc, sizeof(bufc)) == ESP_OK 
            && tcs_read(handler, REG_RDATAL, bufr, sizeof(bufr)) == ESP_OK
            && tcs_read(handler, REG_GDATAL, bufg, sizeof(bufg)) == ESP_OK
            && tcs_read(handler, REG_BDATAL, bufb, sizeof(bufb)) == ESP_OK) {
            uint16_t c = ((uint16_t)bufc[1] << 8) | bufc[0];
            uint16_t r = ((uint16_t)bufr[1] << 8) | bufr[0];
            uint16_t g = ((uint16_t)bufg[1] << 8) | bufg[0];
            uint16_t b = ((uint16_t)bufb[1] << 8) | bufb[0];
            float rn = (float)r / (float)c;
            float gn = (float)g / (float)c;
            float bn = (float)b / (float)c;

            handler->r = (uint8_t)(rn * 255.0f + 0.5f);
            handler->g = (uint8_t)(gn * 255.0f + 0.5f);
            handler->b = (uint8_t)(bn * 255.0f + 0.5f);

            /*ESP_LOGI(TAG,
                "Raw: C=%u R=%u G=%u B=%u | Normalized RGB = %d, %d, %d",
                c, r, g, b, handler->r, handler->g, handler->b);*/
        } else {
            ESP_LOGE(TAG, "Failed to read color data");
        }
}
void get_rgb_values(tcs34725_handler *handler, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (handler == NULL || r == NULL || g == NULL || b == NULL) {
        ESP_LOGE(TAG, "Invalid arguments to get_rgb_values");
        return;
    }
    *r = handler->r;
    *g = handler->g;
    *b = handler->b;
}
