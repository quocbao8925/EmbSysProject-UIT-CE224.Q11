// i2c_lcd.c
#include "i2c_lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "LCD";

// Biến toàn cục lưu port đang dùng để các hàm send_data biết đường gửi
static i2c_port_t g_i2c_num = I2C_NUM_0; 

// Các bit điều khiển (GIỮ NGUYÊN)
#define PIN_RS    0x01
#define PIN_RW    0x02
#define PIN_EN    0x04
#define PIN_BK    0x08

static uint8_t backlight_state = PIN_BK;

// Hàm gửi byte (SỬA ĐỂ DÙNG g_i2c_num)
static esp_err_t i2c_write_byte(uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    // Dùng g_i2c_num thay vì macro cứng
    esp_err_t ret = i2c_master_cmd_begin(g_i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ... (Giữ nguyên hàm lcd_pulse_enable, lcd_write_nibble, lcd_send_byte, send_cmd, send_data...)
// ... Copy lại y nguyên các hàm logic từ câu trả lời trước, chỉ cần đảm bảo i2c_write_byte đã sửa như trên ...

static void lcd_pulse_enable(uint8_t val)
{
    i2c_write_byte(val & ~PIN_EN);
    ets_delay_us(50);
    i2c_write_byte(val | PIN_EN);
    ets_delay_us(50);
    i2c_write_byte(val & ~PIN_EN);
    ets_delay_us(50);
}

static void lcd_write_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | mode | backlight_state;
    lcd_pulse_enable(data);
}

static void lcd_send_byte(uint8_t val, uint8_t mode)
{
    uint8_t high = val & 0xF0;
    uint8_t low = (val << 4) & 0xF0;
    lcd_write_nibble(high, mode);
    lcd_write_nibble(low, mode);
}

void lcd_send_cmd(char cmd) { lcd_send_byte(cmd, 0); }
void lcd_send_data(char data) { lcd_send_byte(data, PIN_RS); }

// --- HÀM INIT ĐƯỢC SỬA ĐỔI ---
void lcd_init(i2c_port_t port_num)
{
    // 1. Lưu port number vào biến toàn cục để dùng sau này
    g_i2c_num = port_num;

    // QUAN TRỌNG: KHÔNG gọi i2c_driver_install ở đây nữa!
    // Chúng ta giả định rằng Main App hoặc PCA9685 đã khởi tạo Bus này rồi.
    
    ESP_LOGI(TAG, "LCD using existing I2C Port %d", port_num);

    // 2. Quy trình khởi tạo LCD (Logic cũ giữ nguyên)
    vTaskDelay(100 / portTICK_PERIOD_MS);

    lcd_write_nibble(0x30, 0); vTaskDelay(10 / portTICK_PERIOD_MS);
    lcd_write_nibble(0x30, 0); vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_write_nibble(0x30, 0); vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_write_nibble(0x20, 0); vTaskDelay(1 / portTICK_PERIOD_MS);

    lcd_send_cmd(0x28); 
    lcd_send_cmd(0x08); 
    lcd_send_cmd(0x01); 
    vTaskDelay(20 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x06); 
    lcd_send_cmd(0x0C); 
    
    ESP_LOGI(TAG, "LCD Init Done on shared bus");
}

// ... (Giữ nguyên lcd_send_string, lcd_put_cur, lcd_clear) ...
void lcd_send_string(char *str) { while (*str) lcd_send_data(*str++); }
void lcd_put_cur(int row, int col) {
    uint8_t address = (row == 0) ? 0x80 : 0xC0; // Đơn giản hóa ví dụ
    if (row == 2) address = 0x94;
    if (row == 3) address = 0xD4;
    address += col;
    lcd_send_cmd(address);
}
void lcd_clear(void) { lcd_send_cmd(0x01); vTaskDelay(20 / portTICK_PERIOD_MS); }