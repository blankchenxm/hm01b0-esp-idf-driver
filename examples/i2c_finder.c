#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C配置 - 根据你的硬件连接修改这些引脚
#define I2C_MASTER_SCL_IO    1              // SCL引脚 (GPIO1)
#define I2C_MASTER_SDA_IO    2              // SDA引脚 (GPIO2)
#define I2C_MASTER_NUM       I2C_NUM_0      // I2C端口号
#define I2C_MASTER_FREQ_HZ   100000         // I2C频率 100kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_TIMEOUT_MS       50             // 超时时间

// I2C有效地址范围 (0x00-0x07 和 0x78-0x7F 是保留地址)
#define I2C_ADDR_MIN   0x08
#define I2C_ADDR_MAX   0x77

static const char *TAG = "I2C_Scanner";

/**
 * @brief 初始化I2C主机
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C初始化成功 - SDA: GPIO%d, SCL: GPIO%d",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return ESP_OK;
}

/**
 * @brief 检测指定地址是否有I2C设备
 */
static bool i2c_detect_device(uint8_t address)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                          I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

/**
 * @brief 扫描I2C总线并以表格形式显示结果
 */
static void i2c_scan_devices(void)
{
    uint8_t devices_found = 0;
    uint8_t device_addresses[128] = {0};

    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║             ESP32-S3 I2C 设备扫描器                      ║\n");
    printf("╠══════════════════════════════════════════════════════════╣\n");
    printf("║  配置: SDA=GPIO%d, SCL=GPIO%d, 频率=%dHz            ║\n",
           I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    printf("╚══════════════════════════════════════════════════════════╝\n");
    printf("\n");

    ESP_LOGI(TAG, "开始扫描I2C总线 (地址范围: 0x%02X - 0x%02X)...",
             I2C_ADDR_MIN, I2C_ADDR_MAX);
    printf("\n");

    // 打印表头 - 类似 i2cdetect 的格式
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (uint8_t row = 0; row < 8; row++) {
        printf("%02X:", row << 4);

        for (uint8_t col = 0; col < 16; col++) {
            uint8_t address = (row << 4) | col;

            // 跳过保留地址
            if (address < I2C_ADDR_MIN || address > I2C_ADDR_MAX) {
                printf("   ");
                continue;
            }

            // 检测设备
            if (i2c_detect_device(address)) {
                printf(" %02X", address);
                device_addresses[devices_found++] = address;
            } else {
                printf(" --");
            }
        }
        printf("\n");
    }

    printf("\n");
    printf("════════════════════════════════════════════════════════════\n");

    // 显示扫描结果
    if (devices_found > 0) {
        printf("✓ 扫描完成! 发现 %d 个I2C设备:\n", devices_found);
        printf("\n");
        printf("  地址(HEX)  地址(DEC)  可能的设备\n");
        printf("  ---------  ---------  ------------------\n");

        for (uint8_t i = 0; i < devices_found; i++) {
            uint8_t addr = device_addresses[i];
            const char *device_name = "未知设备";

            // 常见I2C设备地址识别
            if (addr == 0x3C || addr == 0x3D) {
                device_name = "OLED显示屏 (SSD1306/SH1106)";
            } else if (addr >= 0x20 && addr <= 0x27) {
                device_name = "PCF8574 IO扩展器";
            } else if (addr == 0x27) {
                device_name = "LCD 1602 (PCF8574)";
            } else if (addr >= 0x48 && addr <= 0x4F) {
                device_name = "ADS1115/PCF8591 ADC";
            } else if (addr == 0x50 || addr == 0x51) {
                device_name = "AT24Cxx EEPROM";
            } else if (addr == 0x68) {
                device_name = "MPU6050/DS3231 RTC";
            } else if (addr == 0x69) {
                device_name = "MPU6050 (AD0=1)";
            } else if (addr == 0x76 || addr == 0x77) {
                device_name = "BME280/BMP280 气压传感器";
            } else if (addr == 0x29) {
                device_name = "VL53L0X 激光测距";
            } else if (addr == 0x39) {
                device_name = "TSL2561 光照传感器";
            } else if (addr == 0x40) {
                device_name = "INA219 功率监测 / SHT30";
            } else if (addr == 0x44) {
                device_name = "SHT30/SHT31 温湿度";
            } else if (addr == 0x57) {
                device_name = "MAX30102 心率血氧";
            } else if (addr == 0x5A) {
                device_name = "MLX90614 红外测温";
            } else if (addr == 0x60) {
                device_name = "SI1145 紫外线传感器";
            } else if (addr == 0x62) {
                device_name = "SCD40 CO2传感器";
            } else if (addr == 0x1E) {
                device_name = "HMC5883L 磁力计";
            } else if (addr == 0x53) {
                device_name = "ADXL345 加速度计";
            }

            printf("    0x%02X       %3d      %s\n", addr, addr, device_name);
        }
    } else {
        printf("✗ 未发现任何I2C设备!\n");
        printf("\n");
        printf("请检查:\n");
        printf("  1. I2C设备是否正确连接到 SDA(GPIO%d) 和 SCL(GPIO%d)\n",
               I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        printf("  2. 是否已连接上拉电阻 (4.7K-10K)\n");
        printf("  3. I2C设备是否正常供电\n");
        printf("  4. 接线是否正确 (注意SDA和SCL不要接反)\n");
    }

    printf("════════════════════════════════════════════════════════════\n");
    printf("\n");
}

/**
 * @brief 主程序入口
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 I2C Scanner 启动");

    // 初始化I2C
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C初始化失败，程序终止");
        return;
    }

    // 等待一下让系统稳定
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 循环扫描 - 每5秒扫描一次
    while (1) {
        i2c_scan_devices();

        ESP_LOGI(TAG, "等待5秒后重新扫描... (可以热插拔I2C设备)");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
