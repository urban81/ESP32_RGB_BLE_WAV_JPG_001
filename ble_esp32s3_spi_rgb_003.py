/*
 * ESP32-S3 + ST7701S 400x960 RGB Display (портретная ориентация)
 * Подсветка на GPIO6 (BL PWM), кнопка на GPIO17 (активный высокий уровень).
 * При нажатии кнопки дисплей полностью выключается (DISPLAY OFF) и подсветка гаснет.
 * Повторное нажатие включает.
 *
 * Добавлен индикатор BLE в правом верхнем углу:
 *   - чёрный кружок – нет подключения
 *   - голубой кружок – есть подключение
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_st7701.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_io_expander_tca9554.h"
#include "esp_io_expander.h"
#include "esp_heap_caps.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "jpeg_decoder.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include <errno.h>
#include <unistd.h>

// Подключаем массив с тестовой картинкой (400x960 RGB565)
#include "image2.h"

static const char *TAG = "ST7701S";

// ==================== I2C (TCA9554) ====================
#define I2C_MASTER_SCL_IO           GPIO_NUM_7
#define I2C_MASTER_SDA_IO           GPIO_NUM_15
#define I2C_MASTER_FREQ_HZ          100000
#define TCA9554_ADDR                 0x20

#define TCA9554_PIN_TPRST            0
#define TCA9554_PIN_LCD_RST           2   // сброс дисплея

// ==================== ПОДСВЕТКА ====================
#define BACKLIGHT_GPIO               GPIO_NUM_6   // пин подсветки

// ==================== КНОПКА ====================
#define BUTTON_GPIO                  GPIO_NUM_16  // кнопка на GPIO 16 (активный высокий уровень)

// ==================== PIN DEFINITIONS ====================
#define LCD_SPI_SCK_GPIO            2
#define LCD_SPI_SDA_GPIO            1
#define LCD_SPI_CS_GPIO              42

#define LCD_HSYNC_GPIO               38
#define LCD_VSYNC_GPIO               39
#define LCD_DE_GPIO                  40
#define LCD_PCLK_GPIO                41

#define LCD_R0_GPIO                  19
#define LCD_R1_GPIO                  46
#define LCD_R2_GPIO                  3
#define LCD_R3_GPIO                  8
#define LCD_R4_GPIO                  18

#define LCD_G0_GPIO                  14
#define LCD_G1_GPIO                  13
#define LCD_G2_GPIO                  12
#define LCD_G3_GPIO                  11
#define LCD_G4_GPIO                  10
#define LCD_G5_GPIO                  9

#define LCD_B0_GPIO                  21
#define LCD_B1_GPIO                  5
#define LCD_B2_GPIO                  45
#define LCD_B3_GPIO                  48
#define LCD_B4_GPIO                  47

#define LCD_H_RES                    400
#define LCD_V_RES                    960
#define IMG_W                        400
#define IMG_H                        960

#define PCLK_FREQ_HZ                 8000000
#define PCLK_ACTIVE_NEG               1
#define HSYNC_IDLE_LOW                0
#define VSYNC_IDLE_LOW                0
#define DE_IDLE_HIGH                  0

#define HSYNC_PULSE_WIDTH             9
#define HSYNC_BACK_PORCH              9
#define HSYNC_FRONT_PORCH             45

#define VSYNC_PULSE_WIDTH             2
#define VSYNC_BACK_PORCH              13
#define VSYNC_FRONT_PORCH             15

// ==================== JPEG и BLE ====================
#define JPEG_BUFFER_SIZE (300 * 1024) // 300 KB
static uint8_t *jpeg_buffer = NULL;
static size_t jpeg_received = 0;
static uint16_t total_packets = 0;
static uint16_t packets_received = 0;
static bool receiving = false;
static SemaphoreHandle_t ble_data_sem = NULL;

#define PACKET_TYPE_IMAGE_START 0x01
#define PACKET_TYPE_IMAGE_DATA  0x02
#define PACKET_TYPE_IMAGE_END   0x03

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2s_std.h"

// NimBLE
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// --- КОНФИГУРАЦИЯ ---
#define I2S_BCK_IO           44
#define I2S_WS_IO            43
#define I2S_DIN_IO           17
#define I2S_SAMPLE_RATE      16000

#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01

// Глобальные флаги
static uint16_t audio_char_handle;
static uint16_t conn_handle = 0xffff;
static bool is_connected = false;
static bool notify_enabled = false;
static bool is_receiving_jpg = false; // ФЛАГ: ПРИЕМ КАРТИНКИ
i2s_chan_handle_t rx_handle = NULL;

// --- ИНИЦИАЛИЗАЦИЯ I2S ---
void init_i2s() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = I2S_BCK_IO, .ws = I2S_WS_IO, .din = I2S_DIN_IO },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT; 
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

// --- ЗАДАЧА ПЕРЕДАЧИ ЗВУКА ---
void audio_stream_task(void *pvParameters) {
    const int samples_count = 120; 
    uint32_t *raw_buffer = malloc(samples_count * 4);
    int16_t *send_buffer = malloc(samples_count * 2);
    size_t bytes_read;

    while (1) {
        // УСЛОВИЕ: передаем звук только если нет приема JPG
        if (is_connected && notify_enabled && !is_receiving_jpg) {
            if (i2s_channel_read(rx_handle, raw_buffer, samples_count * 4, &bytes_read, 20) == ESP_OK) {
                int read_samples = bytes_read / 4;
                for (int i = 0; i < read_samples; i++) {
                    send_buffer[i] = (int16_t)(raw_buffer[i] >> 16);
                }
                struct os_mbuf *om = ble_hs_mbuf_from_flat(send_buffer, read_samples * 2);
                if (om) ble_gattc_notify_custom(conn_handle, audio_char_handle, om);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50)); // Ждем, пока освободится канал
        }
    }
}

static int rx_char_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(GATTS_SERVICE_UUID),
     .characteristics = (struct ble_gatt_chr_def[]){{
         .uuid = BLE_UUID16_DECLARE(GATTS_CHAR_UUID),
         .access_cb = rx_char_access,
         .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
         .val_handle = &audio_char_handle,
     }, {0}}},
    {0}};

// --- GAP И СИНХРОНИЗАЦИЯ (ИСПРАВЛЕНО) ---
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            conn_handle = event->connect.conn_handle;
            is_connected = (event->connect.status == 0);
            break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            notify_enabled = event->subscribe.cur_notify;
            break;
    }
    return 0;
}

// ==================== SPIFFS ====================
#define SPIFFS_BASE_PATH "/spiffs"
#define IMAGE_FILE_NAME  "/current.jpg"

static void mount_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = SPIFFS_BASE_PATH,
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted");
    }
}

static void load_and_display_saved_image(void);
static void process_received_jpeg(void);
static void save_received_image(void);
static void display_clear(void);
static void display_image(uint16_t *image, size_t image_len);

static void load_and_display_saved_image(void) {
    FILE *f = fopen(SPIFFS_BASE_PATH IMAGE_FILE_NAME, "rb");
    if (!f) {
        ESP_LOGI(TAG, "No saved image found");
        return;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0 || size > JPEG_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Invalid image size: %ld", size);
        fclose(f);
        return;
    }
    if (!jpeg_buffer) {
        jpeg_buffer = heap_caps_malloc(JPEG_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
        if (!jpeg_buffer) jpeg_buffer = malloc(JPEG_BUFFER_SIZE);
    }
    if (!jpeg_buffer) {
        ESP_LOGE(TAG, "No memory for JPEG buffer");
        fclose(f);
        return;
    }
    size_t read = fread(jpeg_buffer, 1, size, f);
    fclose(f);
    if (read != size) {
        ESP_LOGE(TAG, "Failed to read full image");
        return;
    }
    ESP_LOGI(TAG, "Loaded saved image (%u bytes). Decoding...", read);
    size_t saved = jpeg_received;
    jpeg_received = read;
    process_received_jpeg();
    jpeg_received = saved;
}

static void save_received_image(void) {
    if (!jpeg_buffer || jpeg_received == 0) return;
    unlink(SPIFFS_BASE_PATH IMAGE_FILE_NAME);
    FILE *f = fopen(SPIFFS_BASE_PATH IMAGE_FILE_NAME, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open file for writing. errno=%d", errno);
        return;
    }
    size_t written = fwrite(jpeg_buffer, 1, jpeg_received, f);
    fclose(f);
    if (written == jpeg_received) {
        ESP_LOGI(TAG, "Image saved (%u bytes)", jpeg_received);
    } else {
        ESP_LOGE(TAG, "Write failed. Written=%d, expected=%d", written, jpeg_received);
    }
}

// ==================== ИНИЦИАЛИЗАЦИЯ ST7701 ====================
static const st7701_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, (uint8_t []){0x08},                           1, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t []){0x77, 0x00},                     2, 0},
    {0xC1, (uint8_t []){0x09, 0x08},                     2, 0},
    {0xC2, (uint8_t []){0x01, 0x02},                     2, 0},
    {0xC3, (uint8_t []){0x02},                           1, 0},
    {0xCC, (uint8_t []){0x10},                           1, 0},
    {0xB0, (uint8_t []){0x40, 0x14, 0x59, 0x10, 0x12, 0x08, 0x03, 0x09, 0x05, 0x1E, 0x05, 0x14, 0x10, 0x68, 0x33, 0x15}, 16, 0},
    {0xB1, (uint8_t []){0x40, 0x08, 0x53, 0x09, 0x11, 0x09, 0x02, 0x07, 0x09, 0x1A, 0x04, 0x12, 0x12, 0x64, 0x29, 0x29}, 16, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t []){0x6D},                           1, 0},
    {0xB1, (uint8_t []){0x1D},                           1, 0},
    {0xB2, (uint8_t []){0x87},                           1, 0},
    {0xB3, (uint8_t []){0x80},                           1, 0},
    {0xB5, (uint8_t []){0x49},                           1, 0},
    {0xB7, (uint8_t []){0x85},                           1, 0},
    {0xB8, (uint8_t []){0x20},                           1, 0},
    {0xC1, (uint8_t []){0x78},                           1, 0},
    {0xC2, (uint8_t []){0x78},                           1, 0},
    {0xD0, (uint8_t []){0x88},                           1, 0},
    {0xE0, (uint8_t []){0x00, 0x00, 0x02},               3, 0},
    {0xE1, (uint8_t []){0x02, 0x8C, 0x00, 0x00, 0x03, 0x8C, 0x00, 0x00, 0x00, 0x33, 0x33}, 11, 0},
    {0xE2, (uint8_t []){0x33, 0x33, 0x33, 0x33, 0xC9, 0x3C, 0x00, 0x00, 0xCA, 0x3C, 0x00, 0x00, 0x00}, 13, 0},
    {0xE3, (uint8_t []){0x00, 0x00, 0x33, 0x33},         4, 0},
    {0xE4, (uint8_t []){0x44, 0x44},                     2, 0},
    {0xE5, (uint8_t []){0x05, 0xCD, 0x82, 0x82, 0x01, 0xC9, 0x82, 0x82, 0x07, 0xCF, 0x82, 0x82, 0x03, 0xCB, 0x82, 0x82}, 16, 0},
    {0xE6, (uint8_t []){0x00, 0x00, 0x33, 0x33},         4, 0},
    {0xE7, (uint8_t []){0x44, 0x44},                     2, 0},
    {0xE8, (uint8_t []){0x06, 0xCE, 0x82, 0x82, 0x02, 0xCA, 0x82, 0x82, 0x08, 0xD0, 0x82, 0x82, 0x04, 0xCC, 0x82, 0x82}, 16, 0},
    {0xEB, (uint8_t []){0x08, 0x01, 0xE4, 0xE4, 0x88, 0x00, 0x40}, 7, 0},
    {0xEC, (uint8_t []){0x00, 0x00, 0x00},               3, 0},
    {0xED, (uint8_t []){0xFF, 0xF0, 0x07, 0x65, 0x4F, 0xFC, 0xC2, 0x2F, 0xF2, 0x2C, 0xCF, 0xF4, 0x56, 0x70, 0x0F, 0xFF}, 16, 0},
    {0xEF, (uint8_t []){0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F}, 6, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x11, NULL, 0, 120},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x3A, (uint8_t []){0x55}, 1, 0},
    {0x29, NULL, 0, 0},
};

// ==================== I2C/TCA9554 ====================
static esp_io_expander_handle_t expander_handle = NULL;
static i2c_master_bus_handle_t i2c_bus_handle;

static void i2c_and_expander_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = -1,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle));

    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(i2c_bus_handle, TCA9554_ADDR, &expander_handle));

    ESP_ERROR_CHECK(esp_io_expander_set_dir(expander_handle, TCA9554_PIN_TPRST, IO_EXPANDER_OUTPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(expander_handle, TCA9554_PIN_LCD_RST, IO_EXPANDER_OUTPUT));

    ESP_LOGI(TAG, "Hardware reset via TCA9554...");
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander_handle, TCA9554_PIN_LCD_RST, 0));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander_handle, TCA9554_PIN_LCD_RST, 1));
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_ERROR_CHECK(esp_io_expander_set_level(expander_handle, TCA9554_PIN_TPRST, 1));

    ESP_LOGI(TAG, "I2C and TCA9554 initialized");
}

// ==================== SPI 3-Wire ====================
static esp_lcd_panel_io_handle_t init_3wire_spi(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_SPI_SDA_GPIO,
        .miso_io_num = -1,
        .sclk_io_num = LCD_SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO,
        .cs_gpio_num = LCD_SPI_CS_GPIO,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = LCD_SPI_SCK_GPIO,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = LCD_SPI_SDA_GPIO,
        .io_expander = NULL,
    };

    esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_err_t ret = esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create 3-Wire SPI: %d", ret);
        return NULL;
    }
    ESP_LOGI(TAG, "3-Wire SPI initialized");
    return io_handle;
}

// ==================== BLE ====================
static const char *device_name = "ESP32-S3-JPEG";

static void ble_app_set_addr(void) {
    ble_addr_t addr;
    ble_hs_id_gen_rnd(1, &addr);
    ble_hs_id_set_rnd(addr.val);
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement data: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

static void ble_sync_cb(void) {
    ble_app_advertise();
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_init(void) {
    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ble_svc_gap_device_name_set(device_name);
    ble_app_set_addr();

    ble_hs_cfg.sync_cb = ble_sync_cb;

    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "BLE initialized. Name: %s", device_name);
}

// ==================== ДИСПЛЕЙ ====================
static esp_lcd_panel_handle_t panel_handle = NULL;
static uint16_t *frame_buffer = NULL;

// ========== ДОБАВЛЕНО: буфер для оригинала и индикатор BLE (круг) ==========
static uint16_t *original_frame_buffer = NULL;

#define INDICATOR_X     (LCD_H_RES - 56)   // правая граница
#define INDICATOR_Y     10                  // верхняя граница
#define INDICATOR_SIZE  20                  // размер квадрата, в который вписан круг
#define INDICATOR_RADIUS 8                  // радиус круга

#define COLOR_BLACK     0x0000
#define COLOR_CYAN      0x07FF   // голубой (RGB565)

// Маска круга (1 - пиксель круга, 0 - фон)
static const uint8_t circle_mask[INDICATOR_SIZE][INDICATOR_SIZE] = {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
    {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
    {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
    {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
    {0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};

static void update_ble_indicator(bool force) {
    static bool last_connected = false;
    if (!force && last_connected == is_connected) return;

    if (!original_frame_buffer || !frame_buffer) return;

    // Восстановить фон из оригинала для всего квадрата
    for (int row = 0; row < INDICATOR_SIZE; row++) {
        memcpy(frame_buffer + (INDICATOR_Y + row) * LCD_H_RES + INDICATOR_X,
               original_frame_buffer + (INDICATOR_Y + row) * LCD_H_RES + INDICATOR_X,
               INDICATOR_SIZE * 2);
    }

    // Нарисовать круг нужным цветом согласно маске
    uint16_t color = is_connected ? COLOR_CYAN : COLOR_BLACK;
    for (int row = 0; row < INDICATOR_SIZE; row++) {
        for (int col = 0; col < INDICATOR_SIZE; col++) {
            if (circle_mask[row][col]) {
                frame_buffer[(INDICATOR_Y + row) * LCD_H_RES + (INDICATOR_X + col)] = color;
            }
        }
    }

    // Обновить только область на дисплее
    esp_lcd_panel_draw_bitmap(panel_handle,
                              INDICATOR_X, INDICATOR_Y,
                              INDICATOR_X + INDICATOR_SIZE, INDICATOR_Y + INDICATOR_SIZE,
                              frame_buffer);

    last_connected = is_connected;
}
// ==================== КОНЕЦ ДОБАВЛЕНИЯ ====================

static void display_clear(void) {
    if (frame_buffer) {
        memset(frame_buffer, 0, LCD_H_RES * LCD_V_RES * 2);
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, frame_buffer);
    }
}

static void display_image(uint16_t *image, size_t image_len) {
    if (image_len != LCD_H_RES * LCD_V_RES * 2) {
        ESP_LOGE(TAG, "Invalid image size for display");
        return;
    }
    memcpy(frame_buffer, image, image_len);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, frame_buffer);
}

// ==================== JPEG DECODING ====================
static void process_received_jpeg(void) {
    if (!jpeg_buffer || jpeg_received == 0) return;

    if (jpeg_received < 2 || jpeg_buffer[0] != 0xFF || jpeg_buffer[1] != 0xD8) {
        ESP_LOGE(TAG, "Invalid JPEG header: %02X %02X (expected FF D8)", 
                 jpeg_buffer[0], jpeg_buffer[1]);
        ESP_LOG_BUFFER_HEX(TAG, jpeg_buffer, (jpeg_received > 16) ? 16 : jpeg_received);
        return;
    }

    ESP_LOGI(TAG, "First 16 bytes of JPEG buffer:");
    ESP_LOG_BUFFER_HEX(TAG, jpeg_buffer, (jpeg_received > 16) ? 16 : jpeg_received);
    ESP_LOGI(TAG, "Decoding JPEG (%u bytes)...", jpeg_received);

    #define JPEG_WORK_BUF_SIZE (64 * 1024)
    static uint8_t *workbuf = NULL;
    if (!workbuf) {
        workbuf = heap_caps_malloc(JPEG_WORK_BUF_SIZE, MALLOC_CAP_SPIRAM);
        if (!workbuf) workbuf = malloc(JPEG_WORK_BUF_SIZE);
        if (!workbuf) {
            ESP_LOGE(TAG, "No memory for JPEG work buffer");
            return;
        }
    }

    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = jpeg_buffer,
        .indata_size = jpeg_received,
        .outbuf = NULL,
        .outbuf_size = 0,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = { .swap_color_bytes = 0 },
        .advanced = {
            .working_buffer = workbuf,
            .working_buffer_size = JPEG_WORK_BUF_SIZE,
        }
    };

    esp_jpeg_image_output_t out_info;
    esp_err_t err = esp_jpeg_get_image_info(&jpeg_cfg, &out_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get JPEG info");
        return;
    }

    if (out_info.width != LCD_H_RES || out_info.height != LCD_V_RES) {
        ESP_LOGW(TAG, "JPEG size %ux%u != display %ux%u. Scaling not implemented.",
                 out_info.width, out_info.height, LCD_H_RES, LCD_V_RES);
        return;
    }

    size_t outbuf_size = out_info.width * out_info.height * 2;
    uint8_t *decoded_buf = heap_caps_malloc(outbuf_size, MALLOC_CAP_SPIRAM);
    if (!decoded_buf) decoded_buf = malloc(outbuf_size);
    if (!decoded_buf) {
        ESP_LOGE(TAG, "No memory for decoded image");
        return;
    }

    jpeg_cfg.outbuf = decoded_buf;
    jpeg_cfg.outbuf_size = outbuf_size;
    err = esp_jpeg_decode(&jpeg_cfg, &out_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode failed");
        free(decoded_buf);
        return;
    }

    // ========== ИЗМЕНЕНО: сохраняем оригинал, выводим и рисуем индикатор ==========
    if (original_frame_buffer) {
        memcpy(original_frame_buffer, decoded_buf, outbuf_size);
    }
    display_image((uint16_t*)decoded_buf, outbuf_size);
    update_ble_indicator(true);   // перерисовать индикатор поверх нового изображения
    // ========== КОНЕЦ ИЗМЕНЕНИЯ ==========

    save_received_image();
    free(decoded_buf);
    ESP_LOGI(TAG, "JPEG displayed");
}

// ==================== BLE CALLBACK ====================
static int rx_char_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (attr_handle == audio_char_handle && ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        
        is_receiving_jpg = true; // БЛОКИРУЕМ ЗВУК//if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        size_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len == 0) return 0;

        uint8_t data[len];
        ble_hs_mbuf_to_flat(ctxt->om, data, len, NULL);

        uint8_t type = data[0];

        switch (type) {
            case PACKET_TYPE_IMAGE_START: {
                if (len < 4) break;
                total_packets = data[2] | (data[3] << 8);
                ESP_LOGI(TAG, "START: total packets %d", total_packets);
                packets_received = 0;
                jpeg_received = 0;
                receiving = true;

                if (!jpeg_buffer) {
                    jpeg_buffer = heap_caps_malloc(JPEG_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
                    if (!jpeg_buffer) jpeg_buffer = malloc(JPEG_BUFFER_SIZE);
                }
                display_clear();
                break;
            }
            case PACKET_TYPE_IMAGE_DATA: {
                if (!receiving || !jpeg_buffer) break;
                if (len <= 5) break;
                uint32_t data_len = len - 5;
                if (jpeg_received + data_len <= JPEG_BUFFER_SIZE) {
                    memcpy(jpeg_buffer + jpeg_received, data + 5, data_len);
                    jpeg_received += data_len;
                    packets_received++;
                }
                if (packets_received % 10 == 0) {
                    ESP_LOGI(TAG, "DATA: %d/%d packets, %u bytes",
                             packets_received, total_packets, jpeg_received);
                }
                break;
            }
            case PACKET_TYPE_IMAGE_END: {
                ESP_LOGI(TAG, "END: received %u bytes", jpeg_received);
                if (jpeg_buffer && jpeg_received > 0) {
                    ESP_LOGI(TAG, "Giving semaphore");
                    xSemaphoreGive(ble_data_sem);
                }
                receiving = false;
                is_receiving_jpg = false;
                break;
            }
            default:
            {
                is_receiving_jpg = false;
                ESP_LOGW(TAG, "Unknown packet type: %d", type);
            }
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// ==================== MAIN ====================
void app_main(void)
{
    ESP_LOGI(TAG, "Starting ST7701S...");

    i2c_and_expander_init();

    esp_lcd_panel_io_handle_t io_handle = init_3wire_spi();
    if (!io_handle) return;

    const int rgb_data_pins[] = {
        LCD_R0_GPIO, LCD_R1_GPIO, LCD_R2_GPIO, LCD_R3_GPIO, LCD_R4_GPIO,
        LCD_G0_GPIO, LCD_G1_GPIO, LCD_G2_GPIO, LCD_G3_GPIO, LCD_G4_GPIO, LCD_G5_GPIO,
        LCD_B0_GPIO, LCD_B1_GPIO, LCD_B2_GPIO, LCD_B3_GPIO, LCD_B4_GPIO
    };

    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = PCLK_FREQ_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_pulse_width = HSYNC_PULSE_WIDTH,
            .hsync_back_porch = HSYNC_BACK_PORCH,
            .hsync_front_porch = HSYNC_FRONT_PORCH,
            .vsync_pulse_width = VSYNC_PULSE_WIDTH,
            .vsync_back_porch = VSYNC_BACK_PORCH,
            .vsync_front_porch = VSYNC_FRONT_PORCH,
            .flags = {
                .hsync_idle_low = HSYNC_IDLE_LOW,
                .vsync_idle_low = VSYNC_IDLE_LOW,
                .de_idle_high = DE_IDLE_HIGH,
                .pclk_active_neg = PCLK_ACTIVE_NEG,
                .pclk_idle_high = 0,
            }
        },
        .data_width = 16,
        .bits_per_pixel = 16,
        .num_fbs = 2,
        .bounce_buffer_size_px = 0,
        .psram_trans_align = 64,
        .hsync_gpio_num = LCD_HSYNC_GPIO,
        .vsync_gpio_num = LCD_VSYNC_GPIO,
        .de_gpio_num = LCD_DE_GPIO,
        .pclk_gpio_num = LCD_PCLK_GPIO,
        .disp_gpio_num = -1,
        .data_gpio_nums = {
            rgb_data_pins[0], rgb_data_pins[1], rgb_data_pins[2], rgb_data_pins[3],
            rgb_data_pins[4], rgb_data_pins[5], rgb_data_pins[6], rgb_data_pins[7],
            rgb_data_pins[8], rgb_data_pins[9], rgb_data_pins[10], rgb_data_pins[11],
            rgb_data_pins[12], rgb_data_pins[13], rgb_data_pins[14], rgb_data_pins[15]
        },
        .flags = { .fb_in_psram = 1 }
    };

    st7701_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7701_lcd_init_cmd_t),
        .flags = { .mirror_by_cmd = 0, .enable_io_multiplex = 0 }
    };

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };

    panel_handle = NULL;
    esp_err_t err = esp_lcd_new_panel_st7701(io_handle, &panel_config, &panel_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ST7701 panel: %d", err);
        return;
    }

    err = esp_lcd_panel_init(panel_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init panel: %d", err);
        return;
    }

    uint8_t madctl_final = 0x28;
    ESP_LOGI(TAG, "Applying MADCTL=0x%02X", madctl_final);
    esp_lcd_panel_io_tx_param(io_handle, 0x36, &madctl_final, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "ST7701 panel initialized");

    // ===== Кнопка =====
    gpio_config_t btn_gpio = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_gpio);
    ESP_LOGI(TAG, "Button configured on GPIO %d", BUTTON_GPIO);

    // ===== Подсветка =====
    gpio_config_t bl_gpio = {
        .pin_bit_mask = 1ULL << BACKLIGHT_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&bl_gpio);
    gpio_set_level(BACKLIGHT_GPIO, 1);
    ESP_LOGI(TAG, "Backlight configured on GPIO %d (initial ON)", BACKLIGHT_GPIO);

    void *fb0 = NULL, *fb1 = NULL;
    esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &fb0, &fb1);
    frame_buffer = (uint16_t *)fb0;

    // ========== ДОБАВЛЕНО: выделение оригинального буфера и копирование тестового изображения ==========
    original_frame_buffer = heap_caps_malloc(LCD_H_RES * LCD_V_RES * 2, MALLOC_CAP_SPIRAM);
    if (!original_frame_buffer) {
        original_frame_buffer = malloc(LCD_H_RES * LCD_V_RES * 2);
        ESP_LOGI(TAG, "Original frame buffer allocated in internal RAM");
    } else {
        ESP_LOGI(TAG, "Original frame buffer allocated in PSRAM");
    }
    if (!original_frame_buffer) {
        ESP_LOGE(TAG, "Failed to allocate original frame buffer, BLE indicator disabled");
    } else {
        // Копируем тестовое изображение в оригинальный буфер и во frame_buffer
        memcpy(original_frame_buffer, test_image, LCD_H_RES * LCD_V_RES * 2);
    }
    // ========== КОНЕЦ ДОБАВЛЕНИЯ ==========

    memset(frame_buffer, 0, LCD_H_RES * LCD_V_RES * 2);
    ESP_LOGI(TAG, "Copying image to frame buffer...");
    memcpy(frame_buffer, test_image, IMG_W * IMG_H * 2);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, frame_buffer);
    if (fb1) memcpy(fb1, test_image, IMG_W * IMG_H * 2);
    ESP_LOGI(TAG, "Image displayed. System idle.");

    // ========== ДОБАВЛЕНО: первичное рисование индикатора ==========
    if (original_frame_buffer) {
        update_ble_indicator(true);
    }
    // ========== КОНЕЦ ДОБАВЛЕНИЯ ==========

    mount_spiffs();
    load_and_display_saved_image();

    ble_data_sem = xSemaphoreCreateBinary();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_i2s();

    ble_init();

    xTaskCreate(audio_stream_task, "audio", 8192, NULL, 5, NULL);

    int last_button_state = 0;
    TickType_t last_debounce_time = 0;
    const TickType_t debounce_delay = pdMS_TO_TICKS(50);

    while (1) {
        // ========== ДОБАВЛЕНО: обновляем индикатор при изменении состояния BLE ==========
        if (original_frame_buffer) {
            update_ble_indicator(false);
        }
        // ========== КОНЕЦ ДОБАВЛЕНИЯ ==========

        if (xSemaphoreTake(ble_data_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
            process_received_jpeg();
        }

        int current_button_state = gpio_get_level(BUTTON_GPIO);

        if (current_button_state != last_button_state) {
            last_debounce_time = xTaskGetTickCount();
            last_button_state = current_button_state;
        }

        if ((xTaskGetTickCount() - last_debounce_time) > debounce_delay) {
            if (current_button_state == 1) {
                static bool display_on = true;
                display_on = !display_on;
                ESP_LOGI(TAG, "Button pressed, toggling display to %s", display_on ? "ON" : "OFF");

                if (display_on) {
                    // Включаем дисплей (отправляем команду DISPLAY ON через панель)
                    esp_lcd_panel_disp_off(panel_handle, false); // false = включить
                    gpio_set_level(BACKLIGHT_GPIO, 1);
                    // ========== ДОБАВЛЕНО: перерисовать индикатор после включения ==========
                    if (original_frame_buffer) {
                        update_ble_indicator(true);
                    }
                    // ========== КОНЕЦ ДОБАВЛЕНИЯ ==========
                } else {
                    // Выключаем дисплей
                    esp_lcd_panel_disp_off(panel_handle, true);  // true = выключить
                    gpio_set_level(BACKLIGHT_GPIO, 0);
                }

                vTaskDelay(pdMS_TO_TICKS(200));
                while (gpio_get_level(BUTTON_GPIO) == 1) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                last_debounce_time = xTaskGetTickCount();
            }
        }
    }
}
