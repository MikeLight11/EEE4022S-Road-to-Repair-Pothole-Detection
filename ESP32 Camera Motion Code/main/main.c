// MPU6050 raw IMU streamer for ESP32-C3 (ESP-IDF, native driver/i2c_master.h)
//
// I2C pins: SCL = GPIO4, SDA = GPIO3
// Output UART: TX = GPIO5 (to Pi pin 10 RXD), 921600 baud, 8N1
//
// Samples the MPU6050 at a fixed rate (driven by esp_timer, not the 100 Hz
// FreeRTOS tick) and streams every reading as one CSV line:
//
//   seq,t_us,ax,ay,az,temp,gx,gy,gz
//
//   seq   - sample counter, increments every tick even if a read fails,
//           so a gap in seq on the Pi means a dropped sample
//   t_us  - esp_timer_get_time() at the moment of the read (microseconds)
//   ax..  - raw int16 register values, no offsets or filtering applied
//   temp  - raw temperature register (deg C = temp / 340.0 + 36.53)
//
// Lines starting with '#' are metadata (scales, rates) sent at boot.
// Conversions on the Pi:  accel_g = a / ACCEL_LSB_PER_G
//                         gyro_dps = g / GYRO_LSB_PER_DPS

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#define I2C_PORT        I2C_NUM_0
#define SDA_PIN         GPIO_NUM_3
#define SCL_PIN         GPIO_NUM_4
#define I2C_FREQ_HZ     400000

#define MPU6050_ADDR        0x68
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B

// Full-scale ranges. +-2g clips easily when a wheel hits a pothole, so use +-4g.
#define ACCEL_FS_SEL        1       // 0=2g 1=4g 2=8g 3=16g
#define GYRO_FS_SEL         1       // 0=250 1=500 2=1000 3=2000 deg/s
#define ACCEL_LSB_PER_G     8192.0f // must match ACCEL_FS_SEL
#define GYRO_LSB_PER_DPS    65.5f   // must match GYRO_FS_SEL

// On-chip digital low-pass filter. Acts as an anti-aliasing filter for the
// sample rate below (vibration above Nyquist would alias and can't be removed
// offline). 3 = 44 Hz accel / 42 Hz gyro bandwidth.
#define DLPF_CFG            3

#define SAMPLE_RATE_HZ      10

// 1 = once per second, print the latest sample + read-failure count to the
// USB-C console (idf.py monitor) so you can see the ESP is alive.
// Set to 0 for driving if nothing is plugged into USB.
#define DEBUG_CONSOLE       1

// Second UART used purely to output IMU data, separate from the console.
#define OUT_UART_PORT   UART_NUM_1
#define OUT_UART_TX_PIN GPIO_NUM_5
#define OUT_UART_RX_PIN GPIO_NUM_21
#define OUT_UART_BAUD   9600

static const char *TAG = "imu_raw";

typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu_raw_t;

static i2c_master_dev_handle_t mpu_handle;
static TaskHandle_t sample_task_handle;

static esp_err_t out_uart_init(void)
{
    uart_config_t conf = {
        .baud_rate = OUT_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // TX buffer so uart_write_bytes returns immediately instead of blocking
    // until the bytes are on the wire.
    esp_err_t err = uart_driver_install(OUT_UART_PORT, 256, 4096, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_param_config(OUT_UART_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return uart_set_pin(OUT_UART_PORT, OUT_UART_TX_PIN, OUT_UART_RX_PIN,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t bus_conf = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_conf, &bus_handle);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_conf, &mpu_handle);
}

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(mpu_handle, buf, sizeof(buf), 100);  // timeout in ms, not ticks
}

static esp_err_t mpu6050_configure(void)
{
    // Wake up, clock from X gyro PLL (more stable than the internal oscillator).
    esp_err_t err = mpu6050_write_reg(REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // With DLPF enabled the internal rate is 1 kHz; we poll it at SAMPLE_RATE_HZ.
    if ((err = mpu6050_write_reg(REG_SMPLRT_DIV, 0x00)) != ESP_OK) return err;
    if ((err = mpu6050_write_reg(REG_CONFIG, DLPF_CFG)) != ESP_OK) return err;
    if ((err = mpu6050_write_reg(REG_GYRO_CONFIG, GYRO_FS_SEL << 3)) != ESP_OK) return err;
    return mpu6050_write_reg(REG_ACCEL_CONFIG, ACCEL_FS_SEL << 3);
}

static esp_err_t mpu6050_read_raw(mpu_raw_t *out)
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t data[14];

    esp_err_t err = i2c_master_transmit_receive(mpu_handle, &reg, 1, data, sizeof(data),
                                                 10);  // timeout in ms, not ticks
    if (err != ESP_OK) {
        return err;
    }

    out->ax   = (int16_t)((data[0] << 8) | data[1]);
    out->ay   = (int16_t)((data[2] << 8) | data[3]);
    out->az   = (int16_t)((data[4] << 8) | data[5]);
    out->temp = (int16_t)((data[6] << 8) | data[7]);
    out->gx   = (int16_t)((data[8] << 8) | data[9]);
    out->gy   = (int16_t)((data[10] << 8) | data[11]);
    out->gz   = (int16_t)((data[12] << 8) | data[13]);
    return ESP_OK;
}

static void out_write(const char *s)
{
    uart_write_bytes(OUT_UART_PORT, s, strlen(s));
}

static void send_header(void)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "# imu_raw rate_hz=%d dlpf_cfg=%d\n", SAMPLE_RATE_HZ, DLPF_CFG);
    out_write(buf);
    snprintf(buf, sizeof(buf), "# accel_lsb_per_g=%.1f gyro_lsb_per_dps=%.1f\n",
             ACCEL_LSB_PER_G, GYRO_LSB_PER_DPS);
    out_write(buf);
    out_write("# seq,t_us,ax,ay,az,temp,gx,gy,gz\n");
}

// esp_timer callback: just wake the sampling task.
static void sample_timer_cb(void *arg)
{
    xTaskNotifyGive(sample_task_handle);
}

static void sample_task(void *arg)
{
    uint32_t seq = 0;
    uint32_t read_failures = 0;
    esp_err_t last_err = ESP_OK;
    uint32_t rx_bytes = 0;
    mpu_raw_t raw;
    char line[96];
    int len = 0;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t t_us = esp_timer_get_time();
        esp_err_t err = mpu6050_read_raw(&raw);
        if (err == ESP_OK) {
            len = snprintf(line, sizeof(line), "%lu,%lld,%d,%d,%d,%d,%d,%d,%d\n",
                           (unsigned long)seq, (long long)t_us,
                           raw.ax, raw.ay, raw.az, raw.temp,
                           raw.gx, raw.gy, raw.gz);
            uart_write_bytes(OUT_UART_PORT, line, len);
        } else {
            read_failures++;
            last_err = err;
        }
        seq++;

#if DEBUG_CONSOLE
        // Wiring self-test: jumper TX (GPIO5) to RX (GPIO21) and rx_bytes/s
        // should roughly match the bytes being sent. 0 = nothing leaving GPIO5.
        uint8_t rx_tmp[128];
        int rx_len;
        while ((rx_len = uart_read_bytes(OUT_UART_PORT, rx_tmp, sizeof(rx_tmp), 0)) > 0) {
            rx_bytes += rx_len;
        }
        if (seq % SAMPLE_RATE_HZ == 0) {
            printf("[UART1 TX=GPIO%d %d baud] read_failures=%lu (%s) loopback_rx=%lu B/s  last: %s",
                   OUT_UART_TX_PIN, OUT_UART_BAUD, (unsigned long)read_failures,
                   esp_err_to_name(last_err), (unsigned long)rx_bytes,
                   len ? line : "(no successful read yet)\n");
            rx_bytes = 0;
        }
#endif

        // Re-send the header every ~10 s so a Pi that starts reading
        // mid-stream still gets the scale factors.
        if (seq % (SAMPLE_RATE_HZ * 10) == 0) {
            send_header();
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(out_uart_init());

    if (mpu6050_configure() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not responding on SDA=GPIO%d SCL=GPIO%d", SDA_PIN, SCL_PIN);
        out_write("# ERROR mpu6050 not responding\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "MPU6050 configured, streaming at %d Hz on UART%d @ %d baud",
             SAMPLE_RATE_HZ, OUT_UART_PORT, OUT_UART_BAUD);
    send_header();

    xTaskCreate(sample_task, "imu_sample", 4096, NULL, configMAX_PRIORITIES - 2,
                &sample_task_handle);

    const esp_timer_create_args_t timer_args = {
        .callback = sample_timer_cb,
        .name = "imu_sample",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / SAMPLE_RATE_HZ));
}
