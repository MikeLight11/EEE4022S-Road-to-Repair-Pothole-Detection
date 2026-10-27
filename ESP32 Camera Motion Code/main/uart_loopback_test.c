// UART loopback test for ESP32.
//
// Wiring: jumper GPIO5 (TX) directly to GPIO6 (RX) on the board.
// Sends a counted test message every second on UART1 and checks that the
// same bytes come back on the RX pin, confirming the UART peripheral and
// wiring both work before relying on them elsewhere.
//
// To use: in main/CMakeLists.txt, swap "main.c" for
// "uart_loopback_test.c" in the SRCS list, then build/flash/monitor.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define LOOPBACK_UART_PORT  UART_NUM_1
#define LOOPBACK_TX_PIN     GPIO_NUM_5
#define LOOPBACK_RX_PIN     GPIO_NUM_6
#define LOOPBACK_BAUD       9600

static const char *TAG = "uart_loopback";

void app_main(void)
{
    uart_config_t conf = {
        .baud_rate = LOOPBACK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LOOPBACK_UART_PORT, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LOOPBACK_UART_PORT, &conf));
    ESP_ERROR_CHECK(uart_set_pin(LOOPBACK_UART_PORT, LOOPBACK_TX_PIN, LOOPBACK_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "Loopback test ready. TX=GPIO%d RX=GPIO%d. Jumper them together.",
             LOOPBACK_TX_PIN, LOOPBACK_RX_PIN);

    int count = 0;
    char tx_line[32];
    uint8_t rx_buf[32];

    while (1) {
        int tx_len = snprintf(tx_line, sizeof(tx_line), "loopback #%d\n", count++);
        uart_write_bytes(LOOPBACK_UART_PORT, tx_line, tx_len);

        int rx_len = uart_read_bytes(LOOPBACK_UART_PORT, rx_buf, sizeof(rx_buf) - 1,
                                      pdMS_TO_TICKS(500));
        if (rx_len <= 0) {
            ESP_LOGE(TAG, "No data received - check wiring (TX GPIO%d -> RX GPIO%d)",
                     LOOPBACK_TX_PIN, LOOPBACK_RX_PIN);
        } else {
            rx_buf[rx_len] = '\0';
            bool match = (rx_len == tx_len) && (memcmp(tx_line, rx_buf, tx_len) == 0);
            ESP_LOGI(TAG, "Sent: \"%.*s\" Received: \"%s\" -> %s",
                     tx_len - 1, tx_line, rx_buf, match ? "MATCH" : "MISMATCH");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
