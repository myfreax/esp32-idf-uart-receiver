
#include <driver/uart.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define UART_PORT 2
static const char* TAG = "uart-read";

void app_main(void) {
  uart_config_t config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &config));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 17, 18, 20, 21));
  const int uart_buffer_size = (1024 * 2);
  QueueHandle_t uart_queue;
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, uart_buffer_size,
                                      uart_buffer_size, 10, &uart_queue, 0));
  uint8_t data[128];
  int length = 0;
  while (1) {
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT, (size_t*)&length));
    length = uart_read_bytes(UART_PORT, data, length, 100);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "RX Data Length: %d", length);
    if (length > 0) {
      ESP_LOGI(TAG, "RX: %s", data);
      memset(data, 0, 128);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}