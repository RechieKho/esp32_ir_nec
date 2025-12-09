/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "ir_nec.h"

#define DEFAULT_TASK_STACK_DEPTH (2048)
#define DEFAULT_PRIORITY (tskIDLE_PRIORITY + 10)

#define IR_TX_GPIO_NUM (18)
#define IR_RX_GPIO_NUM (19)
#define BUTTON_INPUT_GPIO_NUM (GPIO_NUM_10)

static const char *TAG = "main";

static rmt_channel_handle_t rx_channel = NULL;
static uint16_t s_last_nec_code_address = 0x0000;
static uint16_t s_last_nec_code_command = 0x0000;
static rmt_symbol_word_t raw_symbols[IR_NEC_RMT_SYMBOL_COUNT];

static QueueHandle_t input_queue = NULL;
static QueueHandle_t receive_queue = NULL;

static void on_button_pressed()
{
    // TODO: Do something.
}

static void on_nec_code_received()
{
    // TODO: Do something.
    printf("Address=%04X, Command=%04X, repeat\r\n\r\n", s_last_nec_code_address, s_last_nec_code_command);
}

static void on_unknown_nec_code_received()
{
    // Ignored.
}

static void button_task(void *arg)
{
    gpio_num_t input_gpio_num;
    while (true)
    {
        if (xQueueReceive(input_queue, &input_gpio_num, pdMS_TO_TICKS(1000)) != pdPASS)
            continue;
        if (input_gpio_num != BUTTON_INPUT_GPIO_NUM)
            continue;

        on_button_pressed();
    }
}

static void ir_receiver_task(void *arg)
{
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &gc_ir_nec_receive_config));
    rmt_rx_done_event_data_t done_event_data;
    while (true)
    {
        if (xQueueReceive(receive_queue, &done_event_data, pdMS_TO_TICKS(1000)) != pdPASS)
            continue;

        // decode RMT symbols
        switch (done_event_data.num_symbols)
        {
        case 34: // NEC normal frame
            if (ir_nec_parse_frame(done_event_data.received_symbols, &s_last_nec_code_address, &s_last_nec_code_command))
                on_nec_code_received();
            break;
        case 2: // NEC repeat frame
            if (ir_nec_is_repeat_code(done_event_data.received_symbols))
                on_nec_code_received();
            break;
        default:
            on_unknown_nec_code_received();
            break;
        }
        ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &gc_ir_nec_receive_config));
    }
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *done_event_data, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, done_event_data, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void gpio_isr_handler(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;
    gpio_num_t gpio_num = (gpio_num_t)arg;
    xQueueSendFromISR(input_queue, &gpio_num, &high_task_wakeup);
}

void app_main(void)
{
    input_queue = xQueueCreate(1, sizeof(gpio_num_t));
    assert(input_queue);
    receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);

    ESP_LOGI(TAG, "Setup Button.");
    gpio_config_t button_config = {};
    button_config.pin_bit_mask = (1ULL << BUTTON_INPUT_GPIO_NUM);
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&button_config);

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add((1ULL << BUTTON_INPUT_GPIO_NUM), gpio_isr_handler, (void *)(BUTTON_INPUT_GPIO_NUM)));

    ESP_LOGI(TAG, "Setup RMT RX channel.");
    rmt_rx_channel_config_t rx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_NEC_RESOLUTION_HZ,
        .mem_block_symbols = IR_NEC_RMT_SYMBOL_COUNT,
        .gpio_num = IR_RX_GPIO_NUM,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL));

    ESP_LOGI(TAG, "Setup RMT TX channel.");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_NEC_RESOLUTION_HZ,
        .mem_block_symbols = IR_NEC_RMT_SYMBOL_COUNT,
        .trans_queue_depth = 4, // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = IR_TX_GPIO_NUM,
    };
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &gc_ir_nec_carrier_cfg));

    ESP_LOGI(TAG, "Setup IR NEC encoder.");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = IR_NEC_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(ir_nec_new_rmt_encoder(&nec_encoder_cfg, &nec_encoder));

    ESP_LOGI(TAG, "Enable RMT TX and RX channels.");
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    xTaskCreate(button_task, "button_task", DEFAULT_TASK_STACK_DEPTH, NULL, DEFAULT_PRIORITY, NULL);
    xTaskCreate(ir_receiver_task, "ir_receiver_task", DEFAULT_TASK_STACK_DEPTH, NULL, DEFAULT_PRIORITY - 5, NULL);

    // ready to receive
    while (1)
    {
        vTaskDelay((1000 / 60) / portTICK_PERIOD_MS);

        // rmt_transmit_config_t transmit_config = {
        //     .loop_count = 0, // no loop
        // };
        //  const ir_nec_scan_code_t scan_code = {
        //      .address = 0x0440,
        //      .command = 0x3003,
        //  };
        //  ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
    }
}
