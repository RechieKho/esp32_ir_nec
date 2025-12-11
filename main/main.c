/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <sys/time.h>

#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "protocol_examples_utils.h"
#include "esp_tls_crypto.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_check.h"

#if !CONFIG_IDF_TARGET_LINUX
#include <esp_wifi.h>
#include <esp_system.h>

#include "nvs_flash.h"
#include "esp_eth.h"
#endif // !CONFIG_IDF_TARGET_LINUX

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "ir_nec.h"

#define ARRAY_LENGTH(arr) (sizeof(arr) / sizeof(arr[0]))

#define DEFAULT_TASK_STACK_DEPTH (2048)
#define DEFAULT_PRIORITY (tskIDLE_PRIORITY + 10)
#define DEFAULT_TRANSMISSION_QUEUE_DEPTH (4)

#define IR_RX_GPIO_NUM (19)
#define BUTTON_INPUT_GPIO_NUM (GPIO_NUM_10)

typedef struct web_control_t
{
    ir_nec_scan_code_t scan_code;
} web_control_t;

typedef struct transmitter_config_t
{
    gpio_num_t channel_gpio_num;
    gpio_num_t hint_gpio_num;
} transmitter_config_t;

typedef struct transmitter_t
{
    rmt_channel_handle_t channel;
    gpio_num_t hint_gpio_num;
} transmitter_t;

static esp_err_t web_control_interface_root_uri_get_handler(httpd_req_t *req);
static esp_err_t web_control_interface_control_uri_post_handler(httpd_req_t *req);

static const char *TAG = "main";
static const transmitter_config_t TRANSMITTER_CONFIGS[] = {
    {.channel_gpio_num = 18,
     .hint_gpio_num = 10},
    {.channel_gpio_num = 20,
     .hint_gpio_num = 11},
    {.channel_gpio_num = 21,
     .hint_gpio_num = 12},
    {.channel_gpio_num = 22,
     .hint_gpio_num = 13},
    {.channel_gpio_num = 23,
     .hint_gpio_num = 14},
    {.channel_gpio_num = 24,
     .hint_gpio_num = 15},
    {.channel_gpio_num = 25,
     .hint_gpio_num = 16}};
static const httpd_uri_t WEB_CONTROL_INTERFACE_ROOT_URI = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = web_control_interface_root_uri_get_handler,
    .user_ctx = NULL};
static const httpd_uri_t WEB_CONTROL_INTERFACE_CONTROL_URI = {
    .uri = "/control",
    .method = HTTP_POST,
    .handler = web_control_interface_control_uri_post_handler,
    .user_ctx = NULL};

static transmitter_t s_transmitters[ARRAY_LENGTH(TRANSMITTER_CONFIGS)];
static rmt_channel_handle_t s_rx_channel = NULL;
static rmt_encoder_handle_t s_nec_encoder = NULL;
static uint16_t s_last_nec_valid_code_address = 0x0000;
static uint16_t s_last_nec_valid_code_command = 0x0000;
static rmt_symbol_word_t s_raw_symbols[IR_NEC_RMT_SYMBOL_COUNT];
static size_t s_transmitter_selector = ARRAY_LENGTH(TRANSMITTER_CONFIGS); //<* When it is between `0` to `(ARRAY_LENGTH(TRANSMITTER_CONFIGS) - 1)`, it is used as an index with the `s_transmitters[s_transmitter_selector]` be used. When it is `ARRAY_LENGTH(TRANSMITTER_CONFIGS)`, all s_transmitters are used.
static httpd_handle_t s_server = NULL;

static QueueHandle_t s_input_queue = NULL;
static QueueHandle_t s_receive_queue = NULL;
static QueueHandle_t s_web_control_queue = NULL;

static httpd_handle_t start_server(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
#if CONFIG_IDF_TARGET_LINUX
    // Setting port as 8001 when building for Linux. Port 80 can be used only by a privileged user in linux.
    // So when a unprivileged user tries to run the application, it throws bind error and the server is not started.
    // Port 8001 can be used by an unprivileged user as well. So the application will not throw bind error and the
    // server will be started.
    config.server_port = 8001;
#endif // !CONFIG_IDF_TARGET_LINUX
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &WEB_CONTROL_INTERFACE_ROOT_URI);
        httpd_register_uri_handler(server, &WEB_CONTROL_INTERFACE_CONTROL_URI);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_server(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void transmit_nec(ir_nec_scan_code_t scan_code)
{
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };
    for (size_t i = 0; i < ARRAY_LENGTH(TRANSMITTER_CONFIGS); i++)
        if (s_transmitter_selector == ARRAY_LENGTH(TRANSMITTER_CONFIGS))
            ESP_ERROR_CHECK(rmt_transmit(s_transmitters[i].channel, s_nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
        else if (s_transmitter_selector == i)
            ESP_ERROR_CHECK(rmt_transmit(s_transmitters[i].channel, s_nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
}

static void update_hints()
{
    for (size_t i = 0; i < ARRAY_LENGTH(TRANSMITTER_CONFIGS); i++)
        gpio_set_level(s_transmitters[i].hint_gpio_num, (s_transmitter_selector == ARRAY_LENGTH(TRANSMITTER_CONFIGS) ? true : s_transmitter_selector == i));
}

static void on_button_pressed()
{
    s_transmitter_selector = (s_transmitter_selector + 1) % (ARRAY_LENGTH(TRANSMITTER_CONFIGS) + 1);
    update_hints();
}

static void on_nec_code_received()
{
    ESP_LOGI(TAG, "NEC Code Received: address=%04X, command=%04X.", s_last_nec_valid_code_address, s_last_nec_valid_code_command);
    const ir_nec_scan_code_t scan_code = {
        .address = s_last_nec_valid_code_address,
        .command = s_last_nec_valid_code_command,
    };
    transmit_nec(scan_code);
}

static void on_unknown_nec_code_received()
{
    // Ignored.
}

static void on_web_control_received(web_control_t web_control)
{
    ESP_LOGI(TAG, "Web Control Received: address=%04X, command=%04X.", web_control.scan_code.address, web_control.scan_code.command);
    transmit_nec(web_control.scan_code);
}

static void gpio_input_task(void *arg)
{
    gpio_num_t input_gpio_num;
    while (true)
    {
        if (xQueueReceive(s_input_queue, &input_gpio_num, pdMS_TO_TICKS(1000)) != pdPASS)
            continue;
        if (input_gpio_num != BUTTON_INPUT_GPIO_NUM)
            continue;

        on_button_pressed();
    }
}

static void ir_receiver_task(void *arg)
{
    ESP_ERROR_CHECK(rmt_receive(s_rx_channel, s_raw_symbols, sizeof(s_raw_symbols), &gc_ir_nec_receive_config));
    rmt_rx_done_event_data_t done_event_data;
    while (true)
    {
        if (xQueueReceive(s_receive_queue, &done_event_data, pdMS_TO_TICKS(1000)) != pdPASS)
            continue;

        // decode RMT symbols
        switch (done_event_data.num_symbols)
        {
        case 34: // NEC normal frame
            if (ir_nec_parse_frame(done_event_data.received_symbols, &s_last_nec_valid_code_address, &s_last_nec_valid_code_command) == ESP_OK)
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
        ESP_ERROR_CHECK(rmt_receive(s_rx_channel, s_raw_symbols, sizeof(s_raw_symbols), &gc_ir_nec_receive_config));
    }
}

static void web_control_task(void *arg)
{
    web_control_t control;
    while (true)
    {
        if (xQueueReceive(s_web_control_queue, &control, pdMS_TO_TICKS(1000)) != pdPASS)
            continue;

        on_web_control_received(control);
    }
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *done_event_data, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(s_receive_queue, done_event_data, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void gpio_isr_handler(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;
    gpio_num_t gpio_num = (gpio_num_t)arg;
    xQueueSendFromISR(s_input_queue, &gpio_num, &high_task_wakeup);
}

static esp_err_t web_control_interface_root_uri_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    // Read the HTML file from the embedded data
    extern const char index_html_start[] asm("_binary_index_html_start");
    extern const char index_html_end[] asm("_binary_index_html_end");

    const size_t html_size = index_html_end - index_html_start;

    // Send response
    return httpd_resp_send(req, index_html_start, html_size);
}

static esp_err_t web_control_interface_control_uri_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;
    char *command = NULL;
    char *address = NULL;

    if (remaining > 0)
    {
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf) - 1))) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                httpd_resp_send_408(req);
            }
            return ESP_FAIL;
        }
        buf[ret] = '\0';

        char *param = strtok(buf, "&");
        while (param != NULL)
        {
            if (strncmp(param, "command=", 8) == 0)
            {
                command = param + 8;
            }
            else if (strncmp(param, "address=", 8) == 0)
            {
                address = param + 8;
            }
            param = strtok(NULL, "&");
        }

        if (command && address)
        {
            web_control_t control;
            control.scan_code.command = (uint16_t)strtol(command, NULL, 16);
            control.scan_code.address = (uint16_t)strtol(address, NULL, 16);

            if (xQueueSend(s_web_control_queue, &control, pdMS_TO_TICKS(1000)) != pdTRUE)
            {
                ESP_LOGE(TAG, "Failed to consume web control.");
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }

            const char *resp_str = "Web control consumed successfully.";
            httpd_resp_send(req, resp_str, strlen(resp_str));
        }
        else
        {
            ESP_LOGE(TAG, "Invalid Web control.");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

#if !CONFIG_IDF_TARGET_LINUX
static void network_disconnect_handler(void *arg, esp_event_base_t event_base,
                                       int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Network disconnected.");
    if (!s_server)
        return;
    ESP_LOGI(TAG, "Stopping the web server.");
    if (stop_server(s_server) == ESP_OK)
        s_server = NULL;
    else
        ESP_LOGE(TAG, "Fail to stop the web server.");
}

static void network_connect_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Network connected.");
    if (s_server)
        return;
    ESP_LOGI(TAG, "Starting webserver");
    s_server = start_server();
}
#endif // !CONFIG_IDF_TARGET_LINUX

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_input_queue = xQueueCreate(1, sizeof(gpio_num_t));
    assert(s_input_queue);
    s_receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(s_receive_queue);
    s_web_control_queue = xQueueCreate(1, sizeof(web_control_t));
    assert(s_receive_queue);

    ESP_LOGI(TAG, "Setup Button.");
    gpio_config_t button_config = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = 1ULL << BUTTON_INPUT_GPIO_NUM,
    };
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
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &s_rx_channel));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s_rx_channel, &cbs, NULL));

    ESP_LOGI(TAG, "Setup s_transmitters.");
    rmt_tx_channel_config_t channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_NEC_RESOLUTION_HZ,
        .mem_block_symbols = IR_NEC_RMT_SYMBOL_COUNT,
        .trans_queue_depth = DEFAULT_TRANSMISSION_QUEUE_DEPTH, // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
    };
    gpio_config_t hint_gpio_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    for (size_t i = 0; i < ARRAY_LENGTH(TRANSMITTER_CONFIGS); i++)
    {
        channel_config.gpio_num = TRANSMITTER_CONFIGS[i].channel_gpio_num;
        ESP_ERROR_CHECK(rmt_new_tx_channel(&channel_config, &s_transmitters[i].channel));
        ESP_ERROR_CHECK(rmt_apply_carrier(s_transmitters[i].channel, &gc_ir_nec_carrier_cfg));

        hint_gpio_config.pin_bit_mask = 1ULL << TRANSMITTER_CONFIGS[i].hint_gpio_num;
        ESP_ERROR_CHECK(gpio_config(&hint_gpio_config));
        s_transmitters[i].hint_gpio_num = TRANSMITTER_CONFIGS[i].hint_gpio_num;
    }

    ESP_LOGI(TAG, "Setup IR NEC encoder.");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = IR_NEC_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(ir_nec_new_rmt_encoder(&nec_encoder_cfg, &s_nec_encoder));

    ESP_LOGI(TAG, "Enable RMT TX and RX channels.");
    for (size_t i = 0; i < ARRAY_LENGTH(TRANSMITTER_CONFIGS); i++)
        ESP_ERROR_CHECK(rmt_enable(s_transmitters[i].channel));
    ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
    update_hints();

    ESP_LOGI(TAG, "Setup Web Server.");
    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
#if !CONFIG_IDF_TARGET_LINUX
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &network_connect_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &network_disconnect_handler, NULL));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &network_connect_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &network_disconnect_handler, NULL));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET
#endif // !CONFIG_IDF_TARGET_LINUX
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    assert(xTaskCreate(gpio_input_task, "gpio_input_task", DEFAULT_TASK_STACK_DEPTH, NULL, DEFAULT_PRIORITY, NULL) == pdPASS);
    assert(xTaskCreate(ir_receiver_task, "ir_receiver_task", DEFAULT_TASK_STACK_DEPTH, NULL, DEFAULT_PRIORITY / 2, NULL) == pdPASS);
    assert(xTaskCreate(web_control_task, "web_control_task", DEFAULT_TASK_STACK_DEPTH * 2, NULL, DEFAULT_PRIORITY, NULL) == pdPASS);

    while (1)
    {
        vTaskDelay((1000 / 60) / portTICK_PERIOD_MS);
    }
}
