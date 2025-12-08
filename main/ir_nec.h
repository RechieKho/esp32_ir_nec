/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0 9000
#define NEC_LEADING_CODE_DURATION_1 4500
#define NEC_PAYLOAD_ZERO_DURATION_0 560
#define NEC_PAYLOAD_ZERO_DURATION_1 560
#define NEC_PAYLOAD_ONE_DURATION_0 560
#define NEC_PAYLOAD_ONE_DURATION_1 1690
#define NEC_REPEAT_CODE_DURATION_0 9000
#define NEC_REPEAT_CODE_DURATION_1 22

#ifndef IR_NEC_RESOLUTION_HZ
#define IR_NEC_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us.
#endif

#ifndef IR_NEC_RMT_SYMBOL_COUNT
#define IR_NEC_RMT_SYMBOL_COUNT (64) // Typical RMT symbol count for a standard NEC frame.
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief RMT receive timing requirement based on NEC protocol.
     */
    extern const rmt_receive_config_t gc_ir_nec_receive_config;

    /**
     * @brief RMT carrier configuration based on NEC protocol.
     */
    extern const rmt_carrier_config_t gc_ir_nec_carrier_cfg;

    /**
     * @brief IR NEC scan code representation
     */
    typedef struct
    {
        uint16_t address;
        uint16_t command;
    } ir_nec_scan_code_t;

    /**
     * @brief Type of IR NEC encoder configuration
     */
    typedef struct
    {
        uint32_t resolution; /*!< Encoder resolution, in Hz */
    } ir_nec_encoder_config_t;

    /**
     * @brief Create RMT encoder for encoding IR NEC frame into RMT symbols.
     *
     * @param[in] config Encoder configuration.
     * @param[out] ret_encoder Returned encoder handle.
     * @return
     *      - ESP_ERR_INVALID_ARG for any invalid arguments.
     *      - ESP_ERR_NO_MEM out of memory when creating IR NEC encoder.
     *      - ESP_OK if creating encoder successfully.
     */
    esp_err_t ir_nec_new_rmt_encoder(const ir_nec_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

    /**
     * @brief Decode RMT symbols into NEC address and command
     *
     * @param[in] rmt_nec_symbols RMT symbols that contains NEC data.
     * @param[out] ret_nec_code_address Returned address code.
     * @param[out] ret_nec_code_command Returned command code.
     * @return
     *      - ESP_ERR_INVALID_ARG for invalid leading code of NEC data.
     *      - ESP_OK if creating encoder successfully.
     */
    esp_err_t ir_nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols, uint16_t *ret_nec_code_address, uint16_t *ret_nec_code_command);

    /**
     * @brief Check whether the RMT symbols represent NEC repeat code.
     *
     * @param[in] rmt_nec_symbols RMT symbols that contains NEC data.
     * @return true if RMT symbols represent NEC repeat code, else false.
     */
    bool ir_nec_is_repeat_code(rmt_symbol_word_t *rmt_nec_symbols);

#ifdef __cplusplus
}
#endif
