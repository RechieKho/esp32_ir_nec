set(requires esp_driver_rmt esp_driver_gpio esp-tls nvs_flash esp_netif esp_http_server)
idf_build_get_property(idf_target IDF_TARGET)

if(${idf_target} STREQUAL "linux")
    list(APPEND requires esp_stubs protocol_examples_common)
else()
    list(APPEND requires esp_wifi esp_eth)
endif()

idf_component_register(SRCS "main.c" "ir_nec.c"
                       PRIV_REQUIRES ${requires}
                       EMBED_FILES "index.html"
                       INCLUDE_DIRS ".")
