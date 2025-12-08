idf_component_register(SRCS "main.c" "ir_nec.c"
                       PRIV_REQUIRES esp_driver_rmt esp_driver_gpio
                       INCLUDE_DIRS ".")
