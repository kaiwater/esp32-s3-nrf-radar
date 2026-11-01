idf_component_register(
    SRCS "main.c" "st7735.c" "nrf24.c" "radar.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd esp_lcd_st7735 esp_driver_spi esp_driver_gpio esp_timer
)
