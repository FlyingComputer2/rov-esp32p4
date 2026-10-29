# rover_sample: RC car applet (steering servo, BTS7960, GY-521, GY-271) with
# MSP v2 RC / telemetry on a datagram transport. core/ is portable C++ and
# host-tested (tests/); src/ is the ESP-IDF side.
idf_component_register(
    SRCS core/msp.cpp
         core/protocol.cpp
         core/params.cpp
         core/controller.cpp
         core/ahrs.cpp
         core/rc_link.cpp
         src/drivers.cpp
         src/rover_sample.cpp
    INCLUDE_DIRS include
    PRIV_INCLUDE_DIRS core src
    PRIV_REQUIRES esp_driver_i2c esp_driver_ledc esp_driver_gptimer esp_driver_gpio esp_adc
                  esp_timer nvs_flash lwip pthread esp_rom hal soc log
)
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
