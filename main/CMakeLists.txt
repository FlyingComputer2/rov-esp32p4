idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES winject_l3 app_bwtest krono winject_proxy vstreamer rover_sample applets esp_driver_gpio esp_driver_i2c esp_eth esp_netif esp_event esp_timer vfs
                                     nvs_flash spiffs pthread)
# bfc (the d-plane queues) throws on fatal errors; with IDF's C++ exceptions
# off a throw aborts.
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fexceptions>)

# P4 boot-hang workaround (see __wrap_esp_restart_noos in main.cpp): every software restart
# becomes an RTC watchdog system reset.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_restart_noos")

# /winject/winject.conf: the SPIFFS image is built from config/ and flashed
# with the app (idf.py flash).
spiffs_create_partition_image(storage ${CMAKE_CURRENT_LIST_DIR}/../config FLASH_IN_PROJECT)
