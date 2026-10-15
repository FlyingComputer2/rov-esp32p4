idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES winject_l3 winject_proxy esp_eth esp_netif esp_event
                                     nvs_flash spiffs pthread)
# /winject/winject.conf: the SPIFFS image is built from config/ and flashed
# with the app (idf.py flash).
spiffs_create_partition_image(storage ${CMAKE_CURRENT_LIST_DIR}/../config FLASH_IN_PROJECT)
