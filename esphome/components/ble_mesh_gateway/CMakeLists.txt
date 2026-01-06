idf_component_register(
    SRCS "ble_mesh_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_event
)
