set(srcs "main.c"
        "board.c"
        "lamp_nvs.c"
        "http_server.c"
        "wifi_setup.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")