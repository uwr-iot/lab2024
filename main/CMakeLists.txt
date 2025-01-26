idf_component_register(
    SRCS    "app_main.c"

    INCLUDE_DIRS    ${CMAKE_SOURCE_DIR}/main

    REQUIRES soc nvs_flash driver console esp_wifi esp_http_client
)
