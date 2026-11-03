# 网页源码只有一份：复用 offline_link/web/index.html
file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/www)
configure_file(${CMAKE_CURRENT_LIST_DIR}/../../offline_link/web/index.html
               ${CMAKE_CURRENT_LIST_DIR}/www/index.html COPYONLY)

idf_component_register(
    SRCS "app_main.c" "link.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "www/index.html"
    REQUIRES esp_http_server esp_wifi esp_netif nvs_flash esp_driver_i2s esp_driver_uart esp_timer
)
