file(GLOB_RECURSE APP_SRCS "main.c" "*.c")
list(APPEND APP_SRCS "${CMAKE_CURRENT_LIST_DIR}/driver/current_sensor/adc_etm_ll.c")
list(REMOVE_DUPLICATES APP_SRCS)

idf_component_register(
    SRCS ${APP_SRCS}
    INCLUDE_DIRS "."
)