file(GLOB_RECURSE APP_SRCS "main.c" "*.c")

idf_component_register(
    SRCS ${APP_SRCS}
    INCLUDE_DIRS "."
)