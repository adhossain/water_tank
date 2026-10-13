##
# @file CMakeLists.txt
# @brief Water Tank Controller
#/
# APP_PATH
set(APP_PATH ${CMAKE_CURRENT_LIST_DIR})
# APP_NAME
get_filename_component(APP_NAME ${APP_PATH} NAME)
# APP_SRCS
aux_source_directory(${APP_PATH}/src APP_SRCS)
# APP_INC
set(APP_INC
    ${APP_PATH}/src
    ${APP_PATH}/include
)
########################################
# Target Configure
########################################
add_library(${EXAMPLE_LIB})
target_sources(${EXAMPLE_LIB}
    PRIVATE
        ${APP_SRCS}
    )
target_include_directories(${EXAMPLE_LIB}
    PRIVATE
        ${APP_INC}
    )
########################################
# Tuya credentials from .env (git-ignored)
########################################
# .env holds KEY=VALUE lines; see .env.example
set(APP_ENV_FILE ${APP_PATH}/.env)
set(APP_ENV_KEYS TUYA_PRODUCT_ID TUYA_DEVICE_UUID TUYA_DEVICE_AUTHKEY)
if(EXISTS ${APP_ENV_FILE})
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${APP_ENV_FILE})
    file(STRINGS ${APP_ENV_FILE} APP_ENV_LINES)
    foreach(line IN LISTS APP_ENV_LINES)
        if(line MATCHES "^[ \t]*([A-Z_]+)[ \t]*=[ \t]*\"?([^\"]*)\"?[ \t]*$")
            set(key ${CMAKE_MATCH_1})
            set(val ${CMAKE_MATCH_2})
            if(key IN_LIST APP_ENV_KEYS AND NOT val STREQUAL "")
                target_compile_definitions(${EXAMPLE_LIB} PRIVATE ${key}="${val}")
            endif()
        endif()
    endforeach()
else()
    message(WARNING "water_tank: ${APP_ENV_FILE} not found - copy .env.example to .env and fill in your Tuya credentials")
endif()
