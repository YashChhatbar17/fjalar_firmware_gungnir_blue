cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})


if(TEST_BUILD)
    set(EXTRA_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/testing/preferences.conf)
else()
    set(EXTRA_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/preferences.conf)
endif()

list(APPEND EXTRA_ZEPHYR_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dummysensor
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/hilsensor
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ms5611
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fjalar)

add_subdirectory(../common ../build/common)

add_subdirectory(lib)

zephyr_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/external_communication
)

# main
target_sources(app PRIVATE
    src/flight_state.c
    src/sensors.c
    src/actuation.c
    src/filter.c
    src/aerodynamics.c
    src/init.c
    src/control.c
    src/external_communication/com_master.c
    src/external_communication/com_can.c
    src/external_communication/com_flash.c
    src/external_communication/com_lora.c
    src/external_communication/com_uart.c
    src/external_communication/com_usb.c
)

if(TEST_BUILD)
    message(STATUS "Building test")
    target_sources(app PRIVATE
        testing/test_main.c
    )
else()
    message(STATUS "Building app")
    target_sources(app PRIVATE
        src/main.c
    )
endif()


target_link_libraries(app PRIVATE protocol)
target_link_libraries(app PRIVATE minmea)
