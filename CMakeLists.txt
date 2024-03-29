#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

set(BOARD_ROOT ${CMAKE_SOURCE_DIR})
set(ZEPHYR_EXTRA_MODULES 
    ${CMAKE_SOURCE_DIR}/lib/sipf
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sipf_lib_sample_app_rx)

target_sources(app PRIVATE
    src/main.c
    src/uart_broker.c
)

target_include_directories(app PRIVATE
    include/
)

if(SIPF_ENVIRONMENT STREQUAL "production")
    message("Set -Werror")
    zephyr_library_compile_options("-Werror")
endif()
if(SIPF_ENVIRONMENT STREQUAL "staging")
    message("Set -Werror")
    zephyr_library_compile_options("-Werror")
endif()
