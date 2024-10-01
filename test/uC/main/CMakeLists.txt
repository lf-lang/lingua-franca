cmake_minimum_required(VERSION 3.22)

# Set toolchain to RISC-V; must be done before call to project()
include($ENV{FP_SDK_PATH}/cmake/riscv-toolchain.cmake)

project(lf-fp-uc
  LANGUAGES C
  DESCRIPTION "LF on FlexPRET using uC runtime"
  VERSION 1.0.0
)

add_executable(fp-lf-uc main.c)

target_link_libraries(fp-lf-uc PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../build/libFlexPRET_uC.a
    ${CMAKE_CURRENT_LIST_DIR}/../build/reactor-uc/libreactor-uc.a
)

set(DEBUG true)

add_subdirectory($ENV{FP_SDK_PATH} BINARY_DIR)
target_link_libraries(fp-lf-uc PRIVATE fp-sdk)

include($ENV{FP_SDK_PATH}/cmake/fp-app.cmake)

fp_add_outputs(fp-lf-uc)
