cmake_minimum_required(VERSION 3.29)
project(rp2040_bsec)

set(CMAKE_CXX_STANDARD 17)

set(BSEC_LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/cortex-m0+")
set(BSEC_INC_PATH "${CMAKE_CURRENT_LIST_DIR}/inc")
set(BME68x_INC_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/bme68x")

add_library(rp2040_bsec STATIC rp2040_bsec.cpp lib/bme68x/bme68x.c)

# Add the BSEC library file
add_library(bsec STATIC IMPORTED GLOBAL)
set_target_properties(bsec PROPERTIES IMPORTED_LOCATION "${BSEC_LIB_PATH}/libalgobsec.a")
set_target_properties(bsec PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BSEC_INC_PATH}")


add_subdirectory(lib/pico-i2c-device)

target_link_libraries(rp2040_bsec
        bsec
        pico_i2c_device
)

# Add the standard include files to the build
target_include_directories(rp2040_bsec PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${BSEC_INC_PATH}
        ${BME68x_INC_PATH}
)