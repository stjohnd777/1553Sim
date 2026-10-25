cmake_minimum_required(VERSION 3.20)
project(mil1553 LANGUAGES CXX)

add_library(mil1553
    src/BC.cpp
    src/BM.cpp
    src/RT.cpp
    src/hal/hal.cpp
)
add_library(mil1553::mil1553 ALIAS mil1553)

target_compile_features(mil1553 PUBLIC cxx_std_17)
set_target_properties(mil1553 PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(mil1553 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
