cmake_minimum_required(VERSION 3.20)
project(SimA LANGUAGES CXX)

# Find Eigen3 or fetch it as fallback
find_package(Eigen3 3.3 QUIET CONFIG)
if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.3 QUIET)
endif()

if(NOT TARGET Eigen3::Eigen)
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(eigen)
endif()

add_library(SimA
    src/sensor/ImuRT.cpp
    src/sensor/TemperatureSensorRT.cpp
    src/sensor/PressureSensorRT.cpp
    src/motor/ActuatorRT.cpp
)
add_library(SimA::SimA ALIAS SimA)

target_compile_features(SimA PUBLIC cxx_std_17)
set_target_properties(SimA PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(SimA PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(SimA PUBLIC mil1553::mil1553 Eigen3::Eigen)
