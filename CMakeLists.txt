cmake_minimum_required(VERSION 3.20)
project(mil1553 LANGUAGES CXX)

add_subdirectory(1553 mil1553)
add_subdirectory(SimA)

option(MIL1553_BUILD_EXAMPLE "Build the 1553 simulation example" ON)
if(MIL1553_BUILD_EXAMPLE)
    find_package(Threads REQUIRED)
    add_executable(1553 main.cpp)
    target_link_libraries(1553 PRIVATE SimA::SimA Threads::Threads)
    set_target_properties(1553 PROPERTIES CXX_EXTENSIONS OFF)
endif()

include(CTest)
if(BUILD_TESTING)
    find_package(GTest CONFIG QUIET)
    if(NOT GTest_FOUND)
        include(FetchContent)
        # Match the parent project's runtime when building with MSVC.
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG b514bdc898e2951020cbdca1304b75f5950d1f59 # v1.15.2
        )
        FetchContent_MakeAvailable(googletest)
    endif()

    add_executable(mil1553_tests
        1553/test/test_bc.cpp
        1553/test/test_bm.cpp
        1553/test/test_hal.cpp
        1553/test/test_rt.cpp
        1553/test/test_sim_time.cpp
        1553/test/test_types.cpp
        SimA/test/test_actuator.cpp
        SimA/test/test_flight_computer.cpp
        SimA/test/test_imu.cpp
        SimA/test/test_sensors.cpp
        SimA/test/test_vehicle.cpp
    )
    target_link_libraries(mil1553_tests PRIVATE SimA::SimA GTest::gtest_main)
    set_target_properties(mil1553_tests PROPERTIES
        CXX_EXTENSIONS OFF
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "${_IMPORT_PREFIX}/lib;/opt/anaconda3/lib"
        BUILD_RPATH "/opt/anaconda3/lib"
    )

    include(GoogleTest)
    gtest_discover_tests(mil1553_tests)
endif()
