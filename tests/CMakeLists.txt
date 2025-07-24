# VAMP C++ Library Tests

# Only build tests if this is the main project or tests are explicitly enabled
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR VAMP_BUILD_TESTS)
    # Create test executable for basic functionality
    add_executable(vamp_tests
        test_basic_functionality.cpp
        test_robot_interfaces.cpp
    )

    # Link against VAMP library
    target_link_libraries(vamp_tests PRIVATE vamp_cpp)

    # Add test discovery (if using CTest)
    enable_testing()
    add_test(NAME vamp_basic_tests COMMAND vamp_tests)

    # Set test properties
    set_tests_properties(vamp_basic_tests PROPERTIES
        TIMEOUT 60
        LABELS "unit;basic"
    )
    
    # Disable strict warnings for tests to focus on functionality
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(vamp_tests PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
    endif()

    message(STATUS "VAMP C++ tests configured")
endif() 