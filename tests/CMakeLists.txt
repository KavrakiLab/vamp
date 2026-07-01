include(Catch)

add_executable(vamp_tests
    test_collision_math.cc
    test_vector.cc
    test_halton.cc
)

target_link_libraries(vamp_tests PRIVATE vamp_cpp Catch2::Catch2WithMain)

catch_discover_tests(vamp_tests)
