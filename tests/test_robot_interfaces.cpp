#include <iostream>
#include <cassert>

// VAMP includes - comprehensive set based on working examples 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vamp/vector.hh>
#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include <vamp/robots/sphere.hh>
#include <vamp/robots/panda.hh>

// External test registry (from test_basic_functionality.cpp)
extern std::vector<std::pair<std::string, std::function<void()>>> tests;
extern void register_test(const std::string& name, std::function<void()> test);

// Use same TEST macro as other file
#define TEST(name) \
    void test_##name(); \
    struct TestRegistrar_##name { \
        TestRegistrar_##name() { register_test(#name, test_##name); } \
    } registrar_##name; \
    void test_##name()

// Test basic robot interface properties
template<typename Robot>
void test_robot_basic_interface() {
    // Test that dimension is accessible and reasonable
    constexpr auto dim = Robot::dimension;
    static_assert(dim > 0, "Robot dimension must be positive");
    static_assert(dim <= 20, "Robot dimension seems unreasonably large");
    
    // Test that Configuration type exists and can be constructed
    typename Robot::Configuration config;
    
    // Test that we can fill the configuration
    config.fill(0.0);
    
    // Test element access using known dimension
    if constexpr (dim > 0) {
        config[0] = 1.0;
        assert(config[0] == 1.0);
    }
    
    if constexpr (dim > 1) {
        config[1] = 2.0;  
        assert(config[1] == 2.0);
    }
    
    std::cout << "✅ Robot interface test passed for robot with dimension: " << dim << std::endl;
}

TEST(sphere_basic_interface) {
    test_robot_basic_interface<vamp::robots::Sphere>();
}

TEST(panda_basic_interface) {
    test_robot_basic_interface<vamp::robots::Panda>();
}

TEST(known_robot_dimensions) {
    static_assert(vamp::robots::Sphere::dimension == 3);
    static_assert(vamp::robots::Panda::dimension == 7);
    
    assert(vamp::robots::Sphere::dimension == 3);
    assert(vamp::robots::Panda::dimension == 7);
    
    std::cout << "✅ Known robot dimensions test passed" << std::endl;
}

TEST(robot_configuration_manipulation) {
    vamp::robots::Sphere::Configuration sphere_config;
    vamp::robots::Panda::Configuration panda_config;
    
    // Test fill operation
    sphere_config.fill(1.0);
    panda_config.fill(2.0);
    
    // Test element access
    sphere_config[0] = 10.0;
    panda_config[0] = 20.0;
    
    assert(sphere_config[0] == 10.0);
    assert(panda_config[0] == 20.0);
    
    std::cout << "✅ Robot configuration manipulation test passed" << std::endl;
} 