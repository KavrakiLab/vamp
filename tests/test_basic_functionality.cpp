#include <iostream>
#include <cassert>
#include <array>

// VAMP includes - comprehensive set based on working examples
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vamp/vector.hh>
#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include <vamp/robots/sphere.hh>
#include <vamp/robots/panda.hh>

// Simple test framework
#define TEST(name) \
    void test_##name(); \
    struct TestRegistrar_##name { \
        TestRegistrar_##name() { register_test(#name, test_##name); } \
    } registrar_##name; \
    void test_##name()

// Test registry
#include <vector>
#include <functional>
std::vector<std::pair<std::string, std::function<void()>>> tests;

void register_test(const std::string& name, std::function<void()> test) {
    tests.emplace_back(name, test);
}

TEST(sphere_robot_basic) {
    using Robot = vamp::robots::Sphere;
    
    // Test basic properties
    static_assert(Robot::dimension == 3, "Sphere robot should be 3D");
    
    // Test configuration creation and basic operations
    Robot::Configuration config;
    config.fill(0.0);
    
    // Test element access (which we know works from the demos)
    config[0] = 1.0;
    assert(config[0] == 1.0);
    
    std::cout << "✅ Sphere robot basic test passed" << std::endl;
}

TEST(panda_robot_basic) {
    using Robot = vamp::robots::Panda;
    
    // Test basic properties  
    static_assert(Robot::dimension == 7, "Panda robot should be 7-DOF");
    
    // Test configuration creation and basic operations
    Robot::Configuration config;
    config.fill(0.0);
    
    // Test element access
    config[0] = 1.0;
    assert(config[0] == 1.0);
    
    std::cout << "✅ Panda robot basic test passed" << std::endl;
}

TEST(configuration_manipulation) {
    using Robot = vamp::robots::Sphere;
    
    // Test configuration manipulation
    Robot::Configuration config;
    
    // Test filling with values
    config.fill(1.0);
    
    // Test individual element access using compile-time known dimension
    config[0] = 2.0;
    config[1] = 3.0; 
    config[2] = 4.0;
    
    assert(config[0] == 2.0);
    assert(config[1] == 3.0);
    assert(config[2] == 4.0);
    
    std::cout << "✅ Configuration manipulation test passed" << std::endl;
}

TEST(robot_constants) {
    // Test that robot constants are accessible
    using SphereRobot = vamp::robots::Sphere;
    using PandaRobot = vamp::robots::Panda;
    
    // Basic dimension checks using static assertions (compile-time)
    static_assert(SphereRobot::dimension == 3);
    static_assert(PandaRobot::dimension == 7);
    
    // Test that dimensions are accessible at runtime too
    assert(SphereRobot::dimension == 3);
    assert(PandaRobot::dimension == 7);
    
    std::cout << "✅ Robot constants test passed" << std::endl;
}

TEST(basic_array_operations) {
    // Test basic standard array operations (separate from VAMP types)
    std::array<float, 3> pos = {1.0f, 2.0f, 3.0f};
    std::array<float, 7> joints = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    
    // Basic array checks to ensure basic C++ functionality works
    assert(pos.size() == 3);
    assert(joints.size() == 7);
    assert(pos[0] == 1.0f);
    assert(joints[6] == 6.0f);
    
    std::cout << "✅ Basic array operations test passed" << std::endl;
}

TEST(configuration_fill_and_access) {
    // Test all robots with their known dimensions
    {
        vamp::robots::Sphere::Configuration config;
        config.fill(5.0);
        
        // Test that all elements are set (using compile-time dimension)
        for (unsigned i = 0; i < vamp::robots::Sphere::dimension; ++i) {
            assert(config[i] == 5.0);
        }
    }
    
    {
        vamp::robots::Panda::Configuration config;  
        config.fill(7.0);
        
        // Test first and last elements
        assert(config[0] == 7.0);
        assert(config[vamp::robots::Panda::dimension - 1] == 7.0);
    }
    
    std::cout << "✅ Configuration fill and access test passed" << std::endl;
}

int main() {
    std::cout << "Running VAMP C++ Library Basic Tests..." << std::endl;
    std::cout << "========================================" << std::endl;
    
    int passed = 0;
    int failed = 0;
    
    for (const auto& [name, test] : tests) {
        try {
            std::cout << "Running test: " << name << std::endl;
            test();
            passed++;
        } catch (const std::exception& e) {
            std::cerr << "❌ Test " << name << " failed: " << e.what() << std::endl;
            failed++;
        } catch (...) {
            std::cerr << "❌ Test " << name << " failed with unknown exception" << std::endl;
            failed++;
        }
    }
    
    std::cout << "========================================" << std::endl;
    std::cout << "Test Results: " << passed << " passed, " << failed << " failed" << std::endl;
    
    if (failed == 0) {
        std::cout << "🎉 All tests passed!" << std::endl;
        return 0;
    } else {
        std::cerr << "💥 Some tests failed!" << std::endl;
        return 1;
    }
} 