cmake_minimum_required(VERSION 3.16...3.22)

option(VAMP_LTO "Use LTO." ON)
option(VAMP_FORCE_COLORED_OUTPUT "Always produce ANSI-colored output." ON)
option(VAMP_FORCE_CLANG "Force the use of Clang." OFF)

option(VAMP_BUILD_PYTHON_BINDINGS "Build VAMP Python bindings" ON)
option(VAMP_INSTALL_CPP_LIBRARY "Install VAMP C++ library (disable for Python wheel builds)" ON)

option(VAMP_BUILD_CPP_DEMO "Build VAMP C++ Demo Scripts" OFF)
option(VAMP_BUILD_OMPL_DEMO "Build VAMP C++ OMPL Integration Demo Scripts" OFF)
option(VAMP_OMPL_PATH "Search Path for OMPL Installation - Only Needed for Demo Script" "")
option(VAMP_BUILD_WASM "Build VAMP WebAssembly module" OFF)
option(VAMP_BUILD_JIT "Build the vamp::jit runtime" OFF)

if(VAMP_FORCE_CLANG)
  find_program(CLANG "clang")
  find_program(CLANGPP "clang++")

  set(CMAKE_C_COMPILER ${CLANG} CACHE STRING "C compiler" FORCE)
  set(CMAKE_CXX_COMPILER ${CLANGPP} CACHE STRING "C++ compiler" FORCE)
endif()

if(VAMP_FORCE_COLORED_OUTPUT)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics)
  endif()
endif()

project(
    vamp
    VERSION 0.2
    LANGUAGES C CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(macros)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CompilerSettings)
include(FetchInitCPM)
include(Dependencies)

# Create VAMP C++ library
add_library(vamp_cpp INTERFACE)

# Set required C++ standard for the library
target_compile_features(vamp_cpp INTERFACE cxx_std_17)

target_include_directories(vamp_cpp
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/impl>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/vamp>
)

target_link_libraries_system(vamp_cpp
    INTERFACE
    Eigen3::Eigen
    nigh
    pdqsort
)

# Link SIMDxorshift if available
if(TARGET simdxorshift)
  target_link_libraries(vamp_cpp INTERFACE simdxorshift)
endif()

# Set library properties
set_target_properties(vamp_cpp PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Create alias target for modern CMake usage
add_library(vamp::vamp ALIAS vamp_cpp)

# vamp::jit — runtime that JIT-compiles a stub per robot/planner via cricket::jit
if(VAMP_BUILD_JIT)
  find_package(cricket REQUIRED)

  # Configure build_paths.hh with the include dirs / flags vamp's JIT stub needs to compile against.
  set(VAMP_SRC_IMPL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/impl")
  set(VAMP_SIMDXORSHIFT_INCLUDE "${SIMDxorshift_SOURCE_DIR}/include")
  get_target_property(VAMP_CRICKET_INCLUDE_DIR cricket::cricket INTERFACE_INCLUDE_DIRECTORIES)
  list(GET VAMP_CRICKET_INCLUDE_DIR 0 VAMP_CRICKET_INCLUDE_DIR)
  get_target_property(VAMP_EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  list(GET VAMP_EIGEN3_INCLUDE_DIR 0 VAMP_EIGEN3_INCLUDE_DIR)
  set(VAMP_NIGH_INCLUDE_DIR "${nigh_SOURCE_DIR}/src")
  set(VAMP_PDQSORT_INCLUDE_DIR "${pdqsort_SOURCE_DIR}")
  configure_file(
      src/impl/vamp/jit/build_paths.hh.in
      ${CMAKE_CURRENT_BINARY_DIR}/generated/vamp/jit/build_paths.hh
      @ONLY
  )

  # Embed the JIT stub templates (preamble + per-planner extern "C" wrappers)
  # at configure time, mirroring cricket's embedded_templates pattern. Lets the
  # stub C++ live in proper templates/*.cc files instead of giant raw-string
  # literals inside stub_gen.cc.
  # filename → @VAR@ placeholder in embedded_stubs.hh.in
  foreach(stub IN ITEMS
      preamble:VAMP_JIT_PREAMBLE
      sampler_stub:VAMP_JIT_SAMPLER_STUB
      simplify_stub:VAMP_JIT_SIMPLIFY_STUB
      debug_stub:VAMP_JIT_DEBUG_STUB
      planner_stub:VAMP_JIT_PLANNER_STUB
  )
    string(REPLACE ":" ";" stub "${stub}")
    list(GET stub 0 stub_file)
    list(GET stub 1 stub_var)
    file(READ src/impl/vamp/jit/templates/${stub_file}.cc ${stub_var})
  endforeach()
  configure_file(
      src/impl/vamp/jit/embedded_stubs.hh.in
      ${CMAKE_CURRENT_BINARY_DIR}/generated/vamp/jit/embedded_stubs.hh
      @ONLY
  )

  add_library(vamp_jit
      src/impl/vamp/jit/stub_gen.cc
      src/impl/vamp/jit/dynamic_robot.cc
  )
  add_library(vamp::jit ALIAS vamp_jit)
  set_target_properties(vamp_jit PROPERTIES POSITION_INDEPENDENT_CODE ON)

  target_include_directories(vamp_jit
      PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/impl>
          $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
          $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/vamp>
  )

  target_link_libraries(vamp_jit
      PUBLIC
          vamp::vamp
          cricket::cricket_jit
      PRIVATE
          cricket::cricket_inja   # vendored single-header inja for stub templating
  )
endif()

# Build Python bindings
include(Python)

# Install C++ library components (conditional for Python wheel builds)
if(VAMP_INSTALL_CPP_LIBRARY)
  # Install C++ library headers and SIMD library (if available)
  set(VAMP_INSTALL_TARGETS vamp_cpp)
  if(TARGET simdxorshift)
    list(APPEND VAMP_INSTALL_TARGETS simdxorshift)
  endif()

  install(TARGETS ${VAMP_INSTALL_TARGETS}
      EXPORT vamp_cppTargets
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  # Install headers
  install(DIRECTORY src/impl/vamp/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/vamp
      FILES_MATCHING PATTERN "*.hh"
  )

  # Generate package configuration files
  configure_package_config_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/vampConfig.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/vampConfig.cmake"
      INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/vamp
  )

  write_basic_package_version_file(
      "${CMAKE_CURRENT_BINARY_DIR}/vampConfigVersion.cmake"
      VERSION ${PROJECT_VERSION}
      COMPATIBILITY SameMajorVersion
  )

  # Install package configuration files
  install(FILES
      "${CMAKE_CURRENT_BINARY_DIR}/vampConfig.cmake"
      "${CMAKE_CURRENT_BINARY_DIR}/vampConfigVersion.cmake"
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/vamp
  )

  # Export targets
  install(EXPORT vamp_cppTargets
      FILE vamp_cppTargets.cmake
      NAMESPACE vamp::
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/vamp
  )
endif()

# C++ demos
if(VAMP_BUILD_CPP_DEMO)
  add_executable(vamp_rrtc_example scripts/cpp/rrtc_example.cc)
  target_link_libraries(vamp_rrtc_example PRIVATE vamp_cpp)
  
  # Disable strict warnings for demos to maintain compatibility
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(vamp_rrtc_example PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
  endif()
endif()

# OMPL integration demo
if(VAMP_BUILD_OMPL_DEMO)
  find_package(ompl QUIET PATHS ${VAMP_OMPL_PATH})

  if(ompl_FOUND)
    add_executable(vamp_ompl_integration scripts/cpp/ompl_integration.cc)
    target_link_libraries(vamp_ompl_integration PRIVATE ompl::ompl vamp_cpp)
    
    # Disable strict warnings for demos to maintain compatibility
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(vamp_ompl_integration PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
    endif()
  else()
    message(WARNING "OMPL not found! Cannot build OMPL demo.")
  endif()
endif()

# WebAssembly build
if(VAMP_BUILD_WASM)
  include(EmscriptenBuild)

  if(NOT VAMP_WASM_ROBOT_MODULES)
    set(VAMP_WASM_ROBOT_MODULES panda fetch ur5)
    set(VAMP_WASM_ROBOT_STRUCTS Panda Fetch UR5)
  endif()

  foreach(robot_name ROBOT_STRUCT IN ZIP_LISTS VAMP_WASM_ROBOT_MODULES VAMP_WASM_ROBOT_STRUCTS)
    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/src/impl/vamp/bindings/wasm/bindings.cc.in
      ${CMAKE_CURRENT_BINARY_DIR}/wasm/${robot_name}_wasm_bindings.cc
      @ONLY
    )

    vamp_create_wasm_module(${robot_name}_wasm
      ${CMAKE_CURRENT_BINARY_DIR}/wasm/${robot_name}_wasm_bindings.cc
    )

    target_include_directories(${robot_name}_wasm PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/impl
    )
  endforeach()
endif()

# Print build configuration
message(STATUS "VAMP build configuration:")
message(STATUS "  - C++ library: ON")
message(STATUS "  - Install C++ library: ${VAMP_INSTALL_CPP_LIBRARY}")
message(STATUS "  - Python bindings: ${VAMP_BUILD_PYTHON_BINDINGS}")
message(STATUS "  - JIT: ${VAMP_BUILD_JIT}")
message(STATUS "  - C++ demos: ${VAMP_BUILD_CPP_DEMO}")
message(STATUS "  - OMPL demos: ${VAMP_BUILD_OMPL_DEMO}")
message(STATUS "  - WASM module: ${VAMP_BUILD_WASM}")
