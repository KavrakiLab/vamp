cmake_minimum_required(VERSION 3.16...3.22)

option(VAMP_LTO "Use LTO." ON)
option(VAMP_FORCE_COLORED_OUTPUT "Always produce ANSI-colored output." ON)
option(VAMP_FORCE_CLANG "Force the use of Clang." OFF)

option(VAMP_BUILD_PYTHON_BINDINGS "Build VAMP Python bindings" ON)

option(VAMP_BUILD_CPP_DEMO "Build VAMP C++ Demo Scripts" OFF)
option(VAMP_BUILD_OMPL_DEMO "Build VAMP C++ OMPL Integration Demo Scripts" OFF)
option(VAMP_OMPL_PATH "Search Path for OMPL Installation - Only Needed for Demo Script" "")

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
include(CompilerSettings)
include(FetchInitCPM)

include(Dependencies)

# Add SIMDxorshift dependency for x86_64 systems
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  CPMAddPackage("gh:lemire/SIMDxorshift#857c1a01df53cf1ee1ae8db3238f0ef42ef8e490")
endif()

# Create VAMP C++ library
add_library(vamp_cpp INTERFACE)

# Always use only the subdirectory for install interface to avoid CMake's parent directory bug
# See: https://gitlab.kitware.com/cmake/cmake/-/issues/17950 and related
# Do NOT use INCLUDES DESTINATION in install(TARGETS ...) for INTERFACE libraries
# If you change include structure, update this path accordingly

# Split common includes into exportable and non-exportable parts
set(VAMP_EXPORTABLE_INCLUDES
  ${NIGH_INCLUDE_DIRS}
  ${PDQSORT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/impl
)

set(VAMP_BUILD_ONLY_INCLUDES)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" AND SIMDxorshift_SOURCE_DIR)
  list(APPEND VAMP_BUILD_ONLY_INCLUDES ${SIMDxorshift_SOURCE_DIR}/include)
endif()

# Common include directories for both C++ library and Python bindings
set(VAMP_COMMON_INCLUDES ${VAMP_EXPORTABLE_INCLUDES} ${VAMP_BUILD_ONLY_INCLUDES})

message(STATUS "VAMP_COMMON_INCLUDES: ${VAMP_COMMON_INCLUDES}")

target_include_directories(vamp_cpp
    INTERFACE
        $<BUILD_INTERFACE:${VAMP_EXPORTABLE_INCLUDES}>
        $<BUILD_INTERFACE:${VAMP_BUILD_ONLY_INCLUDES}>
        $<INSTALL_INTERFACE:include/vamp>
)

target_link_libraries(vamp_cpp
    INTERFACE
    Eigen3::Eigen
    nigh
    pdqsort
)

# Create SIMDxorshift static library for x86_64 systems
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" AND SIMDxorshift_SOURCE_DIR)
  add_library(vamp_simd STATIC
    ${SIMDxorshift_SOURCE_DIR}/src/simdxorshift128plus.c
    ${SIMDxorshift_SOURCE_DIR}/src/xorshift128plus.c
  )
  
  target_include_directories(vamp_simd
    PRIVATE
    ${SIMDxorshift_SOURCE_DIR}/include
  )
  
  # Link the SIMD library to vamp_cpp
  target_link_libraries(vamp_cpp
    INTERFACE
    vamp_simd
  )
  
  # Set library properties for vamp_simd
  set_target_properties(vamp_simd PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
  )
endif()

# Set library properties
set_target_properties(vamp_cpp PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Install C++ library headers and SIMD library (if available)
set(VAMP_INSTALL_TARGETS vamp_cpp)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" AND TARGET vamp_simd)
  list(APPEND VAMP_INSTALL_TARGETS vamp_simd)
endif()

install(TARGETS ${VAMP_INSTALL_TARGETS}
    EXPORT vamp_cppTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY src/impl/vamp/
    DESTINATION include/vamp
    FILES_MATCHING PATTERN "*.hh"
)

# Export targets
install(EXPORT vamp_cppTargets
    FILE vamp_cppTargets.cmake
    NAMESPACE vamp::
    DESTINATION lib/cmake/vamp
)


if(VAMP_BUILD_PYTHON_BINDINGS)
  # Check if Python is available
  if(NOT Python_FOUND)
    message(FATAL_ERROR "VAMP_BUILD_PYTHON_BINDINGS is ON but Python was not found")
  endif()

  if(NOT VAMP_ROBOT_MODULES)
    list(APPEND VAMP_ROBOT_MODULES
      sphere
      ur5
      panda
      fetch
      baxter
    )

    list(APPEND VAMP_ROBOT_STRUCTS
      Sphere
      UR5
      Panda
      Fetch
      Baxter
    )
  endif()

  foreach(robot ${VAMP_ROBOT_MODULES})
    string(APPEND VAMP_ROBOT_INITS "    vb::init_${robot}(pymodule);\n")
    string(APPEND VAMP_ROBOT_DECLS "    void init_${robot}(nanobind::module_ &pymodule);\n")
    string(APPEND VAMP_ROBOT_QUOTED "\"${robot}\",")
  endforeach()

  list(JOIN VAMP_ROBOT_QUOTED ", " VAMP_ROBOT_NAMES)

  configure_file(
    src/impl/vamp/bindings/init.hh.in
    ${CMAKE_CURRENT_BINARY_DIR}/vamp_python_init.hh
    @ONLY
  )

  configure_file(
    src/impl/vamp/bindings/python.cc.in
    ${CMAKE_CURRENT_BINARY_DIR}/python.cc
    @ONLY
  )

  list(APPEND VAMP_EXT_SOURCES
    src/impl/vamp/bindings/environment.cc
    src/impl/vamp/bindings/settings.cc
    ${CMAKE_CURRENT_BINARY_DIR}/python.cc
  )
  foreach(robot_name robot_struct IN ZIP_LISTS VAMP_ROBOT_MODULES VAMP_ROBOT_STRUCTS)
  configure_file(
    src/impl/vamp/bindings/robot.cc.in
    ${CMAKE_CURRENT_BINARY_DIR}/${robot_name}.cc
    @ONLY
  )

  list(APPEND VAMP_EXT_SOURCES
    ${CMAKE_CURRENT_BINARY_DIR}/${robot_name}.cc
  )
  endforeach()

  nanobind_add_module(_core_ext
    NB_STATIC
    STABLE_ABI
    NOMINSIZE
    ${VAMP_EXT_SOURCES}
  )

  target_include_directories(_core_ext
    SYSTEM PRIVATE
    ${VAMP_COMMON_INCLUDES}
    ${CMAKE_CURRENT_BINARY_DIR}
  )

  target_link_libraries(_core_ext
    PRIVATE
    vamp_cpp
    Eigen3::Eigen
  )

  nanobind_add_stub(
    vamp_stub
    MODULE _core_ext
    OUTPUT "${STUB_PREFIX}__init__.pyi"
    PYTHON_PATH $<TARGET_FILE_DIR:_core_ext>
    DEPENDS _core_ext
    VERBOSE
  )

  foreach(robot_name IN LISTS VAMP_ROBOT_MODULES)
    nanobind_add_stub(
      "vamp_${robot_name}_stub"
      MODULE "_core_ext.${robot_name}"
      OUTPUT "${STUB_PREFIX}${robot_name}.pyi"
      PYTHON_PATH $<TARGET_FILE_DIR:_core_ext>
      DEPENDS _core_ext
      VERBOSE
    )
  endforeach()

  install(
    TARGETS _core_ext
    LIBRARY
    DESTINATION vamp/_core
  )

  install(
    FILES "${STUB_PREFIX}__init__.pyi"
    DESTINATION "${CMAKE_SOURCE_DIR}/src/vamp/_core"
  )

  foreach(robot_name IN LISTS VAMP_ROBOT_MODULES)
    install(
      FILES "${STUB_PREFIX}${robot_name}.pyi"
      DESTINATION "${CMAKE_SOURCE_DIR}/src/vamp/_core"
    )
  endforeach()
endif()

# C++ demos
if(VAMP_BUILD_CPP_DEMO)
  add_executable(vamp_rrtc_example scripts/cpp/rrtc_example.cc)
  target_link_libraries(vamp_rrtc_example PRIVATE vamp_cpp)
endif()

# OMPL integration demo
if(VAMP_BUILD_OMPL_DEMO)
  find_package(ompl QUIET PATHS ${VAMP_OMPL_PATH})

  if(ompl_FOUND)
    add_executable(vamp_ompl_integration scripts/cpp/ompl_integration.cc)
    target_link_libraries(vamp_ompl_integration PRIVATE ompl::ompl vamp_cpp)
  else()
    message(WARNING "OMPL not found! Cannot build OMPL demo.")
  endif()
endif()

# Print build configuration
message(STATUS "VAMP build configuration:")
message(STATUS "  - C++ library: ON")
message(STATUS "  - Python bindings: ${VAMP_BUILD_PYTHON_BINDINGS}")
message(STATUS "  - C++ demos: ${VAMP_BUILD_CPP_DEMO}")
message(STATUS "  - OMPL demos: ${VAMP_BUILD_OMPL_DEMO}")