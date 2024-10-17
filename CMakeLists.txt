cmake_minimum_required(VERSION 3.16...3.22)

option(VAMP_LTO "Use LTO." ON)
option(VAMP_FORCE_COLORED_OUTPUT "Always produce ANSI-colored output." ON)
option(VAMP_FORCE_CLANG "Force the use of Clang." OFF)

option(VAMP_BUILD_OMPL_DEMO "Build VAMP C++ OMPL Integration Demo Script" OFF)
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

list(APPEND VAMP_INCLUDES
  src/impl
)

# Header-only C++ library
include(GNUInstallDirs)
add_library(vamp INTERFACE)
target_include_directories(vamp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/impl>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(vamp INTERFACE cxx_std_17)
target_link_libraries(vamp INTERFACE Eigen3::Eigen nigh::nigh pdqsort::pdqsort)
install(TARGETS vamp
  EXPORT ${PROJECT_NAME}_Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file("vampConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file("${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(EXPORT ${PROJECT_NAME}_Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/vamp DESTINATION include)

# Python extension
option(VAMP_BUILD_PYTHON_BINDINGS "Build Python VAMP extensions" OFF)
if(VAMP_BUILD_PYTHON_BINDINGS)
  find_package(Python 3.8
  REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)

  CPMAddPackage("gh:wjakob/nanobind#358d452c314dbe8c07026d984ad8d5aa860f26fb")

  list(APPEND VAMP_EXT_SOURCES
    src/impl/vamp/bindings/python.cc
    src/impl/vamp/bindings/environment.cc
    src/impl/vamp/bindings/settings.cc
  )

  list(APPEND VAMP_ROBOT_MODULES
    sphere
    ur5
    panda
    fetch
    baxter
  )

  foreach(robot_name IN LISTS VAMP_ROBOT_MODULES)
    list(APPEND VAMP_EXT_SOURCES "src/impl/vamp/bindings/${robot_name}.cc")
  endforeach()

  nanobind_add_module(_core_ext
      NB_STATIC
      STABLE_ABI
      NOMINSIZE
      ${VAMP_EXT_SOURCES}
  )

  target_include_directories(_core_ext
      SYSTEM PRIVATE
      ${VAMP_INCLUDES}
  )

  target_link_libraries(_core_ext
      PRIVATE
      Eigen3::Eigen
  )

  if($ENV{GITHUB_ACTIONS})
    set(STUB_PREFIX "")
  else()
    set(STUB_PREFIX "${CMAKE_BINARY_DIR}/stubs/")
  endif()

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

if(VAMP_BUILD_OMPL_DEMO)
  find_package(ompl QUIET PATHS ${VAMP_OMPL_PATH})

  if(ompl_FOUND)
    add_executable(vamp_ompl_integration scripts/cpp/ompl_integration.cc)
    target_link_libraries(vamp_ompl_integration PRIVATE ompl::ompl Eigen3::Eigen)
    target_include_directories(vamp_ompl_integration SYSTEM PRIVATE ${VAMP_INCLUDES})
  else()
    message(WARNING "OMPL not found! Cannot build OMPL demo.")
  endif()
endif()
