cmake_minimum_required(VERSION 3.16...3.22)

option(VAMP_LTO "Use LTO." ON)
option(VAMP_FORCE_COLORED_OUTPUT "Always produce ANSI-colored output." ON)
option(VAMP_FORCE_CLANG "Force the use of Clang." OFF)
option(VAMP_BUILD_PYTHON "Force the use of Clang." ON)

option(VAMP_BUILD_PYTHON "Build Python module" ON)
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

find_package(Eigen3 REQUIRED NO_MODULE)

CPMAddPackage("gh:kavrakilab/nigh#97130999440647c204e0265d05a997dbd8da4e70")
set(NIGH_INCLUDE_DIRS ${nigh_SOURCE_DIR}/src)

CPMAddPackage("gh:orlp/pdqsort#b1ef26a55cdb60d236a5cb199c4234c704f46726")
set(PDQSORT_INCLUDE_DIRS ${pdqsort_SOURCE_DIR})

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  CPMAddPackage("gh:lemire/SIMDxorshift#857c1a01df53cf1ee1ae8db3238f0ef42ef8e490")
  list(APPEND VAMP_SOURCES
    ${SIMDxorshift_SOURCE_DIR}/src/simdxorshift128plus.c
    ${SIMDxorshift_SOURCE_DIR}/src/xorshift128plus.c)
  list(APPEND VAMP_INCLUDES ${SIMDxorshift_SOURCE_DIR}/include)
endif()

list(APPEND VAMP_INCLUDES
  ${NIGH_INCLUDE_DIRS}
  ${PDQSORT_INCLUDE_DIRS}
  src/impl
)

if(VAMP_BUILD_PYTHON)
  find_package(Python 3.8
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)

  CPMAddPackage("gh:wjakob/nanobind#358d452c314dbe8c07026d984ad8d5aa860f26fb")

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
    ${VAMP_SOURCES}
  )

  target_include_directories(_core_ext
    SYSTEM PRIVATE
    ${VAMP_INCLUDES}
    ${CMAKE_CURRENT_BINARY_DIR}
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

if(VAMP_BUILD_CPP_DEMO)
  add_executable(vamp_rrtc_example scripts/cpp/rrtc_example.cc ${VAMP_SOURCES})
  target_link_libraries(vamp_rrtc_example PRIVATE Eigen3::Eigen)
  target_include_directories(vamp_rrtc_example SYSTEM PRIVATE ${VAMP_INCLUDES})
endif()

if(VAMP_BUILD_OMPL_DEMO)
  find_package(ompl QUIET PATHS ${VAMP_OMPL_PATH})

  if(ompl_FOUND)
    add_executable(vamp_ompl_integration scripts/cpp/ompl_integration.cc ${VAMP_SOURCES})
    target_link_libraries(vamp_ompl_integration PRIVATE ompl::ompl Eigen3::Eigen)
    target_include_directories(vamp_ompl_integration SYSTEM PRIVATE ${VAMP_INCLUDES})
  else()
    message(WARNING "OMPL not found! Cannot build OMPL demo.")
  endif()
endif()
