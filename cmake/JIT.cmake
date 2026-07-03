if(VAMP_BUILD_JIT)
  find_package(cricket REQUIRED)

  # Configure build_paths.hh with the include dirs / flags vamp's JIT stub needs to compile against
  set(VAMP_SRC_IMPL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/impl")
  set(VAMP_SIMDXORSHIFT_INCLUDE "${SIMDxorshift_SOURCE_DIR}/include")
  get_target_property(VAMP_CRICKET_INCLUDE_DIR cricket::cricket INTERFACE_INCLUDE_DIRECTORIES)
  list(GET VAMP_CRICKET_INCLUDE_DIR 0 VAMP_CRICKET_INCLUDE_DIR)
  get_target_property(VAMP_EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  list(GET VAMP_EIGEN3_INCLUDE_DIR 0 VAMP_EIGEN3_INCLUDE_DIR)
  set(VAMP_NIGH_INCLUDE_DIR "${nigh_SOURCE_DIR}/src")
  set(VAMP_PDQSORT_INCLUDE_DIR "${pdqsort_SOURCE_DIR}")

  # Extract defines+flags added to the compilation process
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _vamp_jit_build_type_upper)
  set(VAMP_JIT_FLAGS_STR
      "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_vamp_jit_build_type_upper}} ${VAMP_ARCH} ${VAMP_FAST_ARGS}")
  string(REGEX REPLACE "[ \t]+" ";" _vamp_jit_all_tokens "${VAMP_JIT_FLAGS_STR}")
  list(REMOVE_ITEM _vamp_jit_all_tokens "")

  set(VAMP_JIT_DEFINES "")
  set(VAMP_JIT_FLAGS "")
  foreach(tok IN LISTS _vamp_jit_all_tokens)
    if(tok MATCHES "^-D(.+)$")
      list(APPEND VAMP_JIT_DEFINES "${CMAKE_MATCH_1}")
    else()
      list(APPEND VAMP_JIT_FLAGS "${tok}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES VAMP_JIT_DEFINES)
  list(REMOVE_DUPLICATES VAMP_JIT_FLAGS)

  list(LENGTH VAMP_JIT_DEFINES VAMP_JIT_DEFINES_COUNT)
  set(VAMP_JIT_DEFINES_INIT "")
  foreach(d IN LISTS VAMP_JIT_DEFINES)
    string(APPEND VAMP_JIT_DEFINES_INIT "        \"${d}\",\n")
  endforeach()

  list(LENGTH VAMP_JIT_FLAGS VAMP_JIT_FLAGS_COUNT)
  set(VAMP_JIT_FLAGS_INIT "")
  foreach(f IN LISTS VAMP_JIT_FLAGS)
    string(APPEND VAMP_JIT_FLAGS_INIT "        \"${f}\",\n")
  endforeach()

  configure_file(
      src/impl/vamp/jit/build_paths.hh.in
      ${CMAKE_CURRENT_BINARY_DIR}/generated/vamp/jit/build_paths.hh
      @ONLY
  )

  # Embed the JIT stub templates
  foreach(stub IN ITEMS
      preamble:VAMP_JIT_PREAMBLE
      robot_stub:VAMP_JIT_ROBOT_STUB
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