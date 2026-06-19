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