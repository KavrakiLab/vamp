if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "EmscriptenBuild.cmake requires Emscripten toolchain.")
     endif()

message(STATUS "Configuring VAMP for WebAssembly build")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msimd128")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msimd128")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -flto")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -flto")

set(VAMP_DISABLE_SIMDXORSHIFT ON CACHE BOOL "Disable SIMDxorshift for WASM build" FORCE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")

set(VAMP_WASM_OUTPUT_NAME "vamp" CACHE STRING "Output name for WASM module")

set(VAMP_WASM_LINK_FLAGS
    "-lembind"
    "-fexceptions"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sINITIAL_MEMORY=64MB"
    "-sMAXIMUM_MEMORY=512MB"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sENVIRONMENT=web"
    "-sWASM_BIGINT=1"
)
string(JOIN " " VAMP_WASM_LINK_FLAGS_STR ${VAMP_WASM_LINK_FLAGS})

function(vamp_create_wasm_module TARGET_NAME)
    add_executable(${TARGET_NAME} ${ARGN})

    target_link_libraries(${TARGET_NAME} PRIVATE vamp_cpp)

    set_target_properties(${TARGET_NAME} PROPERTIES
        LINK_FLAGS "${VAMP_WASM_LINK_FLAGS_STR}"
        SUFFIX ".js"
    )

    set_target_properties(${TARGET_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wasm"
    )

    message(STATUS "WASM target '${TARGET_NAME}' configured")
endfunction()
