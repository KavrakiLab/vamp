if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	# Need explicit AVX2 for some MacOS clang versions
	set(VAMP_ARCH "-march=native -mavx2")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
	# ARM platforms (aarch64 / arm64)
	set(VAMP_ARCH "-mcpu=native -mtune=native")
else()
	message(FATAL_ERROR "Unsupported architecture ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# default fast args that work on all platforms
set(VAMP_FAST_ARGS "-fno-math-errno -fno-signed-zeros -fno-trapping-math -fno-rounding-math -ffp-contract=fast")

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64") # x86 supports additional flags
	string(APPEND VAMP_FAST_ARGS " -fassociative-math")
	if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang") # Clang supports additional fine-grained flags over GCC
		string(APPEND VAMP_FAST_ARGS " -fno-honor-infinities -fno-honor-nans")
		if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 11.0.0) # clang 11 does not support -fapprox-func
			string(APPEND VAMP_FAST_ARGS " -fapprox-func")
		endif()
	endif()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VAMP_ARCH} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 ${VAMP_FAST_ARGS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${VAMP_ARCH} -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -g")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 ${VAMP_FAST_ARGS}")

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	# Valgrind can't handle avx512 instructions
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -mno-avx512f")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -mno-avx512f")
endif()

if(VAMP_LTO)
	if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
		set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -flto")
	elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		# specify auto paralellism for gcc linking
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto=auto")
		set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -flto=auto")
	endif()
endif()

