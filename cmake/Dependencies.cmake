find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Python 3.8
  REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)

CPMAddPackage("gh:wjakob/nanobind#358d452c314dbe8c07026d984ad8d5aa860f26fb")

CPMAddPackage("gh:kavrakilab/nigh#97130999440647c204e0265d05a997dbd8da4e70")
set(NIGH_INCLUDE_DIRS ${nigh_SOURCE_DIR}/src)

CPMAddPackage("gh:orlp/pdqsort#b1ef26a55cdb60d236a5cb199c4234c704f46726")
set(PDQSORT_INCLUDE_DIRS ${pdqsort_SOURCE_DIR})

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  CPMAddPackage("gh:lemire/SIMDxorshift#857c1a01df53cf1ee1ae8db3238f0ef42ef8e490")
  list(APPEND VAMP_EXT_SOURCES
    ${SIMDxorshift_SOURCE_DIR}/src/simdxorshift128plus.c
    ${SIMDxorshift_SOURCE_DIR}/src/xorshift128plus.c)
  list(APPEND VAMP_EXT_INCLUDES ${SIMDxorshift_SOURCE_DIR}/include)
endif()
