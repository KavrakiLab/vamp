find_package(Eigen3 REQUIRED NO_MODULE)

if(VAMP_BUILD_PYTHON_BINDINGS)
  find_package(Python 3.8
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)

  CPMAddPackage("gh:wjakob/nanobind#358d452c314dbe8c07026d984ad8d5aa860f26fb")
endif()

CPMAddPackage("gh:kavrakilab/nigh#97130999440647c204e0265d05a997dbd8da4e70")
if(nigh_ADDED)
  add_library(nigh INTERFACE)
  target_include_directories(nigh INTERFACE $<BUILD_INTERFACE:${nigh_SOURCE_DIR}/src>)
  set(NIGH_INCLUDE_DIRS ${nigh_SOURCE_DIR}/src)
  install(TARGETS nigh
        EXPORT nigh_TARGETS
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(EXPORT nigh_TARGETS FILE nighTargets.cmake NAMESPACE nigh:: DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/nigh)
  export(EXPORT nigh_TARGETS FILE ${CMAKE_CURRENT_BINARY_DIR}/cmake/nighTargets.cmake NAMESPACE nigh::)
else()
  message(FATAL_ERROR "Could not install required dependency: nigh")
endif()

CPMAddPackage("gh:orlp/pdqsort#b1ef26a55cdb60d236a5cb199c4234c704f46726")
if(pdqsort_ADDED)
  add_library(pdqsort INTERFACE)
  target_include_directories(pdqsort INTERFACE $<BUILD_INTERFACE:${pdqsort_SOURCE_DIR}>)
  set(PDQSORT_INCLUDE_DIRS ${pdqsort_SOURCE_DIR})
  install(TARGETS pdqsort
        EXPORT pdqsort_TARGETS
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(EXPORT pdqsort_TARGETS FILE pdqsortTargets.cmake NAMESPACE pdqsort:: DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/pdqsort)
  export(EXPORT pdqsort_TARGETS FILE ${CMAKE_CURRENT_BINARY_DIR}/cmake/pdqsortTargets.cmake NAMESPACE pdqsort::)
else()
  message(FATAL_ERROR "Could not install required dependency: pdqsort")
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  CPMAddPackage("gh:lemire/SIMDxorshift#857c1a01df53cf1ee1ae8db3238f0ef42ef8e490")
endif()