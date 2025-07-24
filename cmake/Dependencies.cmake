find_package(Eigen3 REQUIRED NO_MODULE)

CPMAddPackage("gh:kavrakilab/nigh#97130999440647c204e0265d05a997dbd8da4e70")
add_library(nigh INTERFACE)
target_include_directories(nigh INTERFACE $<BUILD_INTERFACE:${nigh_SOURCE_DIR}/src>)
if(VAMP_INSTALL_CPP_LIBRARY)
  install(TARGETS nigh
        EXPORT nigh_TARGETS
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(EXPORT nigh_TARGETS FILE nighTargets.cmake NAMESPACE nigh:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nigh)
  export(EXPORT nigh_TARGETS FILE ${CMAKE_CURRENT_BINARY_DIR}/cmake/nighTargets.cmake NAMESPACE nigh::)
endif()

CPMAddPackage("gh:orlp/pdqsort#b1ef26a55cdb60d236a5cb199c4234c704f46726")
add_library(pdqsort INTERFACE)
target_include_directories(pdqsort INTERFACE $<BUILD_INTERFACE:${pdqsort_SOURCE_DIR}>)
if(VAMP_INSTALL_CPP_LIBRARY)
  install(TARGETS pdqsort
        EXPORT pdqsort_TARGETS
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(EXPORT pdqsort_TARGETS FILE pdqsortTargets.cmake NAMESPACE pdqsort:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pdqsort)
  export(EXPORT pdqsort_TARGETS FILE ${CMAKE_CURRENT_BINARY_DIR}/cmake/pdqsortTargets.cmake NAMESPACE pdqsort::)
endif()

# SIMDxorshift for x86_64 systems (includes macOS Intel and Linux x86_64)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
  CPMAddPackage("gh:lemire/SIMDxorshift#857c1a01df53cf1ee1ae8db3238f0ef42ef8e490")
  
  if(SIMDxorshift_SOURCE_DIR)
    add_library(simdxorshift STATIC
      ${SIMDxorshift_SOURCE_DIR}/src/simdxorshift128plus.c
      ${SIMDxorshift_SOURCE_DIR}/src/xorshift128plus.c
    )
    
    target_include_directories(simdxorshift
      PRIVATE
      ${SIMDxorshift_SOURCE_DIR}/include
      INTERFACE
      $<BUILD_INTERFACE:${SIMDxorshift_SOURCE_DIR}/include>
    )
    
    # Set library properties for simdxorshift
    set_target_properties(simdxorshift PROPERTIES
      VERSION ${PROJECT_VERSION}
      SOVERSION ${PROJECT_VERSION_MAJOR}
    )
    
    # SIMDxorshift target created successfully
  endif()
endif()