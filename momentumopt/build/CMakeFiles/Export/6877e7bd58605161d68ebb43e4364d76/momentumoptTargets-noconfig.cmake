#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "momentumopt::momentumopt" for configuration ""
set_property(TARGET momentumopt::momentumopt APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(momentumopt::momentumopt PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmomentumopt.so"
  IMPORTED_SONAME_NOCONFIG "libmomentumopt.so"
  )

list(APPEND _cmake_import_check_targets momentumopt::momentumopt )
list(APPEND _cmake_import_check_files_for_momentumopt::momentumopt "${_IMPORT_PREFIX}/lib/libmomentumopt.so" )

# Import target "momentumopt::demo_momentumopt" for configuration ""
set_property(TARGET momentumopt::demo_momentumopt APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(momentumopt::demo_momentumopt PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/demo_momentumopt"
  )

list(APPEND _cmake_import_check_targets momentumopt::demo_momentumopt )
list(APPEND _cmake_import_check_files_for_momentumopt::demo_momentumopt "${_IMPORT_PREFIX}/bin/demo_momentumopt" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
