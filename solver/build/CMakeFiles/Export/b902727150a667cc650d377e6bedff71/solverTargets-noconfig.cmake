#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "solver::solver" for configuration ""
set_property(TARGET solver::solver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(solver::solver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsolver.so"
  IMPORTED_SONAME_NOCONFIG "libsolver.so"
  )

list(APPEND _cmake_import_check_targets solver::solver )
list(APPEND _cmake_import_check_files_for_solver::solver "${_IMPORT_PREFIX}/lib/libsolver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
