#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "solver_lqr::solver_lqr" for configuration ""
set_property(TARGET solver_lqr::solver_lqr APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(solver_lqr::solver_lqr PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsolver_lqr.so"
  IMPORTED_SONAME_NOCONFIG "libsolver_lqr.so"
  )

list(APPEND _cmake_import_check_targets solver_lqr::solver_lqr )
list(APPEND _cmake_import_check_files_for_solver_lqr::solver_lqr "${_IMPORT_PREFIX}/lib/libsolver_lqr.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
