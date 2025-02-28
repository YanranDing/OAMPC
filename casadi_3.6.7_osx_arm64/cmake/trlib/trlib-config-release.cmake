#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "trlib::trlib" for configuration "Release"
set_property(TARGET trlib::trlib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(trlib::trlib PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtrlib.0.4.dylib"
  IMPORTED_SONAME_RELEASE "/Users/runner/work/casadi/casadi/build/external_projects/lib/libtrlib.0.4.dylib"
  )

list(APPEND _cmake_import_check_targets trlib::trlib )
list(APPEND _cmake_import_check_files_for_trlib::trlib "${_IMPORT_PREFIX}/lib/libtrlib.0.4.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
