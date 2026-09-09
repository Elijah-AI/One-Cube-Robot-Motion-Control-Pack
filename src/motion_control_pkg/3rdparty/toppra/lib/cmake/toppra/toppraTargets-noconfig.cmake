#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "toppra::toppra" for configuration ""
set_property(TARGET toppra::toppra APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(toppra::toppra PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libtoppra.so"
  IMPORTED_SONAME_NOCONFIG "libtoppra.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS toppra::toppra )
list(APPEND _IMPORT_CHECK_FILES_FOR_toppra::toppra "${_IMPORT_PREFIX}/lib/libtoppra.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
