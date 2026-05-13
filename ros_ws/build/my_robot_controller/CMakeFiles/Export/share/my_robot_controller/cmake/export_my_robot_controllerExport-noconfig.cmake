#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "my_robot_controller::my_robot_controller" for configuration ""
set_property(TARGET my_robot_controller::my_robot_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(my_robot_controller::my_robot_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmy_robot_controller.so"
  IMPORTED_SONAME_NOCONFIG "libmy_robot_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS my_robot_controller::my_robot_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_my_robot_controller::my_robot_controller "${_IMPORT_PREFIX}/lib/libmy_robot_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
