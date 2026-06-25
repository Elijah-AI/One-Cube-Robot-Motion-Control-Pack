# ===================================================================================
#  The OctoMap CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    find_package(octomap REQUIRED)
#    target_link_libraries(MY_TARGET_NAME PRIVATE octomap::octomap)
#
#
#    This file will also define the following variables:
#      - OCTOMAP_LIBRARIES      : For backwards compatibility with existing code
#      - OCTOMAP_MAJOR_VERSION  : Major version.
#      - OCTOMAP_MINOR_VERSION  : Minor version.
#      - OCTOMAP_PATCH_VERSION  : Patch version.
#      - OCTOMAP_VERSION        : Major.Minor.Patch version.
#
# ===================================================================================


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was octomap-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)

set(OCTOMAP_MAJOR_VERSION "1")
set(OCTOMAP_MINOR_VERSION "10")
set(OCTOMAP_PATCH_VERSION "0")
set(OCTOMAP_VERSION "1.10.0")

set(OCTOMAP_LIBRARIES octomap::octomap)

# Additionally set the variables using a lower-case project name.
# This fixes discovery for downstream packages that search for lower-case octomap, which has been common in downstream code:
set(octomap_MAJOR_VERSION "${OCTOMAP_MAJOR_VERSION}")
set(octomap_MINOR_VERSION "${OCTOMAP_MINOR_VERSION}")
set(octomap_PATCH_VERSION "${OCTOMAP_PATCH_VERSION}")
set(octomap_VERSION "${OCTOMAP_VERSION}")
set(octomap_LIBRARIES "${OCTOMAP_LIBRARIES}")

include(${CMAKE_CURRENT_LIST_DIR}/octomap-targets.cmake)
