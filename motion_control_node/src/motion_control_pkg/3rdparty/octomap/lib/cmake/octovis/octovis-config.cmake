# ===================================================================================
#  The OctoVis CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    find_package(octovis REQUIRED)
#    target_link_libraries(MY_TARGET_NAME PRIVATE octomap::octovis)
#
#
#    This file will also define the following variables:
#      - OCTOVIS_LIBRARIES      : For backwards compatibility with existing code
#      - OCTOVIS_MAJOR_VERSION  : Major version.
#      - OCTOVIS_MINOR_VERSION  : Minor version.
#      - OCTOVIS_PATCH_VERSION  : Patch version.
#      - OCTOVIS_VERSION        : Major.Minor.Patch version.
#
# ===================================================================================


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was octovis-config.cmake.in                            ########

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
find_dependency(octomap 1.10.0)
find_dependency(OpenGL)

find_dependency(Qt5 5.15.3 COMPONENTS Core Gui OpenGL Widgets Xml)

set(OCTOVIS_MAJOR_VERSION "1")
set(OCTOVIS_MINOR_VERSION "10")
set(OCTOVIS_PATCH_VERSION "0")
set(OCTOVIS_VERSION "1.10.0")

set(OCTOVIS_LIBRARIES octomap::octovis)

include(${CMAKE_CURRENT_LIST_DIR}/octovis-targets.cmake)
