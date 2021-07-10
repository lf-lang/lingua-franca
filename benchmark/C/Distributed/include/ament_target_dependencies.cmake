# Copyright 2014-2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Add the interface targets or definitions, include directories and libraries
# of packages to a target.
#
# Each package name must have been find_package()-ed before.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be either _INTERFACES or _DEFINITIONS, _INCLUDE_DIRS,
# _LIBRARIES, _LIBRARY_DIRS, and _LINK_FLAGS.
# If _INTERFACES is not empty it will be used exclusively, otherwise the other
# variables are being used.
# If _LIBRARY_DIRS is not empty, _LIBRARIES which are not absolute paths already
# will be searched in those directories and their absolute paths will be used instead.
#
# :param target: the target name
# :type target: string
# :param ARGN: a list of package names, which can optionally start
#   with a SYSTEM keyword, followed by an INTERFACE or PUBLIC keyword.
#   If it starts with a SYSTEM keyword, it will be used in
#   target_include_directories() calls.
#   If it starts (or follows) with an INTERFACE or PUBLIC keyword,
#   this keyword will be used in the target_*() calls.
# :type ARGN: list of strings
#
# @public
#
function(ament_target_dependencies target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    cmake_parse_arguments(ARG "INTERFACE;PUBLIC;SYSTEM" "" "" ${ARGN})
    set(ARGVIND 1)
    set(system_keyword "")
    set(optional_keyword "")
    set(required_keyword "PUBLIC")
    if(ARG_SYSTEM)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "SYSTEM")
        message(FATAL_ERROR "ament_target_dependencies() SYSTEM keyword is only allowed before the package names and other keywords")
      endif()
      set(system_keyword SYSTEM)
      math(EXPR ARGVIND "${ARGVIND} + 1")
    endif()
    if(ARG_INTERFACE)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "INTERFACE")
        message(FATAL_ERROR "ament_target_dependencies() INTERFACE keyword is only allowed before the package names")
      endif()
      set(optional_keyword INTERFACE)
      set(required_keyword INTERFACE)
    endif()
    if(ARG_PUBLIC)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "PUBLIC")
        message(FATAL_ERROR "ament_target_dependencies() PUBLIC keyword is only allowed before the package names")
      endif()
      set(optional_keyword PUBLIC)
    endif()
    set(definitions "")
    set(include_dirs "")
    set(interfaces "")
    set(libraries "")
    set(link_flags "")
    foreach(package_name ${ARG_UNPARSED_ARGUMENTS})
      if(NOT "${${package_name}_FOUND}")
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      # if a package provides modern CMake interface targets use them
      # exclusively assuming the classic CMake variables only exist for
      # backward compatibility
      set(use_modern_cmake FALSE)
      if(NOT "${${package_name}_TARGETS}" STREQUAL "")
        foreach(_target ${${package_name}_TARGETS})
          # only use actual targets
          # in case a package uses this variable for other content
          if(TARGET "${_target}")
            list_append_unique(interfaces ${_target})
            set(use_modern_cmake TRUE)
          endif()
        endforeach()
      endif()
      if(NOT use_modern_cmake AND NOT "${${package_name}_INTERFACES}" STREQUAL "")
        foreach(_interface ${${package_name}_INTERFACES})
          # only use actual targets
          # in case a package uses this variable for other content
          if(TARGET "${_interface}")
            list_append_unique(interfaces ${_interface})
            set(use_modern_cmake TRUE)
          endif()
        endforeach()
        if(use_modern_cmake)
          message(DEPRECATION
            "Package ${package_name} is exporting the variable "
            "${package_name}_INTERFACES which is deprecated, it should export
            ${package_name}_TARGETS instead")
        endif()
      endif()
      if(NOT use_modern_cmake)
        # otherwise use the classic CMake variables
        list_append_unique(definitions ${${package_name}_DEFINITIONS})
        list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
        foreach(library ${${package_name}_LIBRARIES})
          if(NOT "${${package_name}_LIBRARY_DIRS}" STREQUAL "")
            if(NOT IS_ABSOLUTE ${library} OR NOT EXISTS ${library})
              find_library(lib NAMES ${library} PATHS ${${package_name}_LIBRARY_DIRS} NO_DEFAULT_PATH)
              if(lib)
                set(library ${lib})
              endif()
            endif()
          endif()
          list(APPEND libraries ${library})
        endforeach()
        list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
      endif()
    endforeach()
    if(NOT ARG_INTERFACE)
      target_compile_definitions(${target}
        ${required_keyword} ${definitions})
      # the interface include dirs must be ordered
      set(interface_include_dirs)
      foreach(interface ${interfaces})
        get_target_property(_include_dirs ${interface} INTERFACE_INCLUDE_DIRECTORIES)
        if(_include_dirs)
          list_append_unique(interface_include_dirs ${_include_dirs})
        endif()
      endforeach()
      ament_include_directories_order(ordered_interface_include_dirs ${interface_include_dirs})
      # the interface include dirs are used privately to ensure proper order
      # and the interfaces cover the public case
      target_include_directories(${target} ${system_keyword}
        PRIVATE ${ordered_interface_include_dirs})
    endif()
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_link_libraries(${target}
      ${optional_keyword} ${interfaces})
    target_include_directories(${target} ${system_keyword}
      ${required_keyword} ${ordered_include_dirs})
    if(NOT ARG_INTERFACE)
      ament_libraries_deduplicate(unique_libraries ${libraries})
      target_link_libraries(${target}
        ${optional_keyword} ${unique_libraries})
      foreach(link_flag IN LISTS link_flags)
        set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
      endforeach()
    endif()
  endif()
endfunction()
