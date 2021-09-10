if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    find_package( ALSA REQUIRED)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    target_link_libraries( ${LF_MAIN_TARGET} "-framework AudioToolbox" )
    target_link_libraries( ${LF_MAIN_TARGET} "-framework CoreFoundation" )
else()
    message(FATAL_ERROR "Your platform is not supported!"
        " The C target supports Linux, MacOS and Windows.")
endif()