if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    target_sources(${LF_MAIN_TARGET} PRIVATE audio_loop_linux.c)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    target_sources(${LF_MAIN_TARGET} PRIVATE audio_loop_mac.c)
else()
    message(FATAL_ERROR "Your platform is not supported!"
        " The C target supports Linux, MacOS and Windows.")
endif()
