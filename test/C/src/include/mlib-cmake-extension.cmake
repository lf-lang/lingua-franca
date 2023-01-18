IF (NOT WIN32)
    target_link_libraries(${LF_MAIN_TARGET} PRIVATE m) # Links the m library
ENDIF()
