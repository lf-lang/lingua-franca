# Disable the old target defined in the generated cmake file
set_target_properties(${LF_MAIN_TARGET} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
# Get the source files assoicated with the generated cmake build
get_target_property(LF_SOURCES ${LF_MAIN_TARGET} SOURCES)
get_target_property(LF_COMPILE_DEFS ${LF_MAIN_TARGET} COMPILE_DEFINITIONS)
get_target_property(LF_LINKS ${LF_MAIN_TARGET} LINK_LIBRARIES)
get_target_property(LF_INCLUDES ${LF_MAIN_TARGET} INCLUDE_DIRECTORIES)
# Use these sources with Zephyr build.
target_compile_definitions(app PUBLIC ${LF_COMPILE_DEFS})
target_sources(app PRIVATE ${LF_SOURCES})
target_link_libraries(app PUBLIC ${LF_LINKS})
target_include_directories(app PUBLIC ${LF_INCLUDES})

# Link corelib with zephyr
target_link_libraries(core PUBLIC zephyr_interface)
zephyr_library_compile_options(-Wl,--print-memory-usage)
