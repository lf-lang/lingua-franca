# Disable the old target defined in the generated cmake file
set_target_properties(${LF_MAIN_TARGET} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
# Get the source files assoicated with the generated cmake build
get_target_property(LF_SOURCES ${LF_MAIN_TARGET} SOURCES)
# Get the compile definitions added to target
get_target_property(LF_COMPILE_DEFS ${LF_MAIN_TARGET} COMPILE_DEFINITIONS)
# Use these sources with Zephyr build.
target_compile_definitions(app PUBLIC ${LF_COMPILE_DEFS})
target_sources(app PRIVATE ${LF_SOURCES})