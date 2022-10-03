# Disable the old target defined in the generated cmake file
set_target_properties(${LF_MAIN_TARGET} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
# Get the source files assoicated with the generated cmake build
get_target_property(LF_SOURCES ${LF_MAIN_TARGET} SOURCES)
# Use these sources with Zephyr build.
target_sources(app PRIVATE ${LF_SOURCES})