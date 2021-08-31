find_package(Curses REQUIRED) # Finds the lncurses library
include_directories(${CURSES_INCLUDE_DIR}) # "The include directories needed to use Curses"
target_link_libraries( ${LF_MAIN_TARGET} ${CURSES_LIBRARIES} ) # Links the Curses library