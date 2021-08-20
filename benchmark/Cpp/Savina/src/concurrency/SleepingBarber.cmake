find_package (Threads)
target_link_libraries(${LF_MAIN_TARGET} ${CMAKE_THREAD_LIBS_INIT})
include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)
