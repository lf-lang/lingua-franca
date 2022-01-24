find_package (Threads)
find_package (CROW)

set(CPP_SOURCES ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${LF_MAIN_TARGET} PUBLIC ${CMAKE_THREAD_LIBS_INIT} CROW::CROW)
target_include_directories(${LF_MAIN_TARGET} PUBLIC CROW::CROW ${CPP_SOURCES})

