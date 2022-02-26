find_package (Threads)
find_package (Crow)

set(CROW_HEADERS ../../Crow/include/)
set(CPP_SOURCES ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${LF_MAIN_TARGET} ${CMAKE_THREAD_LIBS_INIT} Crow::Crow)
target_include_directories(${LF_MAIN_TARGET} PUBLIC ${CPP_SOURCES})

