find_package (Threads)

set(CROW_HEADERS ../../Crow/include/)
set(CPP_SOURCES ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${LF_MAIN_TARGET} ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(${LF_MAIN_TARGET} PUBLIC ${CROW_HEADERS} ${CPP_SOURCES})

