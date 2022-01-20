find_package (Threads)

set(CROW_HEADERS ../../Crow/include/)
set(CPP_SOURCES ../../src/)

target_link_libraries(${LF_MAIN_TARGET} ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(${LF_MAIN_TARGET} PUBLIC ${CROW_HEADERS} ${CPP_SOURCES})

