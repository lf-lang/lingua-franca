
find_package(Threads)
if(TARGET CigaretteSmokerBenchmarkGenerator)
  target_link_libraries(CigaretteSmokerBenchmarkGenerator ${CMAKE_THREAD_LIBS_INIT})
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)

