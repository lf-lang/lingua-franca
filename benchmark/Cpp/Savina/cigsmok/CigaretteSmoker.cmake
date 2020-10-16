
find_package(Threads)
if(TARGET CigaretteSmokerReactorCppBenchmark)
  target_link_libraries(CigaretteSmokerReactorCppBenchmark ${CMAKE_THREAD_LIBS_INIT})
endif()

if(TARGET CigaretteSmokerGeneratedReactorCppBenchmark)
  target_link_libraries(CigaretteSmokerGeneratedReactorCppBenchmark ${CMAKE_THREAD_LIBS_INIT})
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)

