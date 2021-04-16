

if(TARGET MatMulBenchmark)
  target_sources(MatMulBenchmark PUBLIC ${CMAKE_CURRENT_LIST_DIR}/MatMulCommon.cc)
endif()

if(TARGET MatMulBenchmarkGenerator)
  target_sources(MatMulBenchmarkGenerator PUBLIC ${CMAKE_CURRENT_LIST_DIR}/MatMulCommon.cc)
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)

