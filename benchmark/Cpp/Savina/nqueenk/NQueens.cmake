

if(TARGET NQueensBenchmark)
  target_sources(NQueensBenchmark PUBLIC ${CMAKE_CURRENT_LIST_DIR}/NQueensCommon.cc)
endif()

if(TARGET NQueensBenchmarkGenerator)
  target_sources(NQueensBenchmarkGenerator PUBLIC ${CMAKE_CURRENT_LIST_DIR}/NQueensCommon.cc)
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)

