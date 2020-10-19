

if(TARGET GuidedSearchBenchmark)
  target_sources(GuidedSearchBenchmark PUBLIC ${CMAKE_CURRENT_LIST_DIR}/GuidedSearchCommon.cc ${CMAKE_CURRENT_LIST_DIR}/GridNode.cc)
endif()

if(TARGET GuidedSearchBenchmarkGenerator)
  target_sources(GuidedSearchBenchmarkGenerator PUBLIC ${CMAKE_CURRENT_LIST_DIR}/GuidedSearchCommon.cc ${CMAKE_CURRENT_LIST_DIR}/GridNode.cc)
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)

