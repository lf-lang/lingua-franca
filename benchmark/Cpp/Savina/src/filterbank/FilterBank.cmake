
if(TARGET FilterBankBenchmark)
  target_sources(FilterBankBenchmark PUBLIC ${CMAKE_CURRENT_LIST_DIR}/FilterBankCommon.cc)
endif()

if(TARGET FilterBankBenchmarkGenerator)
  target_sources(FilterBankBenchmarkGenerator PUBLIC ${CMAKE_CURRENT_LIST_DIR}/FilterBankCommon.cc)
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)

