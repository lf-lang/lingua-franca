
if(TARGET PiPrecisionBenchmark)
  target_link_libraries(PiPrecisionBenchmark gmpxx gmp)
endif()

if(TARGET PiPrecisionBenchmarkGenerator)
  target_link_libraries(PiPrecisionBenchmarkGenerator gmpxx gmp)
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)

