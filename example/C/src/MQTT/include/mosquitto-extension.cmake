# find_package(Mosquitto REQUIRED)x
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
list(APPEND CMAKE_PREFIX_PATH "/usr/local/")
include(FindMosquitto.cmake)
include_directories(/usr/local/include)
target_link_libraries(${LF_MAIN_TARGET} ${MOSQUITTO_LIBRARY})
target_sources(${LF_MAIN_TARGET} PRIVATE "core/federated/net_util.c")
