cmake_minimum_required(VERSION 3.19)

set(target_name "uC")
message(STATUS "#                uC-Lib: ${target_name}")
add_library(${target_name} STATIC)


target_include_directories(${target_name}
 PUBLIC  "${CMAKE_CURRENT_LIST_DIR}/inc"
)

target_sources(${target_name}
 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC.cpp"
)

message(STATUS "#         uC-Lib-CONFIG: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "########################")
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "STM32H753xx")
	add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/STM32H753xx")
else()
	message(SEND_ERROR "missing uC port")
endif()



target_link_libraries(${target_name}
	PUBLIC BSP
	PUBLIC WLIB
	)