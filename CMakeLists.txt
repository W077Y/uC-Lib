cmake_minimum_required(VERSION 3.15)


set(target_name "uC")

add_library(${target_name} STATIC)


target_include_directories(${target_name}
 PUBLIC  "${CMAKE_CURRENT_LIST_DIR}/inc"
)

target_sources(${target_name}
 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC.cpp"
)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "STM32H753xx")
	target_include_directories(${target_name}
	 PUBLIC  "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx"
	)
	target_sources(${target_name}
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_Register.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_Errors.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_HW_Units.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_HW_Handles.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_HW_Manager.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_IRQ_Manager.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_GPIO.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_UART.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_Timer.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_DMA.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_DAC.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_ADC.hpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc_STM32H753xx/uC_F-Gen.hpp"
	)

	target_sources(${target_name}
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_Register.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_Errors.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_HW_Units.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_HW_Handles.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_HW_Manager.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_IRQ_Manager.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_DAC.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_GPIO.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_UART.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_Timer.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_DMA.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_ADC.cpp"
	 PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src_STM32H753xx/uC_F-Gen.cpp"
	)
else()
	message(SEND_ERROR "missing uC port")
endif()



target_link_libraries(${target_name}
	PUBLIC BSP
	PUBLIC WLIB
	PUBLIC EX_MATH_LIB
	PUBLIC SYS_ANAL
	)