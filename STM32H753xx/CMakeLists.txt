cmake_minimum_required(VERSION 3.19)


target_include_directories(${target_name}
 PUBLIC  "${CMAKE_CURRENT_LIST_DIR}/inc"
)

target_sources(${target_name}
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_Register.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_Errors.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_HW_Units.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_HW_Handles.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_HW_Manager.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_IRQ_Manager.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_GPIO.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_UART.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_Timer.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_DMA.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_DAC.hpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/inc/uC_ADC.hpp"
)

target_sources(${target_name}
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_Register.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_Errors.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_HW_Units.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_HW_Handles.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_HW_Manager.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_IRQ_Manager.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_DAC.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_GPIO.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_UART.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_Timer.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_DMA.cpp"
	PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src/uC_ADC.cpp"
)