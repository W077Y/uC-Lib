#include <WLib_Callback_Interface.hpp>
#include <atomic>
#include <tuple>
#include <uC_DMA.hpp>
#include <uC_Errors.hpp>
#include <uC_IRQ_Manager.hpp>
#include <uC_Register.hpp>
#include <uC_UART.hpp>

namespace
{
  WLib::Callback_Interface<void()>*                                                      irq_handler_usart[uC::USARTs::HW_Unit::max_number_of_usarts]{};
  WLib::Callback_Interface<void(uC::HANDLEs::DMA_Stream_Handle_t::irq_reason_t const&)>* irq_handler_dma_stream[2][8]{};

  constexpr std::tuple<IRQn_Type, WLib::Callback_Interface<void()>*&> get_entry(uC::USARTs::HW_Unit const& hw_unit)
  {
    return { hw_unit.get_irq_type(), irq_handler_usart[hw_unit.get_number()] };
  }

  constexpr std::tuple<IRQn_Type, WLib::Callback_Interface<void(uC::HANDLEs::DMA_Stream_Handle_t::irq_reason_t const&)>*&>
  get_entry(uC::DMA_Streams::HW_Unit const& hw_unit)
  {
    return { hw_unit.get_irq_type(), irq_handler_dma_stream[hw_unit.get_dma_number()][hw_unit.get_stream_number()] };
  }

  void enable_irq(IRQn_Type const& irq_idx, uC::IRQ_Manager::IRQ_Priority const& prio)
  {
    if (NVIC_GetEnableIRQ(irq_idx) != 0)
      uC::Errors::uC_config_error("IRQ already in use");

    NVIC_SetPriority(irq_idx, prio.calculate_prio());
    NVIC_ClearPendingIRQ(irq_idx);
    NVIC_EnableIRQ(irq_idx);
  }
  void disable_irq(IRQn_Type const& irq_idx) { NVIC_DisableIRQ(irq_idx); }

}    // namespace

namespace uC::IRQ_Manager
{
  uint32_t IRQ_Priority::calculate_prio() const
  {
    int32_t const val = this->m_val + 5;
    if (val < 0)
      return 0;
    if (val > 15)
      return 15;
    return val;
  }

  void register_irq(uC::USARTs::HW_Unit const& uart_name, WLib::Callback_Interface<void()>& cb_handle, IRQ_Priority const& prio)
  {
    auto [irq_idx, handler] = get_entry(uart_name);
    handler                 = &cb_handle;
    enable_irq(irq_idx, prio);
  }

  void unregister_irq(uC::USARTs::HW_Unit const& uart_name)
  {
    auto [irq_idx, handler] = get_entry(uart_name);
    disable_irq(irq_idx);
    handler = nullptr;
  }

  void register_irq(uC::DMA_Streams::HW_Unit const&                                                        hw_unit,
                    WLib::Callback_Interface<void(uC::HANDLEs::DMA_Stream_Handle_t::irq_reason_t const&)>& cb_handle,
                    IRQ_Priority const&                                                                    prio)
  {
    auto [irq_idx, handler] = get_entry(hw_unit);
    handler                 = &cb_handle;
    enable_irq(irq_idx, prio);
  }

  void unregister_irq(uC::DMA_Streams::HW_Unit const& hw_unit)
  {
    auto [irq_idx, handler] = get_entry(hw_unit);
    disable_irq(irq_idx);
    handler = nullptr;
  }
}    // namespace uC::IRQ_Manager

uC::HANDLEs::DMA_Stream_Handle_t::irq_reason_t get_and_clear_reason(uC::register_t sr, uC::register_t clear, uint32_t sht)
{
  constexpr uint32_t msk = 0b11'1101;
  uint32_t const     val = sr & (msk << sht);
  clear                  = val;
  return { val >> sht };
}

extern "C" void DMA1_Stream0_IRQHandler() { irq_handler_dma_stream[0][0]->operator()(get_and_clear_reason(DMA1->LISR, DMA1->LIFCR, 0)); }
extern "C" void DMA1_Stream1_IRQHandler() { irq_handler_dma_stream[0][1]->operator()(get_and_clear_reason(DMA1->LISR, DMA1->LIFCR, 6)); }
extern "C" void DMA1_Stream2_IRQHandler() { irq_handler_dma_stream[0][2]->operator()(get_and_clear_reason(DMA1->LISR, DMA1->LIFCR, 16)); }
extern "C" void DMA1_Stream3_IRQHandler() { irq_handler_dma_stream[0][3]->operator()(get_and_clear_reason(DMA1->LISR, DMA1->LIFCR, 22)); }
extern "C" void DMA1_Stream4_IRQHandler() { irq_handler_dma_stream[0][4]->operator()(get_and_clear_reason(DMA1->HISR, DMA1->HIFCR, 0)); }
extern "C" void DMA1_Stream5_IRQHandler() { irq_handler_dma_stream[0][5]->operator()(get_and_clear_reason(DMA1->HISR, DMA1->HIFCR, 6)); }
extern "C" void DMA1_Stream6_IRQHandler() { irq_handler_dma_stream[0][6]->operator()(get_and_clear_reason(DMA1->HISR, DMA1->HIFCR, 16)); }
extern "C" void DMA1_Stream7_IRQHandler() { irq_handler_dma_stream[0][7]->operator()(get_and_clear_reason(DMA1->HISR, DMA1->HIFCR, 22)); }

extern "C" void DMA2_Stream0_IRQHandler() { irq_handler_dma_stream[1][0]->operator()(get_and_clear_reason(DMA2->LISR, DMA2->LIFCR, 0)); }
extern "C" void DMA2_Stream1_IRQHandler() { irq_handler_dma_stream[1][1]->operator()(get_and_clear_reason(DMA2->LISR, DMA2->LIFCR, 6)); }
extern "C" void DMA2_Stream2_IRQHandler() { irq_handler_dma_stream[1][2]->operator()(get_and_clear_reason(DMA2->LISR, DMA2->LIFCR, 16)); }
extern "C" void DMA2_Stream3_IRQHandler() { irq_handler_dma_stream[1][3]->operator()(get_and_clear_reason(DMA2->LISR, DMA2->LIFCR, 22)); }
extern "C" void DMA2_Stream4_IRQHandler() { irq_handler_dma_stream[1][4]->operator()(get_and_clear_reason(DMA2->HISR, DMA2->HIFCR, 0)); }
extern "C" void DMA2_Stream5_IRQHandler() { irq_handler_dma_stream[1][5]->operator()(get_and_clear_reason(DMA2->HISR, DMA2->HIFCR, 6)); }
extern "C" void DMA2_Stream6_IRQHandler() { irq_handler_dma_stream[1][6]->operator()(get_and_clear_reason(DMA2->HISR, DMA2->HIFCR, 16)); }
extern "C" void DMA2_Stream7_IRQHandler() { irq_handler_dma_stream[1][7]->operator()(get_and_clear_reason(DMA2->HISR, DMA2->HIFCR, 22)); }

extern "C" void USART1_IRQHandler() { irq_handler_usart[0]->operator()(); }
extern "C" void USART2_IRQHandler() { irq_handler_usart[1]->operator()(); }
extern "C" void USART3_IRQHandler() { irq_handler_usart[2]->operator()(); }

// extern "C" void NMI_Handler()
//{
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HardFault_Handler()
//{
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void MemManage_Handler()
//{
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BusFault_Handler()
//{
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void UsageFault_Handler()
//{
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SVC_Handler()
//{
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DebugMon_Handler()
//{
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void PendSV_Handler()
//{
//   __asm("bkpt 255");
//   __asm("bx lr");
// }

// Not Managed
// extern "C" void SysTick_Handler()
//{
//  __asm("bkpt 255");
//  __asm("bx lr");
//}

// extern "C" void WWDG_IRQHandler()
//{
//   //	extern "C" void WWDG_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void PVD_AVD_IRQHandler()
//{
//   //	extern "C" void PVD_AVD_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TAMP_STAMP_IRQHandler()
//{
//   //	extern "C" void TAMP_STAMP_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void RTC_WKUP_IRQHandler()
//{
//   //	extern "C" void RTC_WKUP_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void FLASH_IRQHandler()
//{
//   //	extern "C" void FLASH_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void RCC_IRQHandler()
//{
//   //	extern "C" void RCC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void EXTI0_IRQHandler()
//{
//   //	extern "C" void EXTI0_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void EXTI1_IRQHandler()
//{
//   //	extern "C" void EXTI1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void EXTI2_IRQHandler()
//{
//   //	extern "C" void EXTI2_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void EXTI3_IRQHandler()
//{
//   //	extern "C" void EXTI3_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void EXTI4_IRQHandler()
//{
//   //	extern "C" void EXTI4_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
//}

// extern "C" void ADC_IRQHandler()
//{
//   //	extern "C" void ADC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void FDCAN1_IT0_IRQHandler()
//{
//   //	extern "C" void FDCAN1_IT0_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void FDCAN2_IT0_IRQHandler()
//{
//   //	extern "C" void FDCAN2_IT0_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void FDCAN1_IT1_IRQHandler()
//{
//   //	extern "C" void FDCAN1_IT1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void FDCAN2_IT1_IRQHandler()
//{
//   //	extern "C" void FDCAN2_IT1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void EXTI9_5_IRQHandler()
//{
//   //	extern "C" void EXTI9_5_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM1_BRK_IRQHandler()
//{
//   //	extern "C" void TIM1_BRK_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM1_UP_IRQHandler()
//{
//   //	extern "C" void TIM1_UP_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM1_TRG_COM_IRQHandler()
//{
//   //	extern "C" void TIM1_TRG_COM_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM1_CC_IRQHandler()
//{
//   //	extern "C" void TIM1_CC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM2_IRQHandler()
//{
//   //	extern "C" void TIM2_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM3_IRQHandler()
//{
//   //	extern "C" void TIM3_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM4_IRQHandler()
//{
//   //	extern "C" void TIM4_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void I2C1_EV_IRQHandler()
//{
//   //	extern "C" void I2C1_EV_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void I2C1_ER_IRQHandler()
//{
//   //	extern "C" void I2C1_ER_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void I2C2_EV_IRQHandler()
//{
//   //	extern "C" void I2C2_EV_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void I2C2_ER_IRQHandler()
//{
//   //	extern "C" void I2C2_ER_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SPI1_IRQHandler()
//{
//   //	extern "C" void SPI1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SPI2_IRQHandler()
//{
//   //	extern "C" void SPI2_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }

// extern "C" void EXTI15_10_IRQHandler()
//{
//   //	extern "C" void EXTI15_10_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void RTC_Alarm_IRQHandler()
//{
//   //	extern "C" void RTC_Alarm_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM8_BRK_TIM12_IRQHandler()
//{
//   //	extern "C" void TIM8_BRK_TIM12_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM8_UP_TIM13_IRQHandler()
//{
//   //	extern "C" void TIM8_UP_TIM13_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
//{
//   //	extern "C" void TIM8_TRG_COM_TIM14_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM8_CC_IRQHandler()
//{
//   //	extern "C" void TIM8_CC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void FMC_IRQHandler()
//{
//   //	extern "C" void FMC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SDMMC1_IRQHandler()
//{
//   //	extern "C" void SDMMC1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM5_IRQHandler()
//{
//   //	extern "C" void TIM5_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SPI3_IRQHandler()
//{
//   //	extern "C" void SPI3_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void UART4_IRQHandler()
//{
//   //	extern "C" void UART4_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void UART5_IRQHandler()
//{
//   //	extern "C" void UART5_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM6_DAC_IRQHandler()
//{
//   //	extern "C" void TIM6_DAC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM7_IRQHandler()
//{
//   //	extern "C" void TIM7_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }

// extern "C" void ETH_IRQHandler()
//{
//   //	extern "C" void ETH_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void ETH_WKUP_IRQHandler()
//{
//   //	extern "C" void ETH_WKUP_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void FDCAN_CAL_IRQHandler()
//{
//   //	extern "C" void FDCAN_CAL_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void USART6_IRQHandler()
//{
//   //	extern "C" void USART6_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void I2C3_EV_IRQHandler()
//{
//   //	extern "C" void I2C3_EV_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void I2C3_ER_IRQHandler()
//{
//   //	extern "C" void I2C3_ER_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void OTG_HS_EP1_OUT_IRQHandler()
//{
//   //	extern "C" void OTG_HS_EP1_OUT_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void OTG_HS_EP1_IN_IRQHandler()
//{
//   //	extern "C" void OTG_HS_EP1_IN_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void OTG_HS_WKUP_IRQHandler()
//{
//   //	extern "C" void OTG_HS_WKUP_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void OTG_HS_IRQHandler()
//{
//   //	extern "C" void OTG_HS_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DCMI_IRQHandler()
//{
//   //	extern "C" void DCMI_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void CRYP_IRQHandler()
//{
//   //	extern "C" void CRYP_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HASH_RNG_IRQHandler()
//{
//   //	extern "C" void HASH_RNG_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void FPU_IRQHandler()
//{
//   //	extern "C" void FPU_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void UART7_IRQHandler()
//{
//   //	extern "C" void UART7_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void UART8_IRQHandler()
//{
//   //	extern "C" void UART8_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SPI4_IRQHandler()
//{
//   //	extern "C" void SPI4_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SPI5_IRQHandler()
//{
//   //	extern "C" void SPI5_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SPI6_IRQHandler()
//{
//   //	extern "C" void SPI6_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SAI1_IRQHandler()
//{
//   //	extern "C" void SAI1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void LTDC_IRQHandler()
//{
//   //	extern "C" void LTDC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void LTDC_ER_IRQHandler()
//{
//   //	extern "C" void LTDC_ER_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DMA2D_IRQHandler()
//{
//   //	extern "C" void DMA2D_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SAI2_IRQHandler()
//{
//   //	extern "C" void SAI2_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void QUADSPI_IRQHandler()
//{
//   //	extern "C" void QUADSPI_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void LPTIM1_IRQHandler()
//{
//   //	extern "C" void LPTIM1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void CEC_IRQHandler()
//{
//   //	extern "C" void CEC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void I2C4_EV_IRQHandler()
//{
//   //	extern "C" void I2C4_EV_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void I2C4_ER_IRQHandler()
//{
//   //	extern "C" void I2C4_ER_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SPDIF_RX_IRQHandler()
//{
//   //	extern "C" void SPDIF_RX_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void OTG_FS_EP1_OUT_IRQHandler()
//{
//   //	extern "C" void OTG_FS_EP1_OUT_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void OTG_FS_EP1_IN_IRQHandler()
//{
//   //	extern "C" void OTG_FS_EP1_IN_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void OTG_FS_WKUP_IRQHandler()
//{
//   //	extern "C" void OTG_FS_WKUP_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void OTG_FS_IRQHandler()
//{
//   //	extern "C" void OTG_FS_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DMAMUX1_OVR_IRQHandler()
//{
//   //	extern "C" void DMAMUX1_OVR_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HRTIM1_Master_IRQHandler()
//{
//   //	extern "C" void HRTIM1_Master_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HRTIM1_TIMA_IRQHandler()
//{
//   //	extern "C" void HRTIM1_TIMA_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HRTIM1_TIMB_IRQHandler()
//{
//   //	extern "C" void HRTIM1_TIMB_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HRTIM1_TIMC_IRQHandler()
//{
//   //	extern "C" void HRTIM1_TIMC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HRTIM1_TIMD_IRQHandler()
//{
//   //	extern "C" void HRTIM1_TIMD_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HRTIM1_TIME_IRQHandler()
//{
//   //	extern "C" void HRTIM1_TIME_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HRTIM1_FLT_IRQHandler()
//{
//   //	extern "C" void HRTIM1_FLT_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DFSDM1_FLT0_IRQHandler()
//{
//   //	extern "C" void DFSDM1_FLT0_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DFSDM1_FLT1_IRQHandler()
//{
//   //	extern "C" void DFSDM1_FLT1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DFSDM1_FLT2_IRQHandler()
//{
//   //	extern "C" void DFSDM1_FLT2_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DFSDM1_FLT3_IRQHandler()
//{
//   //	extern "C" void DFSDM1_FLT3_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SAI3_IRQHandler()
//{
//   //	extern "C" void SAI3_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SWPMI1_IRQHandler()
//{
//   //	extern "C" void SWPMI1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM15_IRQHandler()
//{
//   //	extern "C" void TIM15_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM16_IRQHandler()
//{
//   //	extern "C" void TIM16_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void TIM17_IRQHandler()
//{
//   //	extern "C" void TIM17_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void MDIOS_WKUP_IRQHandler()
//{
//   //	extern "C" void MDIOS_WKUP_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void MDIOS_IRQHandler()
//{
//   //	extern "C" void MDIOS_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void JPEG_IRQHandler()
//{
//   //	extern "C" void JPEG_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void MDMA_IRQHandler()
//{
//   //	extern "C" void MDMA_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SDMMC2_IRQHandler()
//{
//   //	extern "C" void SDMMC2_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void HSEM1_IRQHandler()
//{
//   //	extern "C" void HSEM1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void ADC3_IRQHandler()
//{
//   //	extern "C" void ADC3_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void DMAMUX2_OVR_IRQHandler()
//{
//   //	extern "C" void DMAMUX2_OVR_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BDMA_Channel0_IRQHandler()
//{
//   //	extern "C" void BDMA_Channel0_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BDMA_Channel1_IRQHandler()
//{
//   //	extern "C" void BDMA_Channel1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BDMA_Channel2_IRQHandler()
//{
//   //	extern "C" void BDMA_Channel2_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BDMA_Channel3_IRQHandler()
//{
//   //	extern "C" void BDMA_Channel3_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BDMA_Channel4_IRQHandler()
//{
//   //	extern "C" void BDMA_Channel4_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BDMA_Channel5_IRQHandler()
//{
//   //	extern "C" void BDMA_Channel5_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BDMA_Channel6_IRQHandler()
//{
//   //	extern "C" void BDMA_Channel6_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void BDMA_Channel7_IRQHandler()
//{
//   //	extern "C" void BDMA_Channel7_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void COMP1_IRQHandler()
//{
//   //	extern "C" void COMP1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void LPTIM2_IRQHandler()
//{
//   //	extern "C" void LPTIM2_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void LPTIM3_IRQHandler()
//{
//   //	extern "C" void LPTIM3_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void LPTIM4_IRQHandler()
//{
//   //	extern "C" void LPTIM4_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void LPTIM5_IRQHandler()
//{
//   //	extern "C" void LPTIM5_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void LPUART1_IRQHandler()
//{
//   //	extern "C" void LPUART1_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void CRS_IRQHandler()
//{
//   //	extern "C" void CRS_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void ECC_IRQHandler()
//{
//   //	extern "C" void ECC_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void SAI4_IRQHandler()
//{
//   //	extern "C" void SAI4_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
// extern "C" void WAKEUP_PIN_IRQHandler()
//{
//   //	extern "C" void WAKEUP_PIN_IRQHandler();
//   __asm("bkpt 255");
//   __asm("bx lr");
// }
