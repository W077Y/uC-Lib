#pragma once
#ifndef UC_IRQ_Manager_HPP
#define UC_IRQ_Manager_HPP

#include <WLib_Callback_Interface.hpp>
#include <uC_HW_Units.hpp>
namespace uC
{
  namespace IRQ_Manager
  {
    class IRQ_Priority
    {
    public:
      constexpr IRQ_Priority(int32_t const& val)
          : m_val(val)
      {
      }

      uint32_t calculate_prio() const;

    private:
      int32_t m_val;
    };

    void register_irq(uC::USARTs::HW_Unit const& hw_unit, WLib::Callback_Interface<void()>& cb_handle, IRQ_Priority const& prio);
    void unregister_irq(uC::USARTs::HW_Unit const& hw_unit);

    void register_irq(uC::DMA_Streams::HW_Unit const&                                               hw_unit,
                      WLib::Callback_Interface<void(uC::Internal::dma_stream_irq_reason_t const&)>& cb_handle,
                      IRQ_Priority const&                                                           prio);
    void unregister_irq(uC::DMA_Streams::HW_Unit const& hw_unit);

    void register_irq(uC::TIMERs::HW_Unit const& hw_unit, WLib::Callback_Interface<void()>& cb_handle, IRQ_Priority const& prio);
    void unregister_irq(uC::TIMERs::HW_Unit const& hw_unit);
  }    // namespace IRQ_Manager
}    // namespace uC

#endif
