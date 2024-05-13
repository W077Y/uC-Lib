#pragma once
#ifndef UC_HW_MANAGER_HPP
#define UC_HW_MANAGER_HPP

#include <atomic>
#include <uC_HW_Units.hpp>

namespace uC
{
  namespace HW_Manager
  {
    bool     lock(uC::GPIOs::HW_Unit const& gpio);
    uint32_t unlock(uC::GPIOs::HW_Unit const& gpio);

    bool     lock(uC::DMA_Streams::HW_Unit const& dma_stream);
    uint32_t unlock(uC::DMA_Streams::HW_Unit const& dma_stream);

    bool     lock(uC::USARTs::HW_Unit const& hw_unit);
    uint32_t unlock(uC::USARTs::HW_Unit const& hw_unit);

    bool     lock(uC::TIMERs::HW_Unit const& hw_unit);
    uint32_t unlock(uC::TIMERs::HW_Unit const& hw_unit);

    bool     lock(uC::DACs::HW_Unit const& hw_unit);
    uint32_t unlock(uC::DACs::HW_Unit const& hw_unit);
    
    bool     lock(uC::ADCs::HW_Unit const& hw_unit);
    uint32_t unlock(uC::ADCs::HW_Unit const& hw_unit);
  }    // namespace HW_Manager
}    // namespace uC

#endif
