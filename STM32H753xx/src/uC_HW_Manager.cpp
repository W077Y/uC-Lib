#include <tuple>
#include <uC_Errors.hpp>
#include <uC_HW_Manager.hpp>

namespace
{
  std::atomic<uint32_t> gpio_multi_use_flags[uC::GPIOs::HW_Unit::max_number_of_ports]            = {};
  std::atomic<uint32_t> dma_stream_multi_use_flags[uC::DMA_Streams::HW_Unit::max_number_of_dmas] = {};
  std::atomic<uint32_t> uart_multi_use_flags[uC::USARTs::HW_Unit::max_number_of_usarts]          = {};
  std::atomic<uint32_t> timer_multi_use_flags[uC::TIMERs::HW_Unit::max_number_of_units]          = {};
  // std::atomic<uint32_t> dac_multi_use_flags[uC::DACs::HW_Unit::max_number_of_units]              = {};
  std::atomic<uint32_t> adc_multi_use_flags[uC::ADCs::HW_Unit::max_number_of_units] = {};

  bool try_lock_hw_unit(std::atomic<uint32_t>& entry, uint32_t const& msk)
  {
    uint32_t val = entry;
    uint32_t new_val;
    do
    {
      if ((val & msk) != 0)
      {
        return false;
      }

      new_val = val | msk;
    } while (!entry.compare_exchange_strong(val, new_val));
    return true;
  }

  uint32_t unlock_hw_unit(std::atomic<uint32_t>& entry, uint32_t const& msk)
  {
    uint32_t val = entry;
    uint32_t new_val;
    do
    {
      new_val = val & ~msk;
    } while (!entry.compare_exchange_strong(val, new_val));

    return new_val;
  }

  constexpr std::tuple<std::atomic<uint32_t>&, uint32_t> get_multi_use_entry(uC::GPIOs::HW_Unit const& hw_unit)
  {
    uint32_t const idx = hw_unit.get_port_number();
    if (idx >= uC::GPIOs::HW_Unit::max_number_of_ports)
      uC::Errors::invalid_hw_unit();
    return { gpio_multi_use_flags[idx], hw_unit.get_pin_mask() };
  }

  constexpr std::tuple<std::atomic<uint32_t>&, uint32_t> get_multi_use_entry(uC::DMA_Streams::HW_Unit const& hw_unit)
  {
    uint32_t const idx = hw_unit.get_dma_number();
    if (idx >= uC::DMA_Streams::HW_Unit::max_number_of_dmas)
      uC::Errors::invalid_hw_unit();
    return { dma_stream_multi_use_flags[idx], 1 << hw_unit.get_stream_number() };
  }

  constexpr std::tuple<std::atomic<uint32_t>&, uint32_t> get_multi_use_entry(uC::USARTs::HW_Unit const& hw_unit)
  {
    uint32_t const idx = hw_unit.get_number();
    if (idx >= uC::USARTs::HW_Unit::max_number_of_usarts)
      uC::Errors::invalid_hw_unit();
    return { uart_multi_use_flags[hw_unit.get_number()], 1 };
  }

  constexpr std::tuple<std::atomic<uint32_t>&, uint32_t> get_multi_use_entry(uC::TIMERs::HW_Unit const& hw_unit)
  {
    uint32_t const idx = hw_unit.get_number();
    if (idx >= uC::TIMERs::HW_Unit::max_number_of_units)
      uC::Errors::invalid_hw_unit();
    return { timer_multi_use_flags[hw_unit.get_number()], 1 };
  }

  constexpr std::tuple<std::atomic<uint32_t>&, uint32_t> get_multi_use_entry(uC::DACs::HW_Unit const& hw_unit)
  {
    uint32_t const idx = hw_unit.get_number();
    if (idx >= uC::DACs::HW_Unit::max_number_of_units)
      uC::Errors::invalid_hw_unit();
    return { timer_multi_use_flags[idx], 1 };
  }

  constexpr std::tuple<std::atomic<uint32_t>&, uint32_t> get_multi_use_entry(uC::ADCs::HW_Unit const& hw_unit)
  {
    uint32_t const idx = hw_unit.get_blk_number();
    if (idx >= uC::ADCs::HW_Unit::max_number_of_units)
      uC::Errors::invalid_hw_unit();
    return { adc_multi_use_flags[idx], 1 << hw_unit.get_adc_number() };
  }

  template <typename T> bool try_lock_hw_unit(T const& name)
  {
    auto [entry, msk] = get_multi_use_entry(name);
    return try_lock_hw_unit(entry, msk);
  }
  template <typename T> uint32_t unlock_hw_unit(T const& name)
  {
    auto [entry, msk] = get_multi_use_entry(name);
    return unlock_hw_unit(entry, msk);
  }
}    // namespace

namespace uC::HW_Manager
{
  bool     lock(uC::GPIOs::HW_Unit const& hw_unit) { return try_lock_hw_unit(hw_unit); }
  uint32_t unlock(uC::GPIOs::HW_Unit const& hw_unit) { return unlock_hw_unit(hw_unit); }

  bool     lock(uC::USARTs::HW_Unit const& hw_unit) { return try_lock_hw_unit(hw_unit); }
  uint32_t unlock(uC::USARTs::HW_Unit const& hw_unit) { return unlock_hw_unit(hw_unit); }

  bool     lock(uC::DMA_Streams::HW_Unit const& hw_unit) { return try_lock_hw_unit(hw_unit); }
  uint32_t unlock(uC::DMA_Streams::HW_Unit const& hw_unit) { return unlock_hw_unit(hw_unit); }

  bool     lock(uC::TIMERs::HW_Unit const& hw_unit) { return try_lock_hw_unit(hw_unit); }
  uint32_t unlock(uC::TIMERs::HW_Unit const& hw_unit) { return unlock_hw_unit(hw_unit); }

  bool     lock(uC::DACs::HW_Unit const& hw_unit) { return try_lock_hw_unit(hw_unit); }
  uint32_t unlock(uC::DACs::HW_Unit const& hw_unit) { return unlock_hw_unit(hw_unit); }

  bool     lock(uC::ADCs::HW_Unit const& hw_unit) { return try_lock_hw_unit(hw_unit); }
  uint32_t unlock(uC::ADCs::HW_Unit const& hw_unit) { return unlock_hw_unit(hw_unit); }
}    // namespace uC::HW_Manager
