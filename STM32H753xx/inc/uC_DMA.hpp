#pragma once
#ifndef UC_DMA_HPP
#define UC_DMA_HPP

#include <span>
#include <uC_HW_Handles.hpp>
#include <wlib.hpp>

namespace uC
{
  class DMA_Buffer_Allocator_Interface
  {
  public:
    template <typename T> auto allocate(std::size_t const& number_of_entries)
    {
      using mem_payload_t = std::aligned_storage_t<sizeof(T), alignof(T)>;
      using RT            = std::span<mem_payload_t>;
      std::byte* tmp      = this->allocate(sizeof(T) * number_of_entries, alignof(T));
      if (tmp == nullptr)
      {
        uC::Errors::uC_config_error("not enouth memory");
        return RT();
      }

      return RT(reinterpret_cast<mem_payload_t*>(tmp), number_of_entries);
    }

    template <typename T> void deallocate(std::span<T>& obj) { return this->deallocate(obj.data()); }

    virtual std::byte* allocate(std::size_t const& size_in_bytes, std::size_t align) = 0;
    virtual void       deallocate(std::byte*)                                        = 0;
  };
}    // namespace uC

namespace BSP
{
  uC::DMA_Buffer_Allocator_Interface& get_dma_buffer_allocator();
}

#endif    // !UC_GPIO_HPP
