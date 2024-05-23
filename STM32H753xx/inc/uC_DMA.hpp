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

  class DMA_Buffer_Allocator final: public uC::DMA_Buffer_Allocator_Interface
  {
  public:
    DMA_Buffer_Allocator(std::byte* begin, std::byte const* end)
        : m_beg(begin)
        , m_pos(begin)
        , m_end(end)
    {
    }

    template <std::size_t N>
    DMA_Buffer_Allocator(std::byte (&buffer)[N])
        : DMA_Buffer_Allocator(&buffer[0], &buffer[N])
    {
    }

    std::byte* allocate(std::size_t const& size_in_bytes, std::size_t align)
    {
      if (align < 32)
        align = 32;

      std::byte* ptr = this->m_pos;
      void*      pos = ptr;
      do
      {
        std::size_t sp = this->m_end - ptr;

        if (std::align(align, size_in_bytes, pos, sp) == nullptr)
        {
          return nullptr;
        }

      } while (!this->m_pos.compare_exchange_weak(ptr, reinterpret_cast<std::byte*>(pos) + size_in_bytes));
      return reinterpret_cast<std::byte*>(pos);
    }

    void deallocate(std::byte*) { uC::Errors::not_implemented(); }

  private:
    std::byte* const        m_beg;
    std::atomic<std::byte*> m_pos;
    std::byte const* const  m_end;
  };
}    // namespace uC

namespace BSP
{
  uC::DMA_Buffer_Allocator_Interface& get_dma_buffer_allocator();
}

#endif    // !UC_GPIO_HPP
