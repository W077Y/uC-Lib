#pragma once
#ifndef UC_REGISTER_HPP
#define UC_REGISTER_HPP

#include <cstdint>

namespace uC
{
  using register_t = uint32_t volatile&;

  inline void     set_bits(register_t reg, uint32_t const& mask) { reg |= mask; }
  inline void     reset_bits(register_t reg, uint32_t const& mask) { reg &= mask; }
  inline void     modify_bits(register_t reg, uint32_t const& mask, uint32_t const& val) { reg = (reg & ~mask) | val; }
  inline uint32_t get_bits(register_t reg, uint32_t const& mask) { return reg & mask; }
  inline uint32_t set_bits_with_rb(register_t reg, uint32_t const& mask)
  {
    set_bits(reg, mask);
    return get_bits(reg, mask);
  }
  inline uint32_t reset_bits_with_rb(register_t reg, uint32_t const& mask)
  {
    reset_bits(reg, mask);
    return get_bits(reg, mask);
  }

  class register_bit_t
  {
    static uint32_t volatile default_value;

  public:
    constexpr register_bit_t() = default;
    constexpr register_bit_t(register_t reg, uint32_t msk)
        : m_reg(reg)
        , m_msk(msk)
    {
    }

    void set() { set_bits_with_rb(this->m_reg, this->m_msk); }
    void reset() { reset_bits_with_rb(this->m_reg, this->m_msk); }

  private:
    register_t m_reg = default_value;
    uint32_t   m_msk = 0;
  };
}    // namespace uC

#endif
