#pragma once
#ifndef UC_HW_HANDLES_HPP
#define UC_HW_HANDLES_HPP

#include <WLib_Callback_Interface.hpp>
#include <uC_Errors.hpp>
#include <uC_HW_Manager.hpp>
#include <uC_HW_Units.hpp>
#include <uC_IRQ_Manager.hpp>

namespace uC
{
  namespace HANDLEs
  {
    class GPIO_Handle_t
    {
      GPIO_Handle_t(GPIO_Handle_t const&)            = delete;
      GPIO_Handle_t(GPIO_Handle_t&&)                 = delete;
      GPIO_Handle_t& operator=(GPIO_Handle_t const&) = delete;
      GPIO_Handle_t& operator=(GPIO_Handle_t&&)      = delete;

    public:
      enum class Speed
      {
        Low       = 0b00,
        Medium    = 0b01,
        High      = 0b10,
        Very_High = 0b11,
      };

      enum class Output_Mode
      {
        Push_Pull  = 0,
        Open_Drain = 1,
      };

      enum class Pull_Mode
      {
        No_Pull  = 0b00,
        Pullup   = 0b01,
        Pulldown = 0b10,
      };

      enum class Mode
      {
        Input  = 0b00,
        Output = 0b01,
        AF     = 0b10,
        Analog = 0b11,
      };

      GPIO_Handle_t(uC::GPIOs::HW_Unit const& gpio)
          : m_pin(gpio)
      {
        if (!uC::HW_Manager::lock(gpio))
          uC::Errors::multiple_use();

        this->m_pin.get_clk_bit().set();
      }

      ~GPIO_Handle_t()
      {
        if (uC::HW_Manager::unlock(this->m_pin) == 0)
          this->m_pin.get_clk_bit().reset();
      }

      void set_speed(Speed const& speed) const
      {
        constexpr uint32_t msk      = 0b11;
        uint32_t const     shift_no = this->m_pin.get_pin_number() * 2;
        modify_bits(this->get_base().OSPEEDR, msk << shift_no, (static_cast<uint32_t>(speed) & msk) << shift_no);
      }

      void set_output_type(Output_Mode const& output_mode) const
      {
        constexpr uint32_t msk      = 0b1;
        uint32_t const     shift_no = this->m_pin.get_pin_number();
        modify_bits(this->get_base().OTYPER, msk << shift_no, (static_cast<uint32_t>(output_mode) & msk) << shift_no);
      }

      void set_pull_mode(Pull_Mode const& pull_mode) const
      {
        constexpr uint32_t msk      = 0b11;
        uint32_t const     shift_no = this->m_pin.get_pin_number() * 2;
        modify_bits(this->get_base().PUPDR, msk << shift_no, (static_cast<uint32_t>(pull_mode) & msk) << shift_no);
      }

      void set_mode(Mode const& mode) const
      {
        constexpr uint32_t msk      = 0b11;
        uint32_t const     shift_no = this->m_pin.get_pin_number() * 2;
        modify_bits(this->get_base().MODER, msk << shift_no, (static_cast<uint32_t>(mode) & msk) << shift_no);
      }

      void set_output_state(bool const& state) const
      {
        this->get_base().BSRR = this->m_pin.get_pin_mask() << 16 | (state ? 1 << this->m_pin.get_pin_mask() : 0);
      }

      void set_alternate_function(uint32_t val) const
      {
        constexpr uint32_t msk      = 0b1111;
        uint32_t const     shift_no = (this->m_pin.get_pin_number() * 4) % 32;
        uint32_t const     sel_idx  = (this->m_pin.get_pin_number() * 4) / 32;
        uC::modify_bits(this->get_base().AFR[sel_idx], msk << shift_no, (val & msk) << shift_no);
      }

      bool is_set() { return (this->get_base().IDR & this->get_mask()) != 0; }

      constexpr GPIO_TypeDef& get_base() const { return this->m_pin.get_base(); }
      constexpr uint32_t      get_mask() const { return this->m_pin.get_pin_mask(); }

    private:
      uC::GPIOs::HW_Unit const m_pin;
    };

    class DMA_Stream_Handle_t
    {
      DMA_Stream_Handle_t(DMA_Stream_Handle_t const&)            = delete;
      DMA_Stream_Handle_t(DMA_Stream_Handle_t&&)                 = delete;
      DMA_Stream_Handle_t& operator=(DMA_Stream_Handle_t const&) = delete;
      DMA_Stream_Handle_t& operator=(DMA_Stream_Handle_t&&)      = delete;

    public:
      using irq_reason_t = uC::Internal::dma_stream_irq_reason_t;

      DMA_Stream_Handle_t(uC::DMA_Streams::HW_Unit const& hw_unit)
          : m_hw_unit(hw_unit)
      {
        if (!uC::HW_Manager::lock(this->m_hw_unit))
          uC::Errors::multiple_use();

        this->m_hw_unit.get_clk_bit().set();
      }

      ~DMA_Stream_Handle_t()
      {
        uC::IRQ_Manager::unregister_irq(this->m_hw_unit);
        if (uC::HW_Manager::unlock(this->m_hw_unit) == 0)
          this->m_hw_unit.get_clk_bit().reset();
      }

      void register_irq(WLib::Callback_Interface<void(irq_reason_t const&)>& cb_handle, uint32_t prio) const
      {
        uC::IRQ_Manager::register_irq(this->m_hw_unit, cb_handle, prio);
      }

      constexpr DMA_Stream_TypeDef&     get_base() const { return this->m_hw_unit.get_dma_stream_base(); }
      constexpr DMAMUX_Channel_TypeDef& get_mux_base() const { return this->m_hw_unit.get_mux_base(); }

    private:
      uC::DMA_Streams::HW_Unit const m_hw_unit;
    };

    class USART_Handle_t
    {
      USART_Handle_t(USART_Handle_t const&)            = delete;
      USART_Handle_t(USART_Handle_t&&)                 = delete;
      USART_Handle_t& operator=(USART_Handle_t const&) = delete;
      USART_Handle_t& operator=(USART_Handle_t&&)      = delete;

    public:
      USART_Handle_t(uC::USARTs::HW_Unit const& hw_unit)
          : m_hw_unit(hw_unit)
      {
        if (!uC::HW_Manager::lock(this->m_hw_unit))
          uC::Errors::multiple_use();

        this->m_hw_unit.get_clk_bit().set();
      }

      ~USART_Handle_t()
      {
        if (uC::HW_Manager::unlock(this->m_hw_unit) == 0)
          this->m_hw_unit.get_clk_bit().reset();
      }

      constexpr USART_TypeDef& get_base() const { return this->m_hw_unit.get_base(); }

      void register_irq(WLib::Callback_Interface<void()>& cb_handle, uint32_t prio) const { uC::IRQ_Manager::register_irq(this->m_hw_unit, cb_handle, prio); }

      uint32_t get_clk() const { return this->m_hw_unit.get_clk(); }

    private:
      uC::USARTs::HW_Unit const m_hw_unit;
    };

    class DAC_Handle_t
    {
      DAC_Handle_t(DAC_Handle_t const&)            = delete;
      DAC_Handle_t(DAC_Handle_t&&)                 = delete;
      DAC_Handle_t& operator=(DAC_Handle_t const&) = delete;
      DAC_Handle_t& operator=(DAC_Handle_t&&)      = delete;

    public:
      DAC_Handle_t(uC::DACs::HW_Unit const& hw_unit)
          : m_hw_unit(hw_unit)
      {
        if (!uC::HW_Manager::lock(this->m_hw_unit))
          uC::Errors::multiple_use();

        this->m_hw_unit.get_clk_bit().set();
      }

      ~DAC_Handle_t()
      {
        if (uC::HW_Manager::unlock(this->m_hw_unit) == 0)
          this->m_hw_unit.get_clk_bit().reset();
      }

      constexpr DAC_TypeDef& get_base() const { return this->m_hw_unit.get_base(); }

      // void register_irq(WLib::Callback_Interface<void()>& cb_handle, uint32_t prio) { uC::IRQ_Manager::register_irq(this->m_hw_unit, cb_handle, prio); }

    private:
      uC::DACs::HW_Unit const m_hw_unit;
    };

    class BASIC_TIMER_Handle_t
    {
      BASIC_TIMER_Handle_t(BASIC_TIMER_Handle_t const&)            = delete;
      BASIC_TIMER_Handle_t(BASIC_TIMER_Handle_t&&)                 = delete;
      BASIC_TIMER_Handle_t& operator=(BASIC_TIMER_Handle_t const&) = delete;
      BASIC_TIMER_Handle_t& operator=(BASIC_TIMER_Handle_t&&)      = delete;

    public:
      BASIC_TIMER_Handle_t(uC::TIMERs::HW_Unit const& hw_unit)
          : m_hw_unit(hw_unit)
      {
        if (!uC::HW_Manager::lock(this->m_hw_unit))
          uC::Errors::multiple_use();

        this->m_hw_unit.get_clk_bit().set();
      }

      ~BASIC_TIMER_Handle_t()
      {
        if (uC::HW_Manager::unlock(this->m_hw_unit) == 0)
          this->m_hw_unit.get_clk_bit().reset();
      }

      constexpr TIM_TypeDef& get_base() const { return this->m_hw_unit.get_base(); }

      void register_irq(WLib::Callback_Interface<void()>& cb_handle, uint32_t prio) const { uC::IRQ_Manager::register_irq(this->m_hw_unit, cb_handle, prio); }

      uint32_t get_clk() const { return this->m_hw_unit.get_clk(); }

    private:
      uC::TIMERs::HW_Unit const m_hw_unit;
    };

    class ADC_Handle_t
    {
      ADC_Handle_t(ADC_Handle_t const&)            = delete;
      ADC_Handle_t(ADC_Handle_t&&)                 = delete;
      ADC_Handle_t& operator=(ADC_Handle_t const&) = delete;
      ADC_Handle_t& operator=(ADC_Handle_t&&)      = delete;

    public:
      ADC_Handle_t(uC::ADCs::HW_Unit const& hw_unit)
          : m_hw_unit(hw_unit)
      {
        if (!uC::HW_Manager::lock(this->m_hw_unit))
          uC::Errors::multiple_use();

        this->m_hw_unit.get_clk_bit().set();
      }

      ~ADC_Handle_t()
      {
        if (uC::HW_Manager::unlock(this->m_hw_unit) == 0)
          this->m_hw_unit.get_clk_bit().reset();
      }

      constexpr ADC_TypeDef&        get_base() const { return this->m_hw_unit.get_base(); }
      constexpr ADC_Common_TypeDef& get_common_base() const { return this->m_hw_unit.get_common_base(); }

      // void register_irq(WLib::Callback_Interface<void()>& cb_handle, uint32_t prio) { uC::IRQ_Manager::register_irq(this->m_hw_unit, cb_handle, prio); }

      uint32_t get_clk() const { return this->m_hw_unit.get_clk(); }

    private:
      uC::ADCs::HW_Unit const m_hw_unit;
    };
  }    // namespace HANDLEs
}    // namespace uC

#endif
