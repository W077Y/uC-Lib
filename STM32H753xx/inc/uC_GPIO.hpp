#pragma once
#ifndef UC_GPIO_HPP
#define UC_GPIO_HPP

#include <uC_HW_Handles.hpp>
#include <wlib.hpp>

namespace uC
{
  class Input_Pin final: public wlib::abstraction::Digital_Input_Interface
  {
  public:
    using Pull_Mode = uC::HANDLEs::GPIO_Handle_t::Pull_Mode;

    Input_Pin(uC::GPIOs::HW_Unit const& gpio, Pull_Mode const& pull_mode)
        : m_handle(gpio)
    {
      this->m_handle.set_pull_mode(pull_mode);
      this->m_handle.set_mode(HANDLEs::GPIO_Handle_t::Mode::Input);
    }

    bool get() override { return this->m_handle.is_set(); }

  private:
    HANDLEs::GPIO_Handle_t m_handle;
  };

  class Output_Pin final: public wlib::abstraction::Digital_Output_Interface
  {
  public:
    using Speed       = uC::HANDLEs::GPIO_Handle_t::Speed;
    using Output_Mode = uC::HANDLEs::GPIO_Handle_t::Output_Mode;
    using Pull_Mode   = uC::HANDLEs::GPIO_Handle_t::Pull_Mode;

    Output_Pin(uC::GPIOs::HW_Unit const& gpio, Speed const& speed, Output_Mode const& output_mode, Pull_Mode const& pull_mode, bool const& init_value)
        : m_handle(gpio)
    {
      this->m_handle.set_speed(speed);
      this->m_handle.set_output_type(output_mode);
      this->m_handle.set_pull_mode(pull_mode);
      this->m_handle.set_output_state(init_value);
      this->m_handle.set_mode(HANDLEs::GPIO_Handle_t::Mode::Output);
    }

    ~Output_Pin() = default;

    void set_high() override { this->m_handle.get_base().BSRR = this->m_handle.get_mask(); }
    void set_low() override { this->m_handle.get_base().BSRR = this->m_handle.get_mask() << 16; }
    void toggle() override
    {
      uint32_t const pin_state       = this->m_handle.get_base().ODR;
      this->m_handle.get_base().BSRR = (pin_state & this->m_handle.get_mask()) << 16 | (~pin_state & this->m_handle.get_mask());
    }

    bool get() override { return this->m_handle.is_set(); }

  private:
    HANDLEs::GPIO_Handle_t m_handle;
  };

  class Alternative_Funktion_Pin
  {
  public:
    using Speed       = uC::HANDLEs::GPIO_Handle_t::Speed;
    using Output_Mode = uC::HANDLEs::GPIO_Handle_t::Output_Mode;
    using Pull_Mode   = uC::HANDLEs::GPIO_Handle_t::Pull_Mode;

    Alternative_Funktion_Pin(uC::GPIOs::HW_Unit const& gpio, Speed const& speed, Output_Mode const& output_mode, Pull_Mode const& pull_mode, uint32_t af_value)
        : m_handle(gpio)
    {
      this->m_handle.set_speed(speed);
      this->m_handle.set_output_type(output_mode);
      this->m_handle.set_pull_mode(pull_mode);
      this->m_handle.set_alternate_function(af_value);
      this->m_handle.set_mode(uC::HANDLEs::GPIO_Handle_t::Mode::AF);
    }
    ~Alternative_Funktion_Pin() = default;

  private:
    HANDLEs::GPIO_Handle_t m_handle;
  };

  class Analog_Pin
  {
  public:
    Analog_Pin(uC::GPIOs::HW_Unit const& gpio)
        : m_handle(gpio)
    {
      this->m_handle.set_mode(uC::HANDLEs::GPIO_Handle_t::Mode::Analog);
    }

    ~Analog_Pin() = default;

  private:
    HANDLEs::GPIO_Handle_t m_handle;
  };
}    // namespace uC

#endif    // !UC_GPIO_HPP
