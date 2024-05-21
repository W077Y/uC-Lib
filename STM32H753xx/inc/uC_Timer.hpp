#pragma once
#ifndef UC_TIMER_HPP
#define UC_TIMER_HPP

#include <cmath>
#include <uC_GPIO.hpp>
#include <uC_HW_Handles.hpp>
#include <wlib.hpp>

namespace uC
{
  class PWM_Timer: public wlib::abstraction::PWM_Interface
  {
    using this_t = PWM_Timer;
    struct af_pin_t
    {
      uC::GPIOs::HW_Unit const pin;
      uint32_t const           af_val;
    };

    struct hw_cfg_t
    {
      uC::TIMERs::HW_Unit const tim;
      af_pin_t const            pin;
    };

  public:
    static constexpr hw_cfg_t TIM_3__A_06 = { uC::TIMERs::TIMER_3, { uC::GPIOs::A_06, 2 } };

    PWM_Timer(hw_cfg_t const& cfg)
        : m_tim_unit(cfg.tim)
        , m_gpio_unit(cfg.pin.pin,
                      uC::HANDLEs::GPIO_Handle_t::Speed::Very_High,
                      uC::HANDLEs::GPIO_Handle_t::Output_Mode::Push_Pull,
                      uC::HANDLEs::GPIO_Handle_t::Pull_Mode::No_Pull,
                      cfg.pin.af_val)
    {
      TIM_TypeDef& tim_base = this->m_tim_unit.get_base();
      // clang-format off
      tim_base.CNT = 0;
      tim_base.ARR = 9999;
      tim_base.CCR1 = 5000;
      tim_base.CCMR1 = ((               0b00 << TIM_CCMR1_CC1S_Pos) & TIM_CCMR1_CC1S_Msk)
                     | (( 0b0'0000'0000'0110 << TIM_CCMR1_OC1M_Pos) & TIM_CCMR1_OC1M_Msk);
      
      tim_base.CCER = TIM_CCER_CC1E;
      tim_base.CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
      // clang-format on
    }

    ~PWM_Timer() = default;

    float operator()(float const& ratio) override
    {
      TIM_TypeDef& tim_base = this->m_tim_unit.get_base();

      uint32_t const val = std::round(this->m_arr * ratio);

      tim_base.CCR1 = val;

      return val / this->m_arr;
    }

  private:
    uC::HANDLEs::BASIC_TIMER_Handle_t const m_tim_unit;
    uC::Alternative_Funktion_Pin const      m_gpio_unit;
    uint32_t const                          m_arr = 10'000;
  };
}    // namespace uC

#endif
