#pragma once
#ifndef UC_F_GEN_HPP
#define UC_F_GEN_HPP

#include <Ex_Math.hpp>
#include <WLib_Fgen_Interface.hpp>
#include <WLib_PIN_Abstraction.hpp>
#include <atomic>
#include <cmath>
#include <uC_DAC.hpp>
#include <uC_DMA.hpp>
#include <uC_Errors.hpp>
#include <uC_GPIO.hpp>
#include <uC_HW_Manager.hpp>
#include <uC_HW_Units.hpp>
#include <uC_IRQ_Manager.hpp>
#include <uC_Timer.hpp>

namespace BSP
{
  WLib::PIN::Digital_Output_Interface& get_output_pin_2();
}

namespace uC
{
  class F_Gen: public WLib::FrequenzGenerator_Interface
  {
    using this_t       = F_Gen;
    using irq_reason_t = uC::HANDLEs::DMA_Stream_Handle_t::irq_reason_t;

    struct hw_cfg_t
    {
      uC::DACs::HW_Unit const   dac;
      uC::TIMERs::HW_Unit const tim;
      uC::GPIOs::HW_Unit const  pin;
      uint32_t const            mux_val;
    };

  public:
    static constexpr hw_cfg_t DAC_CH_1__TIM_6__A_04 = { uC::DACs::DAC_1, uC::TIMERs::TIMER_6, uC::GPIOs::A_04, 67 };

    F_Gen(hw_cfg_t const& cfg, uC::DMA_Streams::HW_Unit const& dma_unit)
        : m_pin_unit(cfg.pin)
        , m_tim_unit(cfg.tim)
        , m_dma_unit(dma_unit)
        , m_dac_unit(cfg.dac)
    {
      DMA_Stream_TypeDef& dma_base = this->m_dma_unit.get_base();
      DAC_TypeDef&        dac_base = this->m_dac_unit.get_base();
      TIM_TypeDef&        tim_base = this->m_tim_unit.get_base();

      this->m_buffer_a = reinterpret_cast<uint32_t*>(BSP::get_dma_buffer_allocator().allocate<uint32_t>(this->m_buffer_len).data());
      this->update_buffer_a();

      this->m_buffer_b = reinterpret_cast<uint32_t*>(BSP::get_dma_buffer_allocator().allocate<uint32_t>(this->m_buffer_len).data());
      this->update_buffer_b();

      this->m_dma_unit.register_irq(this->m_rx_irq_cb, 1);

      // clang-format off
      dma_base.PAR  = reinterpret_cast<uint32_t>(&dac_base.DHR12R1);
      dma_base.M0AR = reinterpret_cast<uint32_t>(&this->m_buffer_a[0]);
      dma_base.M1AR = reinterpret_cast<uint32_t>(&this->m_buffer_b[0]);
      dma_base.NDTR = this->m_buffer_len;
      dma_base.CR   = (DMA_SxCR_MBURST & (0b00  << 23))
                    | (DMA_SxCR_PBURST & (0b00  << 21))
                    | (DMA_SxCR_CT     & (0b0   << 19))
                    | (DMA_SxCR_DBM    & (0b1   << 18))
                    | (DMA_SxCR_PL     & (0b00  << 16))
                    | (DMA_SxCR_PINCOS & (0b0   << 15))
                    | (DMA_SxCR_MSIZE  & (0b10  << 13))
                    | (DMA_SxCR_PSIZE  & (0b10  << 11))
                    | (DMA_SxCR_MINC   & (0b1   << 10))
                    | (DMA_SxCR_PINC   & (0b0   << 9))
                    | (DMA_SxCR_CIRC   & (0b1   << 8))
                    | (DMA_SxCR_DIR    & (0b01  << 6))
                    | (DMA_SxCR_PFCTRL & (0b0   << 5))
                    | (DMA_SxCR_TCIE   & (0b1   << 4))
                    | (DMA_SxCR_HTIE   & (0b0   << 3))
                    | (DMA_SxCR_TEIE   & (0b1   << 2))
                    | (DMA_SxCR_DMEIE  & (0b1   << 1))
                    | (DMA_SxCR_EN     & (0b1   << 0));
      
      this->m_dma_unit.get_mux_base().CCR = (( cfg.mux_val << DMAMUX_CxCR_DMAREQ_ID_Pos) & DMAMUX_CxCR_DMAREQ_ID_Msk);

      dac_base.DHR12RD = 2048;
      dac_base.CR = ((       0b0 << DAC_CR_DMAUDRIE1_Pos) & DAC_CR_DMAUDRIE1_Msk) 
                  | ((       0b1 << DAC_CR_DMAEN1_Pos)    & DAC_CR_DMAEN1_Msk) 
                  | ((    0b1011 << DAC_CR_MAMP1_Pos)     & DAC_CR_MAMP1_Msk) 
                  | ((      0b00 << DAC_CR_WAVE1_Pos)     & DAC_CR_WAVE1_Msk)
                  | ((         5 << DAC_CR_TSEL1_Pos)     & DAC_CR_TSEL1_Msk)
                  | ((       0b1 << DAC_CR_TEN1_Pos)      & DAC_CR_TEN1_Msk)
                  | ((       0b1 << DAC_CR_EN1_Pos)       & DAC_CR_EN1_Msk);

      tim_base.CNT = 0;
      tim_base.ARR = this->m_arr-1;
      tim_base.CR2 = ((0b010 << TIM_CR2_MMS_Pos) & TIM_CR2_MMS_Msk);
      tim_base.CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
      // clang-format on
    }

    ~F_Gen() = default;

    void set_signal(float const& amp, float const& freq_1_min, float const& freq_1_max) override
    {
      this->m_target_amp        = amp;
      this->m_target_freq_1_min = freq_1_min;
      this->m_target_freq_1_max = freq_1_max;
    }

  private:
    void irq_handler(irq_reason_t const& reason)
    {
      BSP::get_output_pin_2().set_high();
      DMA_Stream_TypeDef& dma_base = this->m_dma_unit.get_base();
      if (reason.is_transfer_complete())
      {
        if ((dma_base.CR & DMA_SxCR_CT) != 0)
          this->update_buffer_a();
        else
          this->update_buffer_b();
      }
      BSP::get_output_pin_2().set_low();
    }

    void update_buffer_a() { this->update_buffer(this->m_buffer_a); }
    void update_buffer_b() { this->update_buffer(this->m_buffer_b); }

    void update_buffer(uint32_t* buf)
    {
      float const amp = this->m_target_amp;
      this->m_focus_cirp1.set(this->m_target_freq_1_min, this->m_target_freq_1_max);
      this->m_sine1.set((this->m_target_freq_1_min + this->m_target_freq_1_max) / 2.0);

      this->fill_buffer(buf, this->m_focus_cirp1, amp, this->m_sine1);
      SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(buf), this->m_buffer_len * sizeof(uint32_t));
    }

    class chirp_sin_t
    {
    public:
      constexpr chirp_sin_t(float const& delta_freq, float const& freq_min, float const& freq_max)
          : m_f_min(freq_min)
          , m_f_max(freq_max)
          , m_delta_freq(delta_freq)
          , m_freq(freq_min)
      {
      }

      constexpr void set(float const& f_min, float const& f_max)
      {
        this->m_f_min = f_min;
        this->m_f_max = f_max;
      }
      constexpr float operator()()
      {
        float const ret = std::sin(this->m_phi);
        ++(*this);
        return ret;
      }

    private:
      constexpr chirp_sin_t& operator++()
      {
        constexpr float phi_scale = 2.0f * pi / sample_freq;

        this->m_phi += phi_scale * this->m_freq;
        if (this->m_phi > pi)
          this->m_phi -= 2 * pi;

        this->m_freq += this->m_delta_freq;
        if (this->m_freq >= this->m_f_max)
        {
          this->m_freq -= (this->m_f_max - this->m_f_min);
        }

        return *this;
      }

      float m_f_min;
      float m_f_max;
      float m_delta_freq;
      float m_freq;
      float m_phi = 0.0f;
    };

    class sin_t
    {
    public:
      constexpr sin_t(float const& freq)
          : m_freq(freq)
      {
      }

      constexpr void  set(float const& freq) { this->m_freq = freq; }
      constexpr float operator()()
      {
        float const ret = std::sin(this->m_phi);
        ++(*this);
        return ret;
      }

    private:
      constexpr sin_t& operator++()
      {
        constexpr float phi_scale = 2.0f * pi / sample_freq;

        this->m_phi += phi_scale * this->m_freq;
        if (this->m_phi > pi)
          this->m_phi -= 2 * pi;

        return *this;
      }

      float m_freq;
      float m_phi = 0.0f;
    };

    static constexpr int32_t dac_max_value = 0xFFF;
    static constexpr int32_t dac_min_value = 0x000;
    static constexpr float   sample_freq   = 50'000.0f;
    static constexpr float   pi            = Ex_Math::Constants::pi<float>;

    void fill_buffer(uint32_t* buf, chirp_sin_t& chirp_1, float const& amp_sin_1, sin_t& sin_1)
    {
      constexpr float lam = 0.25f;
      for (uint32_t idx = 0; idx < this->m_buffer_len; ++idx)
      {
        float const val = 0.5f + 0.5f * (lam * chirp_1() + (1 - lam) * amp_sin_1 * sin_1());

        int32_t const tmp = static_cast<int32_t>(std::round(val * dac_max_value));
        if (tmp > dac_max_value)
          buf[idx] = dac_max_value;
        else if (tmp < dac_min_value)
          buf[idx] = dac_min_value;
        else
          buf[idx] = tmp;
      }
    }

    uC::Analog_Pin const                                             m_pin_unit;
    uC::HANDLEs::BASIC_TIMER_Handle_t const                          m_tim_unit;
    uC::HANDLEs::DMA_Stream_Handle_t const                           m_dma_unit;
    uC::HANDLEs::DAC_Handle_t const                                  m_dac_unit;
    WLib::Memberfunction_Callback<this_t, void(irq_reason_t const&)> m_rx_irq_cb{ *this, &this_t::irq_handler };
    uint32_t const                                                   m_arr               = 4'800;
    uint32_t const                                                   m_buffer_len        = 512;
    uint32_t*                                                        m_buffer_a          = nullptr;
    uint32_t*                                                        m_buffer_b          = nullptr;
    std::atomic<float>                                               m_target_amp        = 0.0f;
    std::atomic<float>                                               m_target_freq_1_min = 3'000.0f;
    std::atomic<float>                                               m_target_freq_1_max = 5'000.0f;
    chirp_sin_t                                                      m_focus_cirp1{ 5000.0f / sample_freq, m_target_freq_1_min, m_target_freq_1_max };
    sin_t                                                            m_sine1{ (m_target_freq_1_min + m_target_freq_1_max) / 2.0f };
  };

}    // namespace uC

#endif
