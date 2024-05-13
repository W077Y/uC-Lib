#pragma once
#ifndef UC_HW_UNITS_HPP
#define UC_HW_UNITS_HPP

#include <stm32h753xx.h>
#include <uC_Register.hpp>

extern uint32_t usart_1_clk;
extern uint32_t usart_2_clk;
extern uint32_t usart_3_clk;
extern uint32_t APB1_Timer_clk;
extern uint32_t APB2_Timer_clk;
extern uint32_t ADC_clk;

namespace uC
{
  namespace Internal
  {
    class dma_stream_irq_reason_t
    {
      static constexpr uint32_t flag_tc       = 0b0010'0000;
      static constexpr uint32_t flag_htc      = 0b0001'0000;
      static constexpr uint32_t flag_te       = 0b0000'1000;
      static constexpr uint32_t flag_dm_err   = 0b0000'0100;
      static constexpr uint32_t flag_fifo_err = 0b0000'0001;

    public:
      constexpr dma_stream_irq_reason_t(uint32_t const& flags)
          : m_flags(flags)
      {
      }

      constexpr bool is_transfer_complete() const { return is(dma_stream_irq_reason_t::flag_tc); }
      constexpr bool is_transfer_half_complete() const { return is(dma_stream_irq_reason_t::flag_htc); }
      constexpr bool is_transfer_error() const { return is(dma_stream_irq_reason_t::flag_te); }
      constexpr bool is_fifo_error() const { return is(dma_stream_irq_reason_t::flag_fifo_err); }
      constexpr bool is_direct_mode_error() const { return is(dma_stream_irq_reason_t::flag_dm_err); }

    private:
      constexpr bool is(uint32_t const& mask) const { return (this->m_flags & mask) != 0; }

      uint32_t m_flags = 0;
    };

    class CLK_BIT
    {
    public:
      constexpr CLK_BIT(uint32_t const& addr, uint32_t const& mask) noexcept
          : m_reg_address(addr)
          , m_bit_mask(mask)

      {
      }
      constexpr uC::register_bit_t get_clk_bit() const
      {
        return uC::register_bit_t(*reinterpret_cast<uint32_t volatile*>(this->m_reg_address), this->m_bit_mask);
      }

    private:
      uint32_t const m_reg_address;
      uint32_t const m_bit_mask;
    };

    class PORT_CFG
    {
    public:
      constexpr PORT_CFG(uint32_t const& addr, uint32_t const& port_no, CLK_BIT const& clk_bit) noexcept
          : m_base_address(addr)
          , m_port_number(port_no)
          , m_clk_bit(clk_bit)

      {
      }

      constexpr uC::register_bit_t get_clk_bit() const { return this->m_clk_bit.get_clk_bit(); }
      constexpr uint32_t           get_port_number() const { return this->m_port_number; }
      constexpr GPIO_TypeDef&      get_base() const { return *reinterpret_cast<GPIO_TypeDef*>(this->m_base_address); }

    private:
      uint32_t const m_base_address;
      uint32_t const m_port_number;
      CLK_BIT const  m_clk_bit;
    };

    class GPIO_CFG
    {
    public:
      constexpr GPIO_CFG(PORT_CFG const& port_cfg, uint32_t const& pin_no) noexcept
          : m_port(port_cfg)
          , m_mask(1 << pin_no)

      {
      }

      constexpr uC::register_bit_t get_clk_bit() const { return this->m_port.get_clk_bit(); }
      constexpr GPIO_TypeDef&      get_base() const { return this->m_port.get_base(); }
      constexpr uint32_t           get_pin_number() const
      {
        for (unsigned i = 0; i < 15; i++)
        {
          if (this->m_mask == (1UL << i))
            return i;
        }
        return 16;
      }
      constexpr uint32_t get_pin_mask() const { return this->m_mask; }
      constexpr uint32_t get_port_number() const { return this->m_port.get_port_number(); }

      static constexpr uint32_t max_number_of_ports        = 11;
      static constexpr uint32_t max_number_of_pin_per_port = 16;

    private:
      PORT_CFG const m_port;
      uint32_t const m_mask;
    };

    static constexpr PORT_CFG PORT_A = { GPIOA_BASE, 0, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOAEN } };
    static constexpr PORT_CFG PORT_B = { GPIOB_BASE, 1, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOBEN } };
    static constexpr PORT_CFG PORT_C = { GPIOC_BASE, 2, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOCEN } };
    static constexpr PORT_CFG PORT_D = { GPIOD_BASE, 3, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIODEN } };
    static constexpr PORT_CFG PORT_E = { GPIOE_BASE, 4, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOEEN } };
    static constexpr PORT_CFG PORT_F = { GPIOF_BASE, 5, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOFEN } };
    static constexpr PORT_CFG PORT_G = { GPIOG_BASE, 6, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOGEN } };
    static constexpr PORT_CFG PORT_H = { GPIOH_BASE, 7, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOHEN } };
    static constexpr PORT_CFG PORT_I = { GPIOI_BASE, 8, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOIEN } };
    static constexpr PORT_CFG PORT_J = { GPIOJ_BASE, 9, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOJEN } };
    static constexpr PORT_CFG PORT_K = { GPIOK_BASE, 10, { RCC_BASE + 0x0E0, RCC_AHB4ENR_GPIOKEN } };

    class DMA_CFG
    {
    public:
      constexpr DMA_CFG(uint32_t const& addr, uint32_t const& dma_no, CLK_BIT const& clk_bit) noexcept
          : m_base_address(addr)
          , m_mumber(dma_no)
          , m_clk_bit(clk_bit)
      {
      }

      constexpr uint32_t           get_number() const { return this->m_mumber; }
      constexpr DMA_TypeDef&       get_stream_base() const { return *reinterpret_cast<DMA_TypeDef*>(this->m_base_address); }
      constexpr uC::register_bit_t get_clk_bit() const { return this->m_clk_bit.get_clk_bit(); }

    private:
      uint32_t const m_base_address;
      uint32_t const m_mumber;
      CLK_BIT const  m_clk_bit;
    };

    class DMA_STREAM_CFG
    {
    public:
      constexpr DMA_STREAM_CFG(DMA_CFG const&   dma,
                               uint32_t const&  stream_base_addr,
                               uint32_t const&  stream_no,
                               uint32_t const&  mux_base_addr,
                               IRQn_Type const& irq_type) noexcept
          : m_dma(dma)
          , m_base_address(stream_base_addr)
          , m_mumber(stream_no)
          , m_mux_base_address(mux_base_addr)
          , m_irq_type(irq_type)
      {
      }

      constexpr uint32_t           get_dma_number() const { return this->m_dma.get_number(); }
      constexpr uC::register_bit_t get_clk_bit() const { return this->m_dma.get_clk_bit(); }

      constexpr uint32_t            get_stream_number() const { return this->m_mumber; }
      constexpr DMA_Stream_TypeDef& get_dma_stream_base() const { return *reinterpret_cast<DMA_Stream_TypeDef*>(this->m_base_address); }

      constexpr DMAMUX_Channel_TypeDef& get_mux_base() const { return *reinterpret_cast<DMAMUX_Channel_TypeDef*>(this->m_mux_base_address); }

      static constexpr uint32_t max_number_of_dmas            = 2;
      static constexpr uint32_t max_number_of_streams_per_dma = 8;

      constexpr IRQn_Type get_irq_type() const { return this->m_irq_type; }

    private:
      DMA_CFG const   m_dma;
      uint32_t const  m_base_address;
      uint32_t const  m_mumber;
      uint32_t const  m_mux_base_address;
      IRQn_Type const m_irq_type;
    };

    static constexpr DMA_CFG DMA_1 = { DMA1_BASE, 0, { RCC_BASE + 0x0D8, RCC_AHB1ENR_DMA1EN } };
    static constexpr DMA_CFG DMA_2 = { DMA2_BASE, 1, { RCC_BASE + 0x0D8, RCC_AHB1ENR_DMA2EN } };

    class USART_CFG
    {
    public:
      constexpr USART_CFG(uint32_t const& addr, uint32_t const& uart_no, CLK_BIT const& clk_bit, IRQn_Type const& irq_type, uint32_t& clk) noexcept
          : m_address(addr)
          , m_mumber(uart_no)
          , m_irq_type(irq_type)
          , m_clk_bit(clk_bit)
          , m_clk(clk)
      {
      }

      constexpr uint32_t           get_number() const { return this->m_mumber; }
      constexpr USART_TypeDef&     get_base() const { return *reinterpret_cast<USART_TypeDef*>(this->m_address); }
      constexpr uC::register_bit_t get_clk_bit() const { return this->m_clk_bit.get_clk_bit(); }
      constexpr IRQn_Type          get_irq_type() const { return this->m_irq_type; }

      constexpr uint32_t get_clk() const { return this->m_clk; }

      static constexpr uint32_t max_number_of_usarts = 3;

    private:
      uint32_t const  m_address;
      uint32_t const  m_mumber;
      IRQn_Type const m_irq_type;
      CLK_BIT const   m_clk_bit;
      uint32_t const& m_clk;
    };

    class TIMER_CFG
    {
    public:
      constexpr TIMER_CFG(uint32_t const& addr, uint32_t const& number, CLK_BIT const& clk_bit, IRQn_Type const& irq_type, uint32_t& clk) noexcept
          : m_address(addr)
          , m_mumber(number)
          , m_irq_type(irq_type)
          , m_clk_bit(clk_bit)
          , m_clk(clk)
      {
      }

      constexpr uint32_t           get_number() const { return this->m_mumber; }
      constexpr TIM_TypeDef&       get_base() const { return *reinterpret_cast<TIM_TypeDef*>(this->m_address); }
      constexpr uC::register_bit_t get_clk_bit() const { return this->m_clk_bit.get_clk_bit(); }
      constexpr IRQn_Type          get_irq_type() const { return this->m_irq_type; }

      constexpr uint32_t get_clk() const { return this->m_clk; }

      static constexpr uint32_t max_number_of_units = 17;

    private:
      uint32_t const  m_address;
      uint32_t const  m_mumber;
      IRQn_Type const m_irq_type;
      CLK_BIT const   m_clk_bit;
      uint32_t const& m_clk;
    };

    class DAC_CFG
    {
    public:
      static constexpr uint32_t max_number_of_units = 1;

      constexpr DAC_CFG(uint32_t const& addr, uint32_t const& number, CLK_BIT const& clk_bit) noexcept
          : m_address(addr)
          , m_mumber(number)
          , m_clk_bit(clk_bit)
      {
      }

      constexpr uint32_t           get_number() const { return this->m_mumber; }
      constexpr DAC_TypeDef&       get_base() const { return *reinterpret_cast<DAC_TypeDef*>(this->m_address); }
      constexpr uC::register_bit_t get_clk_bit() const { return this->m_clk_bit.get_clk_bit(); }

    private:
      uint32_t const m_address;
      uint32_t const m_mumber;
      CLK_BIT const  m_clk_bit;
    };

    class ADC_COMMON_BASE_CFG
    {
    public:
      constexpr ADC_COMMON_BASE_CFG(uint32_t const& addr, uint32_t const& number, CLK_BIT const& clk_bit) noexcept
          : m_address(addr)
          , m_mumber(number)
          , m_clk_bit(clk_bit)
      {
      }

      constexpr ADC_Common_TypeDef& get_base() const { return *reinterpret_cast<ADC_Common_TypeDef*>(this->m_address); }
      constexpr uC::register_bit_t  get_clk_bit() const { return this->m_clk_bit.get_clk_bit(); }
      constexpr uint32_t            get_number() const { return this->m_mumber; }

    private:
      uint32_t const m_address;
      uint32_t const m_mumber;
      CLK_BIT const  m_clk_bit;
    };

    static constexpr ADC_COMMON_BASE_CFG ADC_12_COMMON = { ADC12_COMMON_BASE, 0, { RCC_BASE + 0x0D8, RCC_AHB1ENR_ADC12EN } };
    static constexpr ADC_COMMON_BASE_CFG ADC_3_COMMON  = { ADC3_COMMON_BASE, 1, { RCC_BASE + 0x0E0, RCC_AHB4ENR_ADC3EN } };

    class ADC_CFG
    {
    public:
      static constexpr uint32_t max_number_of_units = 2;

      constexpr ADC_CFG(ADC_COMMON_BASE_CFG const& common, uint32_t const& addr, uint32_t const& adc_number, uint32_t& clk) noexcept
          : m_common(common)
          , m_address(addr)
          , m_mumber(adc_number)
          , m_clk(clk)
      {
      }

      constexpr uint32_t            get_blk_number() const { return this->m_common.get_number(); }
      constexpr uint32_t            get_adc_number() const { return this->m_mumber; }
      constexpr ADC_TypeDef&        get_base() const { return *reinterpret_cast<ADC_TypeDef*>(this->m_address); }
      constexpr ADC_Common_TypeDef& get_common_base() const { return this->m_common.get_base(); }
      constexpr uC::register_bit_t  get_clk_bit() const { return this->m_common.get_clk_bit(); }
      constexpr uint32_t            get_clk() const { return this->m_clk; }

    private:
      ADC_COMMON_BASE_CFG const m_common;
      uint32_t const            m_address;
      uint32_t const            m_mumber;
      uint32_t const&           m_clk;
    };
  }    // namespace Internal

  namespace GPIOs
  {
    using HW_Unit          = uC::Internal::GPIO_CFG;
    constexpr HW_Unit A_00 = { uC::Internal::PORT_A, 0 };
    constexpr HW_Unit A_01 = { uC::Internal::PORT_A, 1 };
    constexpr HW_Unit A_02 = { uC::Internal::PORT_A, 2 };
    constexpr HW_Unit A_03 = { uC::Internal::PORT_A, 3 };
    constexpr HW_Unit A_04 = { uC::Internal::PORT_A, 4 };
    constexpr HW_Unit A_05 = { uC::Internal::PORT_A, 5 };
    constexpr HW_Unit A_06 = { uC::Internal::PORT_A, 6 };
    constexpr HW_Unit A_07 = { uC::Internal::PORT_A, 7 };
    constexpr HW_Unit A_08 = { uC::Internal::PORT_A, 8 };
    constexpr HW_Unit A_09 = { uC::Internal::PORT_A, 9 };
    constexpr HW_Unit A_10 = { uC::Internal::PORT_A, 10 };
    constexpr HW_Unit A_11 = { uC::Internal::PORT_A, 11 };
    constexpr HW_Unit A_12 = { uC::Internal::PORT_A, 12 };
    constexpr HW_Unit A_13 = { uC::Internal::PORT_A, 13 };
    constexpr HW_Unit A_14 = { uC::Internal::PORT_A, 14 };
    constexpr HW_Unit A_15 = { uC::Internal::PORT_A, 15 };

    constexpr HW_Unit B_00 = { uC::Internal::PORT_B, 0 };
    constexpr HW_Unit B_01 = { uC::Internal::PORT_B, 1 };
    constexpr HW_Unit B_02 = { uC::Internal::PORT_B, 2 };
    constexpr HW_Unit B_03 = { uC::Internal::PORT_B, 3 };
    constexpr HW_Unit B_04 = { uC::Internal::PORT_B, 4 };
    constexpr HW_Unit B_05 = { uC::Internal::PORT_B, 5 };
    constexpr HW_Unit B_06 = { uC::Internal::PORT_B, 6 };
    constexpr HW_Unit B_07 = { uC::Internal::PORT_B, 7 };
    constexpr HW_Unit B_08 = { uC::Internal::PORT_B, 8 };
    constexpr HW_Unit B_09 = { uC::Internal::PORT_B, 9 };
    constexpr HW_Unit B_10 = { uC::Internal::PORT_B, 10 };
    constexpr HW_Unit B_11 = { uC::Internal::PORT_B, 11 };
    constexpr HW_Unit B_12 = { uC::Internal::PORT_B, 12 };
    constexpr HW_Unit B_13 = { uC::Internal::PORT_B, 13 };
    constexpr HW_Unit B_14 = { uC::Internal::PORT_B, 14 };
    constexpr HW_Unit B_15 = { uC::Internal::PORT_B, 15 };

    constexpr HW_Unit C_00 = { uC::Internal::PORT_C, 0 };
    constexpr HW_Unit C_01 = { uC::Internal::PORT_C, 1 };
    constexpr HW_Unit C_02 = { uC::Internal::PORT_C, 2 };
    constexpr HW_Unit C_03 = { uC::Internal::PORT_C, 3 };
    constexpr HW_Unit C_04 = { uC::Internal::PORT_C, 4 };
    constexpr HW_Unit C_05 = { uC::Internal::PORT_C, 5 };
    constexpr HW_Unit C_06 = { uC::Internal::PORT_C, 6 };
    constexpr HW_Unit C_07 = { uC::Internal::PORT_C, 7 };
    constexpr HW_Unit C_08 = { uC::Internal::PORT_C, 8 };
    constexpr HW_Unit C_09 = { uC::Internal::PORT_C, 9 };
    constexpr HW_Unit C_10 = { uC::Internal::PORT_C, 10 };
    constexpr HW_Unit C_11 = { uC::Internal::PORT_C, 11 };
    constexpr HW_Unit C_12 = { uC::Internal::PORT_C, 12 };
    constexpr HW_Unit C_13 = { uC::Internal::PORT_C, 13 };
    constexpr HW_Unit C_14 = { uC::Internal::PORT_C, 14 };
    constexpr HW_Unit C_15 = { uC::Internal::PORT_C, 15 };

    constexpr HW_Unit D_00 = { uC::Internal::PORT_D, 0 };
    constexpr HW_Unit D_01 = { uC::Internal::PORT_D, 1 };
    constexpr HW_Unit D_02 = { uC::Internal::PORT_D, 2 };
    constexpr HW_Unit D_03 = { uC::Internal::PORT_D, 3 };
    constexpr HW_Unit D_04 = { uC::Internal::PORT_D, 4 };
    constexpr HW_Unit D_05 = { uC::Internal::PORT_D, 5 };
    constexpr HW_Unit D_06 = { uC::Internal::PORT_D, 6 };
    constexpr HW_Unit D_07 = { uC::Internal::PORT_D, 7 };
    constexpr HW_Unit D_08 = { uC::Internal::PORT_D, 8 };
    constexpr HW_Unit D_09 = { uC::Internal::PORT_D, 9 };
    constexpr HW_Unit D_10 = { uC::Internal::PORT_D, 10 };
    constexpr HW_Unit D_11 = { uC::Internal::PORT_D, 11 };
    constexpr HW_Unit D_12 = { uC::Internal::PORT_D, 12 };
    constexpr HW_Unit D_13 = { uC::Internal::PORT_D, 13 };
    constexpr HW_Unit D_14 = { uC::Internal::PORT_D, 14 };
    constexpr HW_Unit D_15 = { uC::Internal::PORT_D, 15 };

    constexpr HW_Unit E_00 = { uC::Internal::PORT_E, 0 };
    constexpr HW_Unit E_01 = { uC::Internal::PORT_E, 1 };
    constexpr HW_Unit E_02 = { uC::Internal::PORT_E, 2 };
    constexpr HW_Unit E_03 = { uC::Internal::PORT_E, 3 };
    constexpr HW_Unit E_04 = { uC::Internal::PORT_E, 4 };
    constexpr HW_Unit E_05 = { uC::Internal::PORT_E, 5 };
    constexpr HW_Unit E_06 = { uC::Internal::PORT_E, 6 };
    constexpr HW_Unit E_07 = { uC::Internal::PORT_E, 7 };
    constexpr HW_Unit E_08 = { uC::Internal::PORT_E, 8 };
    constexpr HW_Unit E_09 = { uC::Internal::PORT_E, 9 };
    constexpr HW_Unit E_10 = { uC::Internal::PORT_E, 10 };
    constexpr HW_Unit E_11 = { uC::Internal::PORT_E, 11 };
    constexpr HW_Unit E_12 = { uC::Internal::PORT_E, 12 };
    constexpr HW_Unit E_13 = { uC::Internal::PORT_E, 13 };
    constexpr HW_Unit E_14 = { uC::Internal::PORT_E, 14 };
    constexpr HW_Unit E_15 = { uC::Internal::PORT_E, 15 };

    constexpr HW_Unit F_00 = { uC::Internal::PORT_F, 0 };
    constexpr HW_Unit F_01 = { uC::Internal::PORT_F, 1 };
    constexpr HW_Unit F_02 = { uC::Internal::PORT_F, 2 };
    constexpr HW_Unit F_03 = { uC::Internal::PORT_F, 3 };
    constexpr HW_Unit F_04 = { uC::Internal::PORT_F, 4 };
    constexpr HW_Unit F_05 = { uC::Internal::PORT_F, 5 };
    constexpr HW_Unit F_06 = { uC::Internal::PORT_F, 6 };
    constexpr HW_Unit F_07 = { uC::Internal::PORT_F, 7 };
    constexpr HW_Unit F_08 = { uC::Internal::PORT_F, 8 };
    constexpr HW_Unit F_09 = { uC::Internal::PORT_F, 9 };
    constexpr HW_Unit F_10 = { uC::Internal::PORT_F, 10 };
    constexpr HW_Unit F_11 = { uC::Internal::PORT_F, 11 };
    constexpr HW_Unit F_12 = { uC::Internal::PORT_F, 12 };
    constexpr HW_Unit F_13 = { uC::Internal::PORT_F, 13 };
    constexpr HW_Unit F_14 = { uC::Internal::PORT_F, 14 };
    constexpr HW_Unit F_15 = { uC::Internal::PORT_F, 15 };

    constexpr HW_Unit G_00 = { uC::Internal::PORT_G, 0 };
    constexpr HW_Unit G_01 = { uC::Internal::PORT_G, 1 };
    constexpr HW_Unit G_02 = { uC::Internal::PORT_G, 2 };
    constexpr HW_Unit G_03 = { uC::Internal::PORT_G, 3 };
    constexpr HW_Unit G_04 = { uC::Internal::PORT_G, 4 };
    constexpr HW_Unit G_05 = { uC::Internal::PORT_G, 5 };
    constexpr HW_Unit G_06 = { uC::Internal::PORT_G, 6 };
    constexpr HW_Unit G_07 = { uC::Internal::PORT_G, 7 };
    constexpr HW_Unit G_08 = { uC::Internal::PORT_G, 8 };
    constexpr HW_Unit G_09 = { uC::Internal::PORT_G, 9 };
    constexpr HW_Unit G_10 = { uC::Internal::PORT_G, 10 };
    constexpr HW_Unit G_11 = { uC::Internal::PORT_G, 11 };
    constexpr HW_Unit G_12 = { uC::Internal::PORT_G, 12 };
    constexpr HW_Unit G_13 = { uC::Internal::PORT_G, 13 };
    constexpr HW_Unit G_14 = { uC::Internal::PORT_G, 14 };
    constexpr HW_Unit G_15 = { uC::Internal::PORT_G, 15 };

    constexpr HW_Unit H_00 = { uC::Internal::PORT_H, 0 };
    constexpr HW_Unit H_01 = { uC::Internal::PORT_H, 1 };
    constexpr HW_Unit H_02 = { uC::Internal::PORT_H, 2 };
    constexpr HW_Unit H_03 = { uC::Internal::PORT_H, 3 };
    constexpr HW_Unit H_04 = { uC::Internal::PORT_H, 4 };
    constexpr HW_Unit H_05 = { uC::Internal::PORT_H, 5 };
    constexpr HW_Unit H_06 = { uC::Internal::PORT_H, 6 };
    constexpr HW_Unit H_07 = { uC::Internal::PORT_H, 7 };
    constexpr HW_Unit H_08 = { uC::Internal::PORT_H, 8 };
    constexpr HW_Unit H_09 = { uC::Internal::PORT_H, 9 };
    constexpr HW_Unit H_10 = { uC::Internal::PORT_H, 10 };
    constexpr HW_Unit H_11 = { uC::Internal::PORT_H, 11 };
    constexpr HW_Unit H_12 = { uC::Internal::PORT_H, 12 };
    constexpr HW_Unit H_13 = { uC::Internal::PORT_H, 13 };
    constexpr HW_Unit H_14 = { uC::Internal::PORT_H, 14 };
    constexpr HW_Unit H_15 = { uC::Internal::PORT_H, 15 };

    constexpr HW_Unit I_00 = { uC::Internal::PORT_I, 0 };
    constexpr HW_Unit I_01 = { uC::Internal::PORT_I, 1 };
    constexpr HW_Unit I_02 = { uC::Internal::PORT_I, 2 };
    constexpr HW_Unit I_03 = { uC::Internal::PORT_I, 3 };
    constexpr HW_Unit I_04 = { uC::Internal::PORT_I, 4 };
    constexpr HW_Unit I_05 = { uC::Internal::PORT_I, 5 };
    constexpr HW_Unit I_06 = { uC::Internal::PORT_I, 6 };
    constexpr HW_Unit I_07 = { uC::Internal::PORT_I, 7 };
    constexpr HW_Unit I_08 = { uC::Internal::PORT_I, 8 };
    constexpr HW_Unit I_09 = { uC::Internal::PORT_I, 9 };
    constexpr HW_Unit I_10 = { uC::Internal::PORT_I, 10 };
    constexpr HW_Unit I_11 = { uC::Internal::PORT_I, 11 };
    constexpr HW_Unit I_12 = { uC::Internal::PORT_I, 12 };
    constexpr HW_Unit I_13 = { uC::Internal::PORT_I, 13 };
    constexpr HW_Unit I_14 = { uC::Internal::PORT_I, 14 };
    constexpr HW_Unit I_15 = { uC::Internal::PORT_I, 15 };

    constexpr HW_Unit J_00 = { uC::Internal::PORT_J, 0 };
    constexpr HW_Unit J_01 = { uC::Internal::PORT_J, 1 };
    constexpr HW_Unit J_02 = { uC::Internal::PORT_J, 2 };
    constexpr HW_Unit J_03 = { uC::Internal::PORT_J, 3 };
    constexpr HW_Unit J_04 = { uC::Internal::PORT_J, 4 };
    constexpr HW_Unit J_05 = { uC::Internal::PORT_J, 5 };
    constexpr HW_Unit J_06 = { uC::Internal::PORT_J, 6 };
    constexpr HW_Unit J_07 = { uC::Internal::PORT_J, 7 };
    constexpr HW_Unit J_08 = { uC::Internal::PORT_J, 8 };
    constexpr HW_Unit J_09 = { uC::Internal::PORT_J, 9 };
    constexpr HW_Unit J_10 = { uC::Internal::PORT_J, 10 };
    constexpr HW_Unit J_11 = { uC::Internal::PORT_J, 11 };
    constexpr HW_Unit J_12 = { uC::Internal::PORT_J, 12 };
    constexpr HW_Unit J_13 = { uC::Internal::PORT_J, 13 };
    constexpr HW_Unit J_14 = { uC::Internal::PORT_J, 14 };
    constexpr HW_Unit J_15 = { uC::Internal::PORT_J, 15 };

    constexpr HW_Unit K_00 = { uC::Internal::PORT_K, 0 };
    constexpr HW_Unit K_01 = { uC::Internal::PORT_K, 1 };
    constexpr HW_Unit K_02 = { uC::Internal::PORT_K, 2 };
    constexpr HW_Unit K_03 = { uC::Internal::PORT_K, 3 };
    constexpr HW_Unit K_04 = { uC::Internal::PORT_K, 4 };
    constexpr HW_Unit K_05 = { uC::Internal::PORT_K, 5 };
    constexpr HW_Unit K_06 = { uC::Internal::PORT_K, 6 };
    constexpr HW_Unit K_07 = { uC::Internal::PORT_K, 7 };
    constexpr HW_Unit K_08 = { uC::Internal::PORT_K, 8 };
    constexpr HW_Unit K_09 = { uC::Internal::PORT_K, 9 };
    constexpr HW_Unit K_10 = { uC::Internal::PORT_K, 10 };
    constexpr HW_Unit K_11 = { uC::Internal::PORT_K, 11 };
    constexpr HW_Unit K_12 = { uC::Internal::PORT_K, 12 };
    constexpr HW_Unit K_13 = { uC::Internal::PORT_K, 13 };
    constexpr HW_Unit K_14 = { uC::Internal::PORT_K, 14 };
    constexpr HW_Unit K_15 = { uC::Internal::PORT_K, 15 };
  }    // namespace GPIOs

  namespace DMA_Streams
  {
    using HW_Unit                    = uC::Internal::DMA_STREAM_CFG;
    constexpr HW_Unit DMA_1_Stream_0 = { Internal::DMA_1, DMA1_Stream0_BASE, 0, DMAMUX1_Channel0_BASE, IRQn_Type::DMA1_Stream0_IRQn };
    constexpr HW_Unit DMA_1_Stream_1 = { Internal::DMA_1, DMA1_Stream1_BASE, 1, DMAMUX1_Channel1_BASE, IRQn_Type::DMA1_Stream1_IRQn };
    constexpr HW_Unit DMA_1_Stream_2 = { Internal::DMA_1, DMA1_Stream2_BASE, 2, DMAMUX1_Channel2_BASE, IRQn_Type::DMA1_Stream2_IRQn };
    constexpr HW_Unit DMA_1_Stream_3 = { Internal::DMA_1, DMA1_Stream3_BASE, 3, DMAMUX1_Channel3_BASE, IRQn_Type::DMA1_Stream3_IRQn };
    constexpr HW_Unit DMA_1_Stream_4 = { Internal::DMA_1, DMA1_Stream4_BASE, 4, DMAMUX1_Channel4_BASE, IRQn_Type::DMA1_Stream4_IRQn };
    constexpr HW_Unit DMA_1_Stream_5 = { Internal::DMA_1, DMA1_Stream5_BASE, 5, DMAMUX1_Channel5_BASE, IRQn_Type::DMA1_Stream5_IRQn };
    constexpr HW_Unit DMA_1_Stream_6 = { Internal::DMA_1, DMA1_Stream6_BASE, 6, DMAMUX1_Channel6_BASE, IRQn_Type::DMA1_Stream6_IRQn };
    constexpr HW_Unit DMA_1_Stream_7 = { Internal::DMA_1, DMA1_Stream7_BASE, 7, DMAMUX1_Channel7_BASE, IRQn_Type::DMA1_Stream7_IRQn };

    constexpr HW_Unit DMA_2_Stream_0 = { Internal::DMA_2, DMA2_Stream0_BASE, 0, DMAMUX1_Channel8_BASE, IRQn_Type::DMA2_Stream0_IRQn };
    constexpr HW_Unit DMA_2_Stream_1 = { Internal::DMA_2, DMA2_Stream1_BASE, 1, DMAMUX1_Channel9_BASE, IRQn_Type::DMA2_Stream1_IRQn };
    constexpr HW_Unit DMA_2_Stream_2 = { Internal::DMA_2, DMA2_Stream2_BASE, 2, DMAMUX1_Channel10_BASE, IRQn_Type::DMA2_Stream2_IRQn };
    constexpr HW_Unit DMA_2_Stream_3 = { Internal::DMA_2, DMA2_Stream3_BASE, 3, DMAMUX1_Channel11_BASE, IRQn_Type::DMA2_Stream3_IRQn };
    constexpr HW_Unit DMA_2_Stream_4 = { Internal::DMA_2, DMA2_Stream4_BASE, 4, DMAMUX1_Channel12_BASE, IRQn_Type::DMA2_Stream4_IRQn };
    constexpr HW_Unit DMA_2_Stream_5 = { Internal::DMA_2, DMA2_Stream5_BASE, 5, DMAMUX1_Channel13_BASE, IRQn_Type::DMA2_Stream5_IRQn };
    constexpr HW_Unit DMA_2_Stream_6 = { Internal::DMA_2, DMA2_Stream6_BASE, 6, DMAMUX1_Channel14_BASE, IRQn_Type::DMA2_Stream6_IRQn };
    constexpr HW_Unit DMA_2_Stream_7 = { Internal::DMA_2, DMA2_Stream7_BASE, 7, DMAMUX1_Channel15_BASE, IRQn_Type::DMA2_Stream7_IRQn };
  }    // namespace DMA_Streams

  namespace USARTs
  {
    using HW_Unit             = uC::Internal::USART_CFG;
    constexpr HW_Unit USART_1 = { USART1_BASE, 0, { RCC_BASE + 0x0F0, RCC_APB2ENR_USART1EN }, IRQn_Type::USART1_IRQn, usart_1_clk };
    constexpr HW_Unit USART_2 = { USART2_BASE, 1, { RCC_BASE + 0x0E8, RCC_APB1LENR_USART2EN }, IRQn_Type::USART2_IRQn, usart_2_clk };
    constexpr HW_Unit USART_3 = { USART3_BASE, 2, { RCC_BASE + 0x0E8, RCC_APB1LENR_USART3EN }, IRQn_Type::USART3_IRQn, usart_3_clk };

  }    // namespace USARTs

  namespace TIMERs
  {
    using HW_Unit             = uC::Internal::TIMER_CFG;
    constexpr HW_Unit TIMER_1 = { TIM1_BASE, 0, { RCC_BASE + 0x0F0, RCC_APB2ENR_TIM1EN }, IRQn_Type::TIM1_UP_IRQn, APB2_Timer_clk };
    constexpr HW_Unit TIMER_2 = { TIM2_BASE, 1, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM2EN }, IRQn_Type::TIM2_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_3 = { TIM3_BASE, 2, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM3EN }, IRQn_Type::TIM3_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_4 = { TIM4_BASE, 3, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM4EN }, IRQn_Type::TIM4_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_5 = { TIM5_BASE, 4, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM5EN }, IRQn_Type::TIM5_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_6 = { TIM6_BASE, 5, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM6EN }, IRQn_Type::TIM6_DAC_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_7 = { TIM7_BASE, 6, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM7EN }, IRQn_Type::TIM7_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_8 = { TIM8_BASE, 7, { RCC_BASE + 0x0F0, RCC_APB2ENR_TIM8EN }, IRQn_Type::TIM8_UP_TIM13_IRQn, APB2_Timer_clk };

    constexpr HW_Unit TIMER_12 = { TIM12_BASE, 11, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM12EN }, IRQn_Type::TIM8_BRK_TIM12_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_13 = { TIM13_BASE, 12, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM13EN }, IRQn_Type::TIM8_UP_TIM13_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_14 = { TIM14_BASE, 13, { RCC_BASE + 0x0E8, RCC_APB1LENR_TIM14EN }, IRQn_Type::TIM8_TRG_COM_TIM14_IRQn, APB1_Timer_clk };
    constexpr HW_Unit TIMER_15 = { TIM15_BASE, 14, { RCC_BASE + 0x0F0, RCC_APB2ENR_TIM15EN }, IRQn_Type::TIM15_IRQn, APB2_Timer_clk };
    constexpr HW_Unit TIMER_16 = { TIM16_BASE, 15, { RCC_BASE + 0x0F0, RCC_APB2ENR_TIM16EN }, IRQn_Type::TIM16_IRQn, APB2_Timer_clk };
    constexpr HW_Unit TIMER_17 = { TIM17_BASE, 16, { RCC_BASE + 0x0F0, RCC_APB2ENR_TIM17EN }, IRQn_Type::TIM17_IRQn, APB2_Timer_clk };
  }    // namespace TIMERs

  namespace DACs
  {
    using HW_Unit           = uC::Internal::DAC_CFG;
    constexpr HW_Unit DAC_1 = { DAC1_BASE, 0, { RCC_BASE + 0x0E8, RCC_APB1LENR_DAC12EN } };
  }    // namespace DACs

  namespace ADCs
  {
    using HW_Unit           = uC::Internal::ADC_CFG;
    constexpr HW_Unit ADC_1 = { uC::Internal::ADC_12_COMMON, ADC1_BASE, 0, ADC_clk };
    constexpr HW_Unit ADC_2 = { uC::Internal::ADC_12_COMMON, ADC2_BASE, 1, ADC_clk };
    constexpr HW_Unit ADC_3 = { uC::Internal::ADC_3_COMMON, ADC3_BASE, 3, ADC_clk };
  }    // namespace ADCs
}    // namespace uC

#endif
