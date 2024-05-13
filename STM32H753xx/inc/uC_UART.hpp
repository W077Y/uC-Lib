#pragma once
#ifndef UC_UART_HPP
#define UC_UART_HPP

#include <WLib_CString_Sink_Interface.hpp>
#include <WLib_Callback_Interface.hpp>
#include <WLib_LockFree_Puplisher.hpp>
#include <WLib_MPSC.hpp>
#include <WLib_Publisher_Interface.hpp>
#include <cstdint>
#include <cstring>
#include <uC_DMA.hpp>
#include <uC_Errors.hpp>
#include <uC_GPIO.hpp>
#include <uC_HW_Handles.hpp>

namespace uC
{
  class UART__TX_DMA__RX_IRQ final
      : public WLib::Publisher_Interface<char>
      , public WLib::CString_Sink_Interface
  {
    using this_t       = UART__TX_DMA__RX_IRQ;
    using irq_reason_t = uC::HANDLEs::DMA_Stream_Handle_t::irq_reason_t;
    struct af_pin_t
    {
      uC::GPIOs::HW_Unit const pin;
      uint32_t const           af_val;
    };

    struct hw_cfg_t
    {
      uC::USARTs::HW_Unit const uart_name;
      af_pin_t const            tx;
      af_pin_t const            rx;
      uint32_t const            mux_val;
    };

  public:
    using payload_t            = typename Publisher_Interface<char>::payload_t;
    using mem_payload_t        = WLib::Container::mpsc_queue_ex_mem<char>::mem_payload_t;
    using Subscriber_Interface = typename Publisher_Interface<char>::Subscriber_Interface;

    static constexpr hw_cfg_t UART_1__TX_A_09__RX__A_10{ uC::USARTs::USART_1, { uC::GPIOs::A_09, 7 }, { uC::GPIOs::A_10, 7 }, 42 };
    static constexpr hw_cfg_t UART_1__TX_A_09__RX__B_07{ uC::USARTs::USART_1, { uC::GPIOs::A_09, 7 }, { uC::GPIOs::B_07, 7 }, 42 };
    static constexpr hw_cfg_t UART_1__TX_B_06__RX__A_10{ uC::USARTs::USART_1, { uC::GPIOs::B_06, 7 }, { uC::GPIOs::A_10, 7 }, 42 };
    static constexpr hw_cfg_t UART_1__TX_B_06__RX__B_07{ uC::USARTs::USART_1, { uC::GPIOs::B_06, 7 }, { uC::GPIOs::B_07, 7 }, 42 };

    static constexpr hw_cfg_t UART_2__TX_A_02__RX__A_03{ uC::USARTs::USART_2, { uC::GPIOs::A_02, 7 }, { uC::GPIOs::A_03, 7 }, 44 };
    static constexpr hw_cfg_t UART_2__TX_A_02__RX__D_06{ uC::USARTs::USART_2, { uC::GPIOs::A_02, 7 }, { uC::GPIOs::D_06, 7 }, 44 };
    static constexpr hw_cfg_t UART_2__TX_D_05__RX__A_03{ uC::USARTs::USART_2, { uC::GPIOs::D_05, 7 }, { uC::GPIOs::A_03, 7 }, 44 };
    static constexpr hw_cfg_t UART_2__TX_D_05__RX__D_06{ uC::USARTs::USART_2, { uC::GPIOs::D_05, 7 }, { uC::GPIOs::D_06, 7 }, 44 };

    UART__TX_DMA__RX_IRQ(hw_cfg_t const& cfg, uC::DMA_Streams::HW_Unit const& dma_stream_name, std::size_t const& buffer_size)
        : m_uart_handle(cfg.uart_name)
        , m_tx_pin(cfg.tx.pin,
                   uC::HANDLEs::GPIO_Handle_t::Speed::Very_High,
                   uC::HANDLEs::GPIO_Handle_t::Output_Mode::Push_Pull,
                   uC::HANDLEs::GPIO_Handle_t::Pull_Mode::No_Pull,
                   cfg.tx.af_val)
        , m_rx_pin(cfg.rx.pin,
                   uC::HANDLEs::GPIO_Handle_t::Speed::Very_High,
                   uC::HANDLEs::GPIO_Handle_t::Output_Mode::Push_Pull,
                   uC::HANDLEs::GPIO_Handle_t::Pull_Mode::No_Pull,
                   cfg.rx.af_val)
        , m_dma_handle(dma_stream_name)
        , m_buffer(BSP::get_dma_buffer_allocator().allocate<mem_payload_t>(buffer_size))
    {
      USART_TypeDef&          uart_base   = this->m_uart_handle.get_base();
      DMA_Stream_TypeDef&     stream_base = this->m_dma_handle.get_base();
      DMAMUX_Channel_TypeDef& mux_base    = this->m_dma_handle.get_mux_base();

      // clang-format off
      uart_base.BRR = this->m_uart_handle.get_clk() / 115200;
      uart_base.CR3 = USART_CR3_DMAT;
      uart_base.CR2 = 0;
      uart_base.CR1 = (( 0 << USART_CR1_FIFOEN_Pos)         & USART_CR1_FIFOEN_Msk) // todo: (( 1 << USART_CR1_FIFOEN_Pos)         & USART_CR1_FIFOEN_Msk)
                    | (( 1 << USART_CR1_RXNEIE_RXFNEIE_Pos) & USART_CR1_RXNEIE_RXFNEIE_Msk)
                    | (( 1 << USART_CR1_TE_Pos)             & USART_CR1_TE_Msk)
                    | (( 1 << USART_CR1_RE_Pos)             & USART_CR1_RE_Msk)
                    | (( 1 << USART_CR1_UE_Pos)             & USART_CR1_UE_Msk);
      
      stream_base.PAR = reinterpret_cast<uint32_t>(&uart_base.TDR);
      stream_base.CR  = DMA_SxCR_MINC 
                      | (( 0b01 << DMA_SxCR_DIR_Pos)           & DMA_SxCR_DIR_Msk)
                      | DMA_SxCR_TCIE
                      | DMA_SxCR_TEIE;

      mux_base.CCR = (( cfg.mux_val << DMAMUX_CxCR_DMAREQ_ID_Pos) & DMAMUX_CxCR_DMAREQ_ID_Msk);
      // clang-format on

      this->m_dma_handle.register_irq(this->m_transfer_complete_cb, 5);
      this->m_uart_handle.register_irq(this->m_rx_irq_cb, 0);
    }

    ~UART__TX_DMA__RX_IRQ() = default;

    bool operator()(char const* c_str) override
    {
      std::size_t len = std::strlen(c_str);
      bool const  ret = this->m_buffer.push_back(c_str, len);
      this->start_transmission();
      return ret;
    }

  private:
    bool subscribe(Subscriber_Interface& sub) override { return this->m_pup.subscribe(sub); }

    void unsubscribe(Subscriber_Interface& sub) override { return this->m_pup.unsubscribe(sub); }

    void start_transmission()
    {
      if (this->m_cur_blk_len != 0)
        return;
      this->p_start_transmission();
    }

    bool p_start_transmission()
    {
      std::span<char> blk = this->m_buffer.peak_span();
      if (blk.empty())
      {
        this->m_cur_blk_len = 0;
        return false;
      }

      uint32_t len = blk.size();
      if (len > 0xFFFF)
        len = 0xFFFF;

      DMA_Stream_TypeDef& base = this->m_dma_handle.get_base();
      base.CR &= ~DMA_SxCR_EN;
      base.NDTR = len;
      base.M0AR = reinterpret_cast<uint32_t>(blk.data());
      SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(blk.data()), len);
      this->m_cur_blk_len = len;
      base.CR |= DMA_SxCR_EN;
      return true;
    }

    void p_finish_transmission(irq_reason_t const& reason)
    {
      if (reason.is_transfer_complete())
      {
        this->m_buffer.drop(this->m_cur_blk_len);
        this->p_start_transmission();
      }
      if (reason.is_fifo_error())
      {
        __NOP();
      }
    }

    void rx_irq_handler()
    {
      USART_TypeDef& usart_base = this->m_uart_handle.get_base();
      while (usart_base.ISR & USART_ISR_RXNE_RXFNE)
      {
        char const c = usart_base.RDR;
        this->m_pup.notify_all_subscribers(c);
      }
    }

    std::atomic<bool>                                                m_trans_ongoing = false;
    std::atomic<uint32_t>                                            m_cur_blk_len   = 0;
    WLib::Memberfunction_Callback<this_t, void(irq_reason_t const&)> m_transfer_complete_cb{ *this, &this_t::p_finish_transmission };
    WLib::Memberfunction_Callback<this_t, void()>                    m_rx_irq_cb{ *this, &this_t::rx_irq_handler };

    uC::HANDLEs::USART_Handle_t              m_uart_handle;
    uC::Alternative_Funktion_Pin             m_tx_pin;
    uC::Alternative_Funktion_Pin             m_rx_pin;
    uC::HANDLEs::DMA_Stream_Handle_t         m_dma_handle;
    WLib::Container::mpsc_queue_ex_mem<char> m_buffer;
    WLib::LockFree_Publisher<char, 5>        m_pup;
  };

}    // namespace uC

#endif    // !UC_GPIO_HPP
