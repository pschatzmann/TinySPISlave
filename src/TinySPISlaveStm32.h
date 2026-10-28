// STM32 backend (STM32duino): spi_init() in peripheral mode + LL, interrupt driven.
#pragma once

#include <SPI.h>  // pulls in utility/spi_com.h and the HAL headers
#include "stm32yyxx_ll_spi.h"

#if defined(SPI_SR_RXP)
#define TINYSPI_STM32_SPI_V2 1  // H7/H5/U5/... style SPI with TXP/RXP flags
#endif

namespace tinyspi {
namespace detail {

class Backend {
 public:
  bool begin(const Config& cfg, Shared* s) {
    s_ = s;
    cs_ = cfg.cs;
    memset(&spi_, 0, sizeof(spi_));
    // End of frame = CS rising edge. Must be armed *before* spi_init(): attachInterrupt()
    // sets the pin to input, spi_init() then switches it to the NSS alternate function
    // while the EXTI line stays configured.
    attachInterruptParam(cfg.cs, &Backend::csIsr, RISING, this);
    spi_.pin_miso = digitalPinToPinName(cfg.miso);
    spi_.pin_mosi = digitalPinToPinName(cfg.mosi);
    spi_.pin_sclk = digitalPinToPinName(cfg.sck);
    spi_.pin_ssel = digitalPinToPinName(cfg.cs);
    // The speed argument is ignored by the hardware in peripheral mode.
    spi_init(&spi_, SPI_SPEED_CLOCK_DEFAULT, (spi_mode_e)cfg.mode, true, SPI_PERIPHERAL_C);
    int idx = spi_.spi ? indexOf(spi_.spi) : -1;  // NULL: pins not on one SPI instance
    if (idx < 0 || slot(idx)) {
      detachInterrupt(cfg.cs);
      return false;
    }
    idx_ = idx;

    SPI_TypeDef* h = spi_.spi;
#if defined(SPI_CR2_FRXTH)
    LL_SPI_Disable(h);
    LL_SPI_SetRxFIFOThreshold(h, LL_SPI_RX_FIFO_TH_QUARTER);  // 8 bit frames
    LL_SPI_Enable(h);
#endif
    slot(idx_) = this;
    irq_ = irqOf(h);
    HAL_NVIC_SetPriority(irq_, 1, 0);
    HAL_NVIC_EnableIRQ(irq_);
#if TINYSPI_STM32_SPI_V2
    LL_SPI_EnableIT_RXP(h);
#else
    LL_SPI_EnableIT_RXNE(h);
#endif
    return true;
  }

  void end() {
    detachInterrupt(cs_);
    HAL_NVIC_DisableIRQ(irq_);
    slot(idx_) = nullptr;
    spi_deinit(&spi_);
  }

  void kickTx() {
#if TINYSPI_STM32_SPI_V2
    LL_SPI_EnableIT_TXP(spi_.spi);
#else
    LL_SPI_EnableIT_TXE(spi_.spi);
#endif
  }

  static void dispatch(int idx) {
    if (slot(idx)) slot(idx)->service();
  }

 private:
  static void csIsr(void* self) {
    Backend* b = static_cast<Backend*>(self);
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    b->service();  // drain what is left in the rx FIFO
    b->s_->frameEnd();
    __set_PRIMASK(pm);
  }

  static Backend*& slot(int i) {
    static Backend* inst[6] = {};
    return inst[i];
  }

  static int indexOf(SPI_TypeDef* i) {
#if defined(SPI1_BASE)
    if (i == SPI1) return 0;
#endif
#if defined(SPI2_BASE)
    if (i == SPI2) return 1;
#endif
#if defined(SPI3_BASE)
    if (i == SPI3) return 2;
#endif
#if defined(SPI4_BASE)
    if (i == SPI4) return 3;
#endif
#if defined(SPI5_BASE)
    if (i == SPI5) return 4;
#endif
#if defined(SPI6_BASE)
    if (i == SPI6) return 5;
#endif
    return -1;
  }

  static IRQn_Type irqOf(SPI_TypeDef* i) {
#if defined(SPI1_BASE)
    if (i == SPI1) return SPI1_IRQn;
#endif
#if defined(STM32G0xx) || defined(STM32C0xx)
#if defined(SPI2_BASE)
    if (i == SPI2) return SPI2_3_IRQn;
#endif
#if defined(SPI3_BASE)
    if (i == SPI3) return SPI2_3_IRQn;
#endif
#else
#if defined(SPI2_BASE)
    if (i == SPI2) return SPI2_IRQn;
#endif
#if defined(SPI3_BASE)
    if (i == SPI3) return SPI3_IRQn;
#endif
#if defined(SPI4_BASE)
    if (i == SPI4) return SPI4_IRQn;
#endif
#if defined(SPI5_BASE)
    if (i == SPI5) return SPI5_IRQn;
#endif
#if defined(SPI6_BASE)
    if (i == SPI6) return SPI6_IRQn;
#endif
#endif
    return SPI1_IRQn;
  }

  void service() {
    SPI_TypeDef* h = spi_.spi;
#if TINYSPI_STM32_SPI_V2
    while (LL_SPI_IsActiveFlag_RXP(h)) s_->pushRx(LL_SPI_ReceiveData8(h));
    if (LL_SPI_IsActiveFlag_OVR(h)) LL_SPI_ClearFlag_OVR(h);
    while (LL_SPI_IsActiveFlag_TXP(h)) {
      int b = s_->tx.pop();
      if (b < 0) break;
      LL_SPI_TransmitData8(h, (uint8_t)b);
    }
    if (s_->tx.available()) {
      LL_SPI_EnableIT_TXP(h);
    } else {
      LL_SPI_DisableIT_TXP(h);
      if (s_->tx.available()) LL_SPI_EnableIT_TXP(h);
    }
#else
    while (LL_SPI_IsActiveFlag_RXNE(h)) s_->pushRx(LL_SPI_ReceiveData8(h));
    if (LL_SPI_IsActiveFlag_OVR(h)) LL_SPI_ClearFlag_OVR(h);
#if defined(SPI_SR_FTLVL)
    while (LL_SPI_GetTxFIFOLevel(h) != LL_SPI_TX_FIFO_FULL) {
#else
    while (LL_SPI_IsActiveFlag_TXE(h)) {
#endif
      int b = s_->tx.pop();
      if (b < 0) break;
      LL_SPI_TransmitData8(h, (uint8_t)b);
    }
    if (s_->tx.available()) {
      LL_SPI_EnableIT_TXE(h);
    } else {
      LL_SPI_DisableIT_TXE(h);
      if (s_->tx.available()) LL_SPI_EnableIT_TXE(h);
    }
#endif
  }

  Shared* s_ = nullptr;
  spi_t spi_;
  IRQn_Type irq_ = SPI1_IRQn;
  int idx_ = 0;
  int cs_ = -1;
};

}  // namespace detail
}  // namespace tinyspi

// The core does not define the SPI vectors, so provide them (weak: a user
// handler wins). Only the instances present on the selected chip are emitted.
#define TINYSPI_STM32_HANDLER(name, idx)                                  \
  extern "C" __attribute__((used, weak)) void name(void) {                \
    tinyspi::detail::Backend::dispatch(idx);                              \
  }

#if defined(STM32G0xx) || defined(STM32C0xx)
#if defined(SPI1_BASE)
TINYSPI_STM32_HANDLER(SPI1_IRQHandler, 0)
#endif
extern "C" __attribute__((used, weak)) void SPI2_3_IRQHandler(void) {
  tinyspi::detail::Backend::dispatch(1);
  tinyspi::detail::Backend::dispatch(2);
}
#else
#if defined(SPI1_BASE)
TINYSPI_STM32_HANDLER(SPI1_IRQHandler, 0)
#endif
#if defined(SPI2_BASE)
TINYSPI_STM32_HANDLER(SPI2_IRQHandler, 1)
#endif
#if defined(SPI3_BASE)
TINYSPI_STM32_HANDLER(SPI3_IRQHandler, 2)
#endif
#if defined(SPI4_BASE)
TINYSPI_STM32_HANDLER(SPI4_IRQHandler, 3)
#endif
#if defined(SPI5_BASE)
TINYSPI_STM32_HANDLER(SPI5_IRQHandler, 4)
#endif
#if defined(SPI6_BASE)
TINYSPI_STM32_HANDLER(SPI6_IRQHandler, 5)
#endif
#endif
