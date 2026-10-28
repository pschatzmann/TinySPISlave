// RP2040 / RP2350 backend (arduino-pico): PL022 SPI block in slave mode.
#pragma once

#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/spi.h>

namespace tinyspi {
namespace detail {

class Backend {
 public:
  bool begin(const Config& cfg, Shared* s) {
    // GPIO function select: (pin & 3) = 0:RX 1:CSn 2:SCK 3:TX, block = (pin >> 3) & 1.
    // In slave mode RX is the master's MOSI and TX is the master's MISO.
    if (!pinsOk(cfg)) return false;
    idx_ = (cfg.sck >> 3) & 1;
    if (slot(idx_)) return false;  // already in use by another instance
    s_ = s;
    spi_ = idx_ ? spi1 : spi0;
    cs_ = cfg.cs;
    pins_[0] = cfg.sck;
    pins_[1] = cfg.miso;
    pins_[2] = cfg.mosi;
    pins_[3] = cfg.cs;

    spi_init(spi_, 1000 * 1000);
    spi_set_format(spi_, 8, (spi_cpol_t)((cfg.mode >> 1) & 1), (spi_cpha_t)(cfg.mode & 1),
                   SPI_MSB_FIRST);
    spi_set_slave(spi_, true);
    for (int p : pins_) gpio_set_function(p, GPIO_FUNC_SPI);

    spi_hw_t* hw = spi_get_hw(spi_);
    hw->icr = SPI_SSPICR_RORIC_BITS | SPI_SSPICR_RTIC_BITS;
    hw->imsc = SPI_SSPIMSC_RXIM_BITS | SPI_SSPIMSC_RTIM_BITS | SPI_SSPIMSC_RORIM_BITS;
    slot(idx_) = this;
    // End of frame = CS rising edge (shared handler: coexists with attachInterrupt)
    gpio_set_irq_enabled(cs_, GPIO_IRQ_EDGE_RISE, true);
    irq_add_shared_handler(IO_IRQ_BANK0, idx_ ? &Backend::gpioIsr1 : &Backend::gpioIsr0,
                           PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);
    irq_set_enabled(IO_IRQ_BANK0, true);
    uint irq = idx_ ? SPI1_IRQ : SPI0_IRQ;
    irq_set_exclusive_handler(irq, idx_ ? &Backend::isr1 : &Backend::isr0);
    irq_set_enabled(irq, true);
    return true;
  }

  void end() {
    uint irq = idx_ ? SPI1_IRQ : SPI0_IRQ;
    irq_set_enabled(irq, false);
    irq_remove_handler(irq, idx_ ? &Backend::isr1 : &Backend::isr0);
    gpio_set_irq_enabled(cs_, GPIO_IRQ_EDGE_RISE, false);
    irq_remove_handler(IO_IRQ_BANK0, idx_ ? &Backend::gpioIsr1 : &Backend::gpioIsr0);
    spi_deinit(spi_);
    for (int p : pins_) gpio_set_function(p, GPIO_FUNC_NULL);
    slot(idx_) = nullptr;
  }

  /// Wake the TX interrupt so it moves queued bytes into the FIFO.
  void kickTx() { hw_set_bits(&spi_get_hw(spi_)->imsc, SPI_SSPIMSC_TXIM_BITS); }

 private:
  static bool pinsOk(const Config& cfg) {
    int blk = (cfg.sck >> 3) & 1;
    return ((cfg.mosi >> 3) & 1) == blk && ((cfg.miso >> 3) & 1) == blk &&
           ((cfg.cs >> 3) & 1) == blk && (cfg.sck & 3) == 2 && (cfg.cs & 3) == 1 &&
           (cfg.mosi & 3) == 0 && (cfg.miso & 3) == 3;
  }

  static Backend*& slot(int i) {
    static Backend* inst[2] = {nullptr, nullptr};
    return inst[i];
  }
  static void isr0() { if (slot(0)) slot(0)->service(); }
  static void isr1() { if (slot(1)) slot(1)->service(); }
  static void gpioIsr0() { if (slot(0)) slot(0)->onCsRise(); }
  static void gpioIsr1() { if (slot(1)) slot(1)->onCsRise(); }

  void onCsRise() {
    if (gpio_get_irq_event_mask(cs_) & GPIO_IRQ_EDGE_RISE) {
      gpio_acknowledge_irq(cs_, GPIO_IRQ_EDGE_RISE);
      service();  // drain what is left in the rx FIFO
      s_->frameEnd();
    }
  }

  void service() {
    spi_hw_t* hw = spi_get_hw(spi_);
    while (spi_is_readable(spi_)) s_->pushRx((uint8_t)hw->dr);
    hw->icr = SPI_SSPICR_RORIC_BITS | SPI_SSPICR_RTIC_BITS;
    while (spi_is_writable(spi_)) {
      int b = s_->tx.pop();
      if (b < 0) break;
      hw->dr = (uint32_t)b;
    }
    if (s_->tx.available()) {
      hw_set_bits(&hw->imsc, SPI_SSPIMSC_TXIM_BITS);
    } else {
      hw_clear_bits(&hw->imsc, SPI_SSPIMSC_TXIM_BITS);
      // closes the race with a write() that landed after the check above
      if (s_->tx.available()) hw_set_bits(&hw->imsc, SPI_SSPIMSC_TXIM_BITS);
    }
  }

  Shared* s_ = nullptr;
  spi_inst_t* spi_ = nullptr;
  uint8_t idx_ = 0;
  int cs_ = -1;
  int pins_[4] = {-1, -1, -1, -1};
};

}  // namespace detail
}  // namespace tinyspi
