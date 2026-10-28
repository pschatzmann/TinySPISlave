/**
 * @file TinySPISlaveRegisters.h
 * @brief Table driven register map on top of TinySPISlave (included by TinySPISlave.h).
 *
 * Every register is described by a RegisterDef: wire address, size in bytes,
 * access rights, byte order on the wire and storage (your own variable/struct,
 * or internal storage when none is given). Registers can have any size 1..255.
 *
 * Wire protocol (one command byte, CS delimits frames):
 *
 *   write frame:  [cmd][b0][b1]...   bytes go into register `addr`; with auto-increment
 *                                    surplus bytes continue into the next register of
 *                                    the table
 *   read request: [cmd][n]           n = bytes wanted; omitted => size of register `addr`
 *   read reply:   NEXT frame         the master clocks n bytes, MISO carries the data
 *
 * cmd = (addr & addrMask) | (read ? readMask : 0), see RegisterProtocol. A slave cannot
 * answer inside the frame that carries the address, hence the separate reply frame.
 * The reply is staged as soon as the request frame ends, so the master only needs a
 * short CS-high gap (>= ~50 us RP2040/STM32, ~200 us ESP32).
 *
 * Usage:
 * @code
 *   uint16_t id = 0xBEEF; int16_t temp; uint8_t ctrl; float gains[3];
 *   tinyspi::RegisterDef defs[] = {
 *     tinyspi::Reg(0x00, id, tinyspi::ReadOnly),
 *     tinyspi::Reg(0x01, temp, tinyspi::ReadOnly, true),   // big endian on the wire
 *     tinyspi::Reg(0x02, ctrl),
 *     tinyspi::Reg(0x10, gains),                            // 12 byte register
 *     tinyspi::Reg(0x20, 64),                               // 64 byte, internal storage
 *     tinyspi::Reg(0x30, 4, [](uint8_t, void* out, void*) {  // computed on every read
 *       uint32_t t = millis(); memcpy(out, &t, 4); }),
 *   };
 *   tinyspi::RegisterMap regs(defs, sizeof(defs) / sizeof(defs[0]));
 *   // setup(): spi.begin(...); regs.begin(spi);
 *   // loop():  regs.set(0x01, readSensor()); regs.update();
 * @endcode
 *
 * Threading: frames are processed in the frame hook (interrupt on RP2040/STM32,
 * backend task on ESP32). While a RegisterMap is attached do not use the slave's
 * read()/write() yourself. Use get()/set()/read()/write() of the map to access the
 * values from loop(): they are atomic with respect to the SPI hook; plain access to
 * your own variables is not. On ESP32 use Config::slots = 1 (the default 2 delays
 * replies by one frame). The table must stay valid and sorted by ascending address.
 */
#pragma once

namespace tinyspi {

enum Access : uint8_t {
  ReadWrite = 0,
  ReadOnly = 1,   ///< writes from the master are ignored
  WriteOnly = 2,  ///< reads from the master return the fill byte
};

/// Produces the value of a register when the master reads it. Fill all `size`
/// bytes of `out` (host byte order). Runs in the frame hook (ISR on RP2040/STM32,
/// backend task on ESP32): keep it short, no Serial/heap/blocking calls.
typedef void (*RegisterReadFn)(uint8_t addr, void* out, void* ctx);

struct RegisterDef {
  uint8_t addr;      ///< id on the wire (unique, table sorted ascending)
  uint8_t size;      ///< bytes, 1..255
  Access access;
  bool bigEndian;    ///< byte order on the wire, memory order is the host's
  void* data;        ///< storage, nullptr => allocated by the map (zeroed)
  RegisterReadFn readFn;  ///< optional: computes the reply; result is also kept in storage
  void* ctx;         ///< passed to readFn
};

/// Register backed by one of your variables/structs/arrays (size = sizeof(var)).
template <typename T>
RegisterDef Reg(uint8_t addr, T& var, Access access = ReadWrite, bool bigEndian = false) {
  RegisterDef d = {addr, (uint8_t)sizeof(T), access, bigEndian, (void*)&var, nullptr, nullptr};
  return d;
}

/// Register with `size` bytes of storage owned by the map.
inline RegisterDef Reg(uint8_t addr, size_t size, Access access = ReadWrite,
                       bool bigEndian = false) {
  RegisterDef d = {addr, (uint8_t)size, access, bigEndian, nullptr, nullptr, nullptr};
  return d;
}

/// Read-only register of `size` bytes whose value is computed by `fn` on every read.
inline RegisterDef Reg(uint8_t addr, size_t size, RegisterReadFn fn, void* ctx = nullptr,
                       bool bigEndian = false) {
  RegisterDef d = {addr, (uint8_t)size, ReadOnly, bigEndian, nullptr, fn, ctx};
  return d;
}

struct RegisterProtocol {
  uint8_t readMask = 0x80;  ///< cmd bit(s) set => read request
  uint8_t addrMask = 0x7F;  ///< cmd bits that hold the register address
  uint8_t incMask = 0x00;   ///< 0: auto-increment always; else only if (cmd & incMask)
  uint8_t fill = 0x00;      ///< returned for unknown / write-only registers
};

class RegisterMap {
 public:
  /// Runs from update() (loop context) after the master wrote register `addr`.
  typedef void (*WriteCallback)(uint8_t addr, void* ctx);
  /// Runs in the frame hook (ISR/task, keep it short) right before register `addr`
  /// is copied into a read reply, so you can refresh its value. For a single
  /// register prefer RegisterDef::readFn (see Reg(addr, size, fn)).
  typedef void (*ReadCallback)(uint8_t addr, void* ctx);

  RegisterMap(const RegisterDef* defs, size_t count) : defs_(defs), count_(count) {}
  ~RegisterMap() {
    end();
    free(ptr_);
    free(pool_);
  }

  /// Attach to a running TinySPISlave. Only one map per slave.
  bool begin(TinySPISlave& slave, const RegisterProtocol& proto = RegisterProtocol()) {
    end();
    if (!prepare()) return false;
    slave_ = &slave;
    proto_ = proto;
    wHead_ = 0;
    wTail_ = 0;
    slave.onFrameImmediate(&RegisterMap::hookThunk, this);
    return true;
  }

  void end() {
    if (slave_) slave_->onFrameImmediate(nullptr, nullptr);
    slave_ = nullptr;
  }

  /// Call from loop(): delivers onWrite() notifications.
  void update() {
    while (wTail_ != wHead_) {
      uint8_t a = ev_[wTail_];
      __atomic_thread_fence(__ATOMIC_SEQ_CST);
      wTail_ = (wTail_ + 1) % kEvents;
      if (wcb_) wcb_(a, wctx_);
    }
  }

  void onWrite(WriteCallback cb, void* ctx = nullptr) { wcb_ = cb; wctx_ = ctx; }
  void onBeforeRead(ReadCallback cb, void* ctx = nullptr) { rcb_ = cb; rctx_ = ctx; }

  /// Write notifications dropped because update() was too slow (values were stored).
  uint32_t lostWriteEvents() const { return lost_; }
  /// Master accesses that touched unknown / read-only / write-only registers.
  uint32_t errors() const { return errors_; }

  bool has(uint8_t addr) { return find(addr) >= 0; }
  size_t sizeOf(uint8_t addr) { int i = find(addr); return i < 0 ? 0 : defs_[i].size; }

  // ---- application side access (atomic w.r.t. the SPI hook), host byte order ----
  /// Copy up to `len` bytes of register `addr`; returns bytes copied (0 = unknown).
  size_t read(uint8_t addr, void* dst, size_t len) {
    int i = find(addr);
    if (i < 0) return 0;
    size_t n = len < defs_[i].size ? len : defs_[i].size;
    detail::Critical c;
    memcpy(dst, ptr_[i], n);
    return n;
  }
  size_t write(uint8_t addr, const void* src, size_t len) {
    int i = find(addr);
    if (i < 0) return 0;
    size_t n = len < defs_[i].size ? len : defs_[i].size;
    detail::Critical c;
    memcpy(ptr_[i], src, n);
    return n;
  }
  template <typename T>
  T get(uint8_t addr) {
    T v;
    memset(&v, 0, sizeof(v));
    read(addr, &v, sizeof(T));
    return v;
  }
  template <typename T>
  bool set(uint8_t addr, const T& value) { return write(addr, &value, sizeof(T)) > 0; }

  /// Atomic read-modify-write of a register's first byte (flags/status registers).
  void setBits(uint8_t addr, uint8_t mask) { modify(addr, mask, 0); }
  void clearBits(uint8_t addr, uint8_t mask) { modify(addr, 0, mask); }

 private:
  static const uint8_t kEvents = 16;

  static void hookThunk(size_t len, void* self) { static_cast<RegisterMap*>(self)->onFrame(len); }

  /// Validate the table, resolve storage pointers.
  bool prepare() {
    if (ptr_) return true;
    if (!defs_ || count_ == 0) return false;
    size_t pool = 0;
    for (size_t i = 0; i < count_; i++) {
      if (defs_[i].size == 0 || (i && defs_[i].addr <= defs_[i - 1].addr)) return false;
      if (!defs_[i].data) pool += defs_[i].size;
    }
    ptr_ = (uint8_t**)malloc(count_ * sizeof(uint8_t*));
    pool_ = pool ? (uint8_t*)calloc(pool, 1) : nullptr;
    if (!ptr_ || (pool && !pool_)) {
      free(ptr_);
      free(pool_);
      ptr_ = nullptr;
      pool_ = nullptr;
      return false;
    }
    size_t off = 0;
    for (size_t i = 0; i < count_; i++) {
      if (defs_[i].data) {
        ptr_[i] = (uint8_t*)defs_[i].data;
      } else {
        ptr_[i] = pool_ + off;
        off += defs_[i].size;
      }
    }
    return true;
  }

  int find(uint8_t addr) {
    if (!prepare()) return -1;
    for (size_t i = 0; i < count_; i++) {
      if (defs_[i].addr == addr) return (int)i;
      if (defs_[i].addr > addr) break;
    }
    return -1;
  }

  void modify(uint8_t addr, uint8_t set, uint8_t clr) {
    int i = find(addr);
    if (i < 0) return;
    detail::Critical c;
    ptr_[i][0] = (uint8_t)((ptr_[i][0] | set) & ~clr);
  }

  /// Memory index of the wire byte `off` of register `i`.
  size_t memIndex(int i, size_t off) const {
    return defs_[i].bigEndian ? defs_[i].size - 1 - off : off;
  }

  void onFrame(size_t len) {
    int c = slave_->read();
    if (c < 0) return;
    len--;
    uint8_t cmd = (uint8_t)c;
    bool inc = proto_.incMask == 0 || (cmd & proto_.incMask);
    int idx = find(cmd & proto_.addrMask);

    if (cmd & proto_.readMask) {
      size_t want = idx >= 0 ? defs_[idx].size : 1;
      if (len >= 1) {
        want = (uint8_t)slave_->read();
        len--;
      }
      while (len--) slave_->read();  // ignore trailing bytes
      stageReply(idx, want, inc);
    } else {
      storeWrite(idx, len, inc);
    }
  }

  void storeWrite(int idx, size_t len, bool inc) {
    size_t off = 0;
    bool touched = false;
    while (len--) {
      int d = slave_->read();
      if (d < 0) break;
      if (idx < 0 || idx >= (int)count_ || defs_[idx].access == ReadOnly) {
        errors_ = errors_ + 1;
      } else {
        detail::Critical crit;
        ptr_[idx][memIndex(idx, off)] = (uint8_t)d;
        touched = true;
      }
      if (idx >= 0 && idx < (int)count_ && ++off == defs_[idx].size) {
        if (touched) pushEvent(defs_[idx].addr);
        touched = false;
        off = 0;
        if (inc) idx++;
      }
    }
    if (touched) pushEvent(defs_[idx].addr);  // frame ended inside a register
  }

  void refresh(int idx) {
    if (rcb_) rcb_(defs_[idx].addr, rctx_);
    if (defs_[idx].readFn && defs_[idx].access != WriteOnly) {
      uint8_t tmp[255];
      memset(tmp, 0, defs_[idx].size);
      defs_[idx].readFn(defs_[idx].addr, tmp, defs_[idx].ctx);
      detail::Critical crit;
      memcpy(ptr_[idx], tmp, defs_[idx].size);
    }
  }

  void stageReply(int idx, size_t want, bool inc) {
    slave_->flushTx();
    size_t room = (size_t)slave_->availableForWrite();
    if (want > room) {
      want = room;
      errors_ = errors_ + 1;
    }
    size_t off = 0;
    while (want) {
      if (idx < 0 || idx >= (int)count_) {  // unknown register / past the table: fill
        errors_ = errors_ + 1;
        while (want--) slave_->write(proto_.fill);
        return;
      }
      if (off == 0) refresh(idx);  // global callback + per-register readFn
      size_t n = defs_[idx].size - off;
      if (n > want) n = want;
      bool blocked = defs_[idx].access == WriteOnly;
      if (blocked) errors_ = errors_ + 1;
      {
        detail::Critical crit;
        for (size_t k = 0; k < n; k++) {
          slave_->write(blocked ? proto_.fill : ptr_[idx][memIndex(idx, off + k)]);
        }
      }
      want -= n;
      off += n;
      if (off == defs_[idx].size) {
        off = 0;
        if (inc) idx++;  // else: the same register is repeated
      }
    }
  }

  void pushEvent(uint8_t addr) {
    uint8_t h = wHead_, nx = (h + 1) % kEvents;
    if (nx == wTail_) {
      lost_ = lost_ + 1;
      return;
    }
    ev_[h] = addr;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    wHead_ = nx;
  }

  TinySPISlave* slave_ = nullptr;
  const RegisterDef* defs_;
  size_t count_;
  uint8_t** ptr_ = nullptr;
  uint8_t* pool_ = nullptr;
  RegisterProtocol proto_;
  WriteCallback wcb_ = nullptr;
  void* wctx_ = nullptr;
  ReadCallback rcb_ = nullptr;
  void* rctx_ = nullptr;
  uint8_t ev_[kEvents];
  volatile uint8_t wHead_ = 0;
  volatile uint8_t wTail_ = 0;
  volatile uint32_t lost_ = 0;
  volatile uint32_t errors_ = 0;
};

}  // namespace tinyspi
