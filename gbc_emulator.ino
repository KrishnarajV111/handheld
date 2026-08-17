#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define ENABLE_SOUND 0
#define ENABLE_LCD 1

#include "walnut_cgb.h"

// ═══════════════════════════════════════════════════════════════
// PROFILING
// ═══════════════════════════════════════════════════════════════
struct ProfileData {
  uint32_t emu_time_acc;
  uint32_t ppu_time_acc;
  uint32_t core0_wait_acc;
  volatile uint32_t tft_time_acc;
  uint32_t sd_time_acc;
  uint32_t save_time_acc;
  uint32_t cache_misses;
  uint32_t prefetch_requests;
  uint32_t prefetch_hits;
  uint32_t prefetch_sync_fallbacks;
  uint32_t prefetch_cancellations;
  uint32_t prefetch_wasted_bytes;
  uint32_t prefetch_wait_time_acc;
  uint32_t demand_requests;
  uint32_t demand_wait_time_acc;
  uint32_t loop_time_acc;
  uint32_t frames;
  uint32_t min_free_heap;
};
static ProfileData prof;

static void reset_prof() {
  prof.emu_time_acc = 0;
  prof.ppu_time_acc = 0;
  prof.core0_wait_acc = 0;
  prof.tft_time_acc = 0;
  prof.sd_time_acc = 0;
  prof.save_time_acc = 0;
  prof.save_time_acc = 0;
  prof.cache_misses = 0;
  prof.prefetch_requests = 0;
  prof.prefetch_hits = 0;
  prof.prefetch_sync_fallbacks = 0;
  prof.prefetch_cancellations = 0;
  prof.prefetch_wasted_bytes = 0;
  prof.prefetch_wait_time_acc = 0;
  prof.demand_requests = 0;
  prof.demand_wait_time_acc = 0;
  prof.loop_time_acc = 0;
  prof.frames = 0;
  prof.min_free_heap = ESP.getFreeHeap();
}

// ═══════════════════════════════════════════════════════════════
// PIN CONFIGURATION — DUAL SPI BUS ARCHITECTURE
// ═══════════════════════════════════════════════════════════════
//
// The ESP32 has TWO independent hardware SPI controllers:
//   VSPI (SPI3) — drives the TFT display
//   HSPI (SPI2) — drives the SD card
//
// They have completely separate buses, DMA channels, and GPIO pins.
// TFT is updated via VSPI while emulator state is held.
// Core 1 manages all SD interactions via HSPI simultaneously. I/O.
//
// ┌─────────────────────────────────────────────────────────┐
// │                     REWIRING GUIDE                      │
// ├─────────────────────────────────────────────────────────┤
// │                                                         │
// │  TFT Display (VSPI — UNCHANGED from before):            │
// │    SDI / MOSI  →  GPIO 23                               │
// │    SCK / CLK   →  GPIO 18                               │
// │    CS          →  GPIO 15                               │
// │    DC          →  GPIO 2                                │
// │    RST         →  GPIO 4                                │
// │    LED         →  3.3V                                  │
// │    SDO / MISO  →  (Not connected)                       │
// │    VCC         →  3.3V                                  │
// │    GND         →  GND                                   │
// │                                                         │
// │  SD Card Module (HSPI — NEW WIRING!):                   │
// │    MOSI        →  GPIO 5    ← (was 23, MOVE THIS)      │
// │    MISO        →  GPIO 19   ← (was shared, now exclusive) │
// │    SCK / CLK   →  GPIO 21   ← (was 18, MOVE THIS)      │
// │    CS          →  GPIO 22   ← (was 5,  MOVE THIS)      │
// │    VCC         →  3.3V                                  │
// │    GND         →  GND                                   │
// │                                                         │
// │  Buttons: UNCHANGED                                     │
// │    UP=13, DOWN=12, LEFT=14, RIGHT=27                    │
// │    A=32, B=33, START=25, SELECT=26                      │
// └─────────────────────────────────────────────────────────┘

// ── VSPI: TFT Display ──
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4

// ── HSPI: SD Card (NEW pins!) ──
#define SD_MOSI  5
#define SD_MISO  19
#define SD_SCK   21
#define SD_CS    22

// ── Buttons ──
#define BTN_UP     13
#define BTN_DOWN   12
#define BTN_LEFT   14
#define BTN_RIGHT  27
#define BTN_A      32
#define BTN_B      33
#define BTN_START  25
#define BTN_SELECT 26

// TFT on default VSPI (SPI3)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// SD on separate HSPI (SPI2) — completely independent bus
SPIClass hspi(HSPI);

// DMG palette
uint16_t CURRENT_PALETTE_RGB565[4] = { 0x7BEF, 0x5BE8, 0x3AD1, 0x2A29 };

// ═══════════════════════════════════════════════════════════════
// O(1) ROM CACHE — CONTIGUOUS 64KB BLOCK
// ═══════════════════════════════════════════════════════════════
#define BANK_SIZE    16384
#define CACHE_SLOTS  3
#define MAX_BANKS    512

struct CacheSlot {
  uint8_t  *data;
  int16_t  bank_num;
};

static CacheSlot bank_cache[CACHE_SLOTS];
static uint8_t  *cache_base = NULL;
static uint8_t   rom_cache_static[CACHE_SLOTS * BANK_SIZE]; // TEST C: Static 64KB cache
static uint16_t  framebuffer_static[160 * 144];             // TEST C: Static 46KB framebuffer
static int8_t    bank_to_slot[MAX_BANKS];

// [PHASE 3] Cartridge-aware replacement: Active & Prev slots
static uint8_t *hot_bank_ptr  = NULL;
static int      hot_bank_num  = -1;
static int      hot_bank_slot = -1;
static int      prev_bank_slot = -1;

#include <atomic>

// [PHASE 5] Predictive Prefetch State
enum PrefetchState { PF_FREE = 0, PF_REQUESTED = 1, PF_LOADING = 2, PF_READY = 3 };
static std::atomic<PrefetchState> pf_state(PF_FREE);
static std::atomic<uint32_t> pf_request_gen(0);
static std::atomic<int> pf_request_bank(-1);
static std::atomic<int> pf_active_slot(-1);

// [PHASE 5.5] Demand State
enum DemandState { DEMAND_FREE = 0, DEMAND_REQUESTED = 1, DEMAND_LOADING = 2, DEMAND_READY = 3, DEMAND_ERROR = 4 };
static std::atomic<DemandState> dem_state(DEMAND_FREE);
static std::atomic<uint32_t> dem_gen(0);
static std::atomic<int> dem_bank(-1);
static std::atomic<int> dem_slot(-1);

// [PHASE 5.5] Save State
enum SaveState { SAVE_FREE = 0, SAVE_REQUESTED = 1, SAVE_WRITING = 2, SAVE_DONE = 3, SAVE_ERROR = 4 };
static std::atomic<SaveState> save_state(SAVE_FREE);

static File     rom_file;
static uint32_t rom_total_size;
static uint32_t frame_count = 0;
static TaskHandle_t sdTaskHandle = NULL;

static inline uint8_t* __attribute__((always_inline))
resolve_bank_l2(int bank) {
  if (bank < MAX_BANKS) {
    int8_t slot = bank_to_slot[bank];
    if (slot >= 0) {
      prev_bank_slot = hot_bank_slot;
      hot_bank_ptr  = bank_cache[slot].data;
      hot_bank_num  = bank;
      hot_bank_slot = slot;
      return hot_bank_ptr;
    }
  }
  return NULL;
}

static uint8_t* cache_miss(int bank) {
  prof.cache_misses++;

  // [PHASE 5] Check if the bank we need was predictively prefetched
  PrefetchState p_state = pf_state.load(std::memory_order_acquire);
  int req_bank = pf_request_bank.load(std::memory_order_relaxed);
  
  if (req_bank == bank) {
    if (p_state == PF_READY) {
      prof.prefetch_hits++;
      int slot = pf_active_slot.load(std::memory_order_relaxed);
      bank_to_slot[bank] = slot;
      pf_state.store(PF_FREE, std::memory_order_release);
      
      prev_bank_slot = hot_bank_slot;
      hot_bank_ptr = bank_cache[slot].data;
      hot_bank_num = bank;
      hot_bank_slot = slot;
      return hot_bank_ptr;
    } else if (p_state == PF_REQUESTED || p_state == PF_LOADING) {
      uint32_t t0 = micros();
      while (true) {
        p_state = pf_state.load(std::memory_order_acquire);
        if (p_state == PF_READY || p_state == PF_FREE) break;
        if ((micros() - t0) >= 2000) break;
        taskYIELD();
      }
      prof.prefetch_wait_time_acc += (micros() - t0);
      
      if (p_state == PF_READY) {
        prof.prefetch_hits++;
        int slot = pf_active_slot.load(std::memory_order_relaxed);
        bank_to_slot[bank] = slot;
        pf_state.store(PF_FREE, std::memory_order_release);
        
        prev_bank_slot = hot_bank_slot;
        hot_bank_ptr = bank_cache[slot].data;
        hot_bank_num = bank;
        hot_bank_slot = slot;
        return hot_bank_ptr;
      } else {
        // Timeout! Advance generation to invalidate the request
        prof.prefetch_sync_fallbacks++;
        pf_request_gen.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  // [PHASE 5.5] Select victim slot safely. If all taken, cancel prefetch and wait.
  Serial.printf(
    "CACHE MISS: bank=%d hot=%d prev=%d pf_state=%d pf_slot=%d\n",
    bank,
    hot_bank_slot,
    prev_bank_slot,
    (int)pf_state.load(std::memory_order_acquire),
    pf_active_slot.load(std::memory_order_relaxed)
  );

  for (int i = 0; i < CACHE_SLOTS; i++) {
      Serial.printf(
        "  slot %d: bank=%d hot=%s prev=%s pf_active=%s\n",
        i,
        bank_cache[i].bank_num,
        (i == hot_bank_slot) ? "YES" : "NO",
        (i == prev_bank_slot) ? "YES" : "NO",
        (i == pf_active_slot.load()) ? "YES" : "NO"
      );
  }

  int victim_slot = -1;
  uint32_t t_victim = micros();
  while (victim_slot == -1) {
    for (int i = 1; i < CACHE_SLOTS; i++) {
      // If we only have 3 slots (1 fixed + 2 dynamic), protecting both hot and prev 
      // leaves ZERO slots available for a cache miss, causing a deadlock!
      // Therefore, if CACHE_SLOTS < 4, we must sacrifice the prev_bank_slot protection.
      bool is_protected = (i == hot_bank_slot) || (CACHE_SLOTS >= 4 && i == prev_bank_slot);
      if (!is_protected) {
        p_state = pf_state.load(std::memory_order_acquire);
        int pf_slot = pf_active_slot.load(std::memory_order_relaxed);
        if (p_state != PF_FREE && i == pf_slot) continue;
        
        victim_slot = i;
        break;
      }
    }
    
    if (victim_slot == -1) {
      // All slots protected! We must wait for the prefetch to finish and FREE.
      if ((micros() - t_victim) > 100000) { // 100ms timeout
        Serial.println("FATAL: Cache victim timeout");
        while(1) { taskYIELD(); }
      }
      taskYIELD();
    }
  }
  
  int old = bank_cache[victim_slot].bank_num;
  if (old >= 0 && old < MAX_BANKS) bank_to_slot[old] = -1;
  bank_cache[victim_slot].bank_num = -1;

  // [PHASE 5.5] Issue DEMAND to Core 1 sdTask
  dem_bank.store(bank, std::memory_order_relaxed);
  dem_slot.store(victim_slot, std::memory_order_relaxed);
  dem_gen.fetch_add(1, std::memory_order_relaxed);
  dem_state.store(DEMAND_REQUESTED, std::memory_order_release);
  prof.demand_requests++;
  
  if (sdTaskHandle) xTaskNotifyGive(sdTaskHandle);
  
  uint32_t t_wait = micros();
  while (true) {
    DemandState d_state = dem_state.load(std::memory_order_acquire);
    if (d_state == DEMAND_READY) break;
    if (d_state == DEMAND_ERROR) {
      Serial.println("SD DEMAND ERROR!");
      while (1) { taskYIELD(); }
    }
    if ((micros() - t_wait) > 150000) { // 150ms timeout
      Serial.println("FATAL: SD DEMAND timeout");
      while (1) { taskYIELD(); }
    }
    taskYIELD();
  }
  prof.demand_wait_time_acc += (micros() - t_wait);
  
  dem_state.store(DEMAND_FREE, std::memory_order_release);
  
  bank_cache[victim_slot].bank_num = bank;
  bank_to_slot[bank] = victim_slot;
  
  prev_bank_slot = hot_bank_slot;
  hot_bank_ptr  = bank_cache[victim_slot].data;
  hot_bank_num  = bank;
  hot_bank_slot = victim_slot;
  return hot_bank_ptr;
}

uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr) {
  uint32_t bank = addr >> 14;
  uint32_t offset = addr & 0x3FFF;
  if (bank == 0) return bank_cache[0].data[offset];
  if (bank == hot_bank_num) return hot_bank_ptr[offset];
  
  uint8_t *data = resolve_bank_l2(bank);
  if (!data) data = cache_miss(bank);
  return data[offset];
}

uint16_t gb_rom_read_16bit(struct gb_s *gb, const uint_fast32_t addr) {
  uint32_t bank = addr >> 14;
  uint32_t offset = addr & 0x3FFF;
  if (offset < BANK_SIZE - 1) {
    uint8_t *data;
    if (bank == 0) data = bank_cache[0].data;
    else if (bank == hot_bank_num) data = hot_bank_ptr;
    else {
      data = resolve_bank_l2(bank);
      if (!data) data = cache_miss(bank);
    }
    return data[offset] | (data[offset + 1] << 8);
  }
  return gb_rom_read(gb, addr) | (gb_rom_read(gb, addr + 1) << 8);
}

uint32_t gb_rom_read_32bit(struct gb_s *gb, const uint_fast32_t addr) {
  uint32_t bank = addr >> 14;
  uint32_t offset = addr & 0x3FFF;
  if (offset < BANK_SIZE - 3) {
    uint8_t *data;
    if (bank == 0) data = bank_cache[0].data;
    else if (bank == hot_bank_num) data = hot_bank_ptr;
    else {
      data = resolve_bank_l2(bank);
      if (!data) data = cache_miss(bank);
    }
    return data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24);
  }
  return gb_rom_read(gb, addr) | (gb_rom_read(gb, addr + 1) << 8) | (gb_rom_read(gb, addr + 2) << 16) | (gb_rom_read(gb, addr + 3) << 24);
}

// ═══════════════════════════════════════════════════════════════
// CART RAM - Saves are dispatched to Core 1 sdTask
// ═══════════════════════════════════════════════════════════════
#define SAVE_INTERVAL_FRAMES 300
static char save_path[100];
static uint8_t *cart_ram_buf = NULL;
static size_t   cart_ram_size = 0;
static bool     cart_ram_dirty = false;

void save_cart_ram() {
  if (!cart_ram_dirty || cart_ram_size == 0) return;
  
  save_state.store(SAVE_REQUESTED, std::memory_order_release);
  if (sdTaskHandle) xTaskNotifyGive(sdTaskHandle);
  
  while (true) {
    SaveState s_state = save_state.load(std::memory_order_acquire);
    if (s_state == SAVE_DONE) break;
    if (s_state == SAVE_ERROR) {
      Serial.println("SD SAVE ERROR!");
      break;
    }
    taskYIELD();
  }
  
  save_state.store(SAVE_FREE, std::memory_order_release);
  cart_ram_dirty = false;
}

void load_cart_ram() {
  if (cart_ram_size == 0) return;
  File f = SD.open(save_path);
  if (f && f.size() == cart_ram_size) {
    f.read(cart_ram_buf, cart_ram_size);
    f.close();
  } else {
    memset(cart_ram_buf, 0xFF, cart_ram_size);
  }
}

uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr) {
  if (!cart_ram_buf || addr >= cart_ram_size) return 0xFF;
  return cart_ram_buf[addr];
}

void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
                       const uint8_t val) {
  if (!cart_ram_buf || addr >= cart_ram_size) return;
  cart_ram_buf[addr] = val;
  cart_ram_dirty = true;
}

struct priv_t {};

void gb_error(struct gb_s *gb, const enum gb_error_e gb_err,
              const uint16_t val) {
  save_cart_ram();
  while (1);
}

// ═══════════════════════════════════════════════════════════════
// DUAL-CORE TFT RENDERING — Core 1 pushes via VSPI
// ═══════════════════════════════════════════════════════════════
#define GB_W      160
#define GB_H      144
#define FB_PIXELS (GB_W * GB_H)
#define FB_BYTES  (FB_PIXELS * 2)
#define OFFSET_X  ((320 - GB_W) / 2)
#define OFFSET_Y  ((240 - GB_H) / 2)

static uint16_t *framebuffer = NULL;
static bool frame_drawn = false;

static uint16_t * volatile render_buffer = NULL;
static TaskHandle_t renderTaskHandle = NULL;

// [PHASE 5] Callback from Walnut when MBC bank changes
void gb_rom_bank_changed_callback(struct gb_s* gb, uint16_t new_bank) {
  if (new_bank < MAX_BANKS && bank_to_slot[new_bank] >= 0) return; 

  PrefetchState state = pf_state.load(std::memory_order_acquire);
  if (state == PF_REQUESTED || state == PF_LOADING) {
    return; // Do not issue another request while one is in flight
  }

  if (state == PF_READY) {
    if (pf_request_bank.load(std::memory_order_relaxed) == new_bank) return;
    pf_state.store(PF_FREE, std::memory_order_release); // Discard stale completed prefetch
  }

  // Pick victim slot
  int victim = -1;
  for (int i = 1; i < CACHE_SLOTS; i++) {
    if (i != hot_bank_slot && i != prev_bank_slot) { victim = i; break; }
  }
  
  if (victim > 0) {
    // Safely invalidate victim metadata on Core 0
    int old = bank_cache[victim].bank_num;
    if (old >= 0 && old < MAX_BANKS) bank_to_slot[old] = -1;
    bank_cache[victim].bank_num = -1;
    
    pf_request_bank.store(new_bank, std::memory_order_relaxed);
    pf_active_slot.store(victim, std::memory_order_relaxed);
    pf_request_gen.fetch_add(1, std::memory_order_relaxed);
    pf_state.store(PF_REQUESTED, std::memory_order_release);
    prof.prefetch_requests++;
    
    if (sdTaskHandle) xTaskNotifyGive(sdTaskHandle);
  }
}

// Core 1 High-Priority Task: TFT VSPI
void renderTask(void *pvParameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (render_buffer) {
      uint32_t t0 = micros();
      tft.startWrite();
      tft.setAddrWindow(OFFSET_X, OFFSET_Y, GB_W, GB_H);
      tft.writePixels((uint16_t*)render_buffer, FB_PIXELS);
      tft.endWrite();
      prof.tft_time_acc += (micros() - t0);
      render_buffer = NULL;
    }
  }
}

// Core 1 Low-Priority Task: SD I/O (HSPI)
void sdTask(void *pvParameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // 1. DEMAND Priority
    DemandState d_state = dem_state.load(std::memory_order_acquire);
    if (d_state == DEMAND_REQUESTED) {
      uint32_t my_gen = dem_gen.load(std::memory_order_relaxed);
      int my_slot = dem_slot.load(std::memory_order_relaxed);
      int my_bank = dem_bank.load(std::memory_order_relaxed);
      
      dem_state.store(DEMAND_LOADING, std::memory_order_release);
      
      uint32_t offset = (uint32_t)my_bank * BANK_SIZE;
      size_t to_read = BANK_SIZE;
      if (offset + to_read > rom_total_size) {
        to_read = rom_total_size - offset;
      }
      
      uint32_t t0 = micros();
      bool success = rom_file.seek(offset);
      if (success) {
        size_t read_bytes = rom_file.read(bank_cache[my_slot].data, to_read);
        if (read_bytes != to_read && to_read > 0) success = false;
      }
      prof.sd_time_acc += (micros() - t0);
      
      uint32_t current_gen = dem_gen.load(std::memory_order_acquire);
      if (current_gen == my_gen) {
        if (success) {
          dem_state.store(DEMAND_READY, std::memory_order_release);
        } else {
          dem_state.store(DEMAND_ERROR, std::memory_order_release);
        }
      } else {
        dem_state.store(DEMAND_FREE, std::memory_order_release);
      }
      if (sdTaskHandle) xTaskNotifyGive(sdTaskHandle); // Loop again
      continue;
    }
    
    // 2. PREFETCH Priority
    PrefetchState p_state = pf_state.load(std::memory_order_acquire);
    if (p_state == PF_REQUESTED) {
      uint32_t my_gen = pf_request_gen.load(std::memory_order_relaxed);
      int my_slot = pf_active_slot.load(std::memory_order_relaxed);
      int my_bank = pf_request_bank.load(std::memory_order_relaxed);
      
      pf_state.store(PF_LOADING, std::memory_order_release);
      
      uint32_t offset = (uint32_t)my_bank * BANK_SIZE;
      size_t to_read = BANK_SIZE;
      if (offset + to_read > rom_total_size) {
        to_read = rom_total_size - offset;
      }
      
      uint32_t t0 = micros();
      bool success = rom_file.seek(offset);
      if (success) {
        size_t read_bytes = rom_file.read(bank_cache[my_slot].data, to_read);
        if (read_bytes != to_read && to_read > 0) success = false;
      }
      prof.sd_time_acc += (micros() - t0);
      
      uint32_t current_gen = pf_request_gen.load(std::memory_order_acquire);
      if (current_gen == my_gen) {
        if (success) {
          bank_cache[my_slot].bank_num = my_bank;
          pf_state.store(PF_READY, std::memory_order_release);
        } else {
          pf_state.store(PF_FREE, std::memory_order_release);
          prof.prefetch_cancellations++;
        }
      } else {
        pf_state.store(PF_FREE, std::memory_order_release);
        prof.prefetch_wasted_bytes += to_read;
        prof.prefetch_cancellations++;
      }
      if (sdTaskHandle) xTaskNotifyGive(sdTaskHandle);
      continue;
    }
    
    // 3. SAVE Priority
    SaveState s_state = save_state.load(std::memory_order_acquire);
    if (s_state == SAVE_REQUESTED) {
      save_state.store(SAVE_WRITING, std::memory_order_release);
      
      uint32_t t0 = micros();
      bool success = false;
      File f = SD.open(save_path, FILE_WRITE);
      if (f) {
        size_t written = f.write(cart_ram_buf, cart_ram_size);
        f.close();
        if (written == cart_ram_size) success = true;
      }
      prof.save_time_acc += (micros() - t0);
      
      if (success) {
        save_state.store(SAVE_DONE, std::memory_order_release);
      } else {
        save_state.store(SAVE_ERROR, std::memory_order_release);
      }
    }
  }
}

void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[160],
                   const uint_fast8_t line) {
  uint32_t t0 = micros();
  frame_drawn = true;
  uint16_t *dst = &framebuffer[line * GB_W];
  if (gb->cgb.cgbMode) {
    for (unsigned int x = 0; x < GB_W; x++)
      dst[x] = gb->cgb.fixPalette[pixels[x]];
  } else {
    for (unsigned int x = 0; x < GB_W; x++)
      dst[x] = CURRENT_PALETTE_RGB565[pixels[x] & 3];
  }
  prof.ppu_time_acc += (micros() - t0);
}

void debugPrint(const char* str) {
  // Wait for any pending render to finish before touching TFT
  while (render_buffer != NULL) { delay(1); }
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println(str);
  Serial.println(str);
}

// ═══════════════════════════════════════════════════════════════
// FILE PICKER
// ═══════════════════════════════════════════════════════════════
#define MAX_FILES  20
#define MAX_FNAME  64
static char file_names[MAX_FILES][MAX_FNAME];
static int  file_list_size = 0;

void build_file_list() {
  File root = SD.open("/");
  file_list_size = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      const char *name = entry.name();
      const char *dot = strrchr(name, '.');
      if (dot) {
        const char *ext = dot + 1;
        if ((strcasecmp(ext, "gb") == 0 || strcasecmp(ext, "gbc") == 0)
            && file_list_size < MAX_FILES) {
          strncpy(file_names[file_list_size], name, MAX_FNAME - 1);
          file_names[file_list_size][MAX_FNAME - 1] = '\0';
          file_list_size++;
        }
      }
    }
    entry.close();
  }
  root.close();
}

int file_picker() {
  build_file_list();
  if (file_list_size == 0) { debugPrint("No ROMs on SD!"); while (1); }
  int sel = 0;
  bool picked = false;

  while (!picked) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.println("Select Game:");
    tft.drawFastHLine(0, 20, 320, ILI9341_WHITE);

    int start = max(0, sel - 4);
    int end   = min(file_list_size, start + 10);
    for (int i = start; i < end; i++) {
      tft.setCursor(0, 30 + (i - start) * 20);
      if (i == sel) {
        tft.setTextColor(ILI9341_GREEN);
        tft.print("> ");
      } else {
        tft.setTextColor(ILI9341_WHITE);
        tft.print("  ");
      }
      tft.println(file_names[i]);
    }

    while (digitalRead(BTN_DOWN)==LOW || digitalRead(BTN_UP)==LOW ||
           digitalRead(BTN_A)==LOW) delay(10);
    while (1) {
      if (digitalRead(BTN_DOWN)==LOW) { sel = (sel+1) % file_list_size; break; }
      if (digitalRead(BTN_UP)==LOW)   { sel = (sel-1+file_list_size) % file_list_size; break; }
      if (digitalRead(BTN_A)==LOW)    { picked = true; break; }
      delay(50);
    }
  }
  return sel;
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
static struct gb_s *gb = NULL;
static struct priv_t priv;

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  btStop();

  // ── CRITICAL: Verify core assignment ───────────────────────
  Serial.printf("setup() running on Core %d\n", xPortGetCoreID());
  Serial.printf("CONFIG_ARDUINO_RUNNING_CORE = %d\n", CONFIG_ARDUINO_RUNNING_CORE);

  // ── Hardware init ──────────────────────────────────────────
  pinMode(BTN_UP, INPUT_PULLUP);    pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);     pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_SELECT, INPUT_PULLUP);

  // Initialize VSPI (TFT) at 40 MHz
  // PREVENT PIN CONFLICT: explicitly start SPI with -1 for unused pins
  SPI.begin(18, -1, 23, -1); // SCK=18, MISO=-1, MOSI=23, SS=-1
  tft.begin(40000000);
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  // Initialize HSPI (SD Card)
  // We do this BEFORE memory allocations so the SD library can grab 
  // pristine DMA-capable memory chunks for its FAT buffers!
  hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  
  #define SD_SPI_SPEED 4000000 
  bool ok = SD.begin(SD_CS, hspi, SD_SPI_SPEED);

  if (!ok) {
    // Robust hardware fallback: Some Arduino-ESP32 core versions overwrite 
    // custom SPI pins during SD.begin(). A hard end/begin resets the matrix.
    hspi.end();
    hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    ok = SD.begin(SD_CS, hspi, SD_SPI_SPEED);
    
    if (!ok) {
        Serial.println("SD Failed! Check wiring or formatting.");
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(10, 10);
        tft.println("SD FAILED!");
        while (1);
    }
  }
  #ifdef ESP_ARDUINO_VERSION_MAJOR
  Serial.printf("ESP Arduino Version: %d.%d.%d\n", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
#else
  Serial.println("ESP Arduino Version: < 2.0.0");
#endif

  Serial.println("SD on HSPI: OK");

  // ── 8-bit DRAM Heap diagnostic ─────────────────────────────────────────
  auto printMem = [](const char* label) {
    Serial.printf("[%s] Free: %u  Largest: %u  |  DRAM(8-bit) Free: %u  Largest: %u\n",
                  label, 
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap(),
                  heap_caps_get_free_size(MALLOC_CAP_8BIT),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  };

  printMem("Startup");

  // ── Allocate gb_s (50 KB) dynamically ────────────────────────────
  Serial.printf("sizeof(gb_s): %u\n", sizeof(struct gb_s));
  gb = (struct gb_s*)malloc(sizeof(struct gb_s));
  if (!gb) { Serial.println("OOM: gb_s"); while (1); }
  memset(gb, 0, sizeof(struct gb_s));
  printMem("After gb_s");

  // Assign global pointers to static arrays
  cache_base = rom_cache_static;
  framebuffer = framebuffer_static;

  memset(bank_to_slot, -1, sizeof(bank_to_slot));
  for (int i = 0; i < CACHE_SLOTS; i++) {
    bank_cache[i].data = cache_base + i * BANK_SIZE;
    bank_cache[i].bank_num = -1;
  }
  hot_bank_slot = 1;
  prev_bank_slot = 2;
  printMem("After all setup allocations");

  // ── Hardware init ──────────────────────────────────────────
  pinMode(BTN_UP, INPUT_PULLUP);    pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);     pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_SELECT, INPUT_PULLUP);

  // SD and TFT are now initialized at the top of setup()

  // ── ROM selection ──────────────────────────────────────────
  int sel_idx = file_picker();

  char sel_path[100];
  snprintf(sel_path, sizeof(sel_path), "/%s", file_names[sel_idx]);

  strncpy(save_path, sel_path, sizeof(save_path));
  char *dot = strrchr(save_path, '.');
  if (dot) strcpy(dot, ".sav");

  rom_file = SD.open(sel_path);
  if (!rom_file) { debugPrint("ROM open fail!"); while (1); }
  rom_total_size = rom_file.size();

  debugPrint("Loading...");

  // 6. Read ROM bank 0 from SD into bank_cache[0].data
  uint32_t to_read = BANK_SIZE;
  if (to_read > rom_total_size) to_read = rom_total_size;
  rom_file.seek(0);
  size_t bytes_read = rom_file.read(bank_cache[0].data, to_read);
  
  // 7. Verify the read length
  if (bytes_read < to_read) {
    Serial.printf("FATAL: Failed to read ROM bank 0. Expected %u, got %u\n", to_read, bytes_read);
    while(1) { taskYIELD(); }
  }

  // 8. Set bank info
  bank_cache[0].bank_num = 0;
  bank_to_slot[0] = 0;

  hot_bank_num = -1;
  hot_bank_slot = -1;
  hot_bank_ptr = NULL;
  prev_bank_slot = -1;

  // 9. ONLY NOW call gb_init()
  enum gb_init_error_e init_err = gb_init(gb, &gb_rom_read, &gb_rom_read_16bit, &gb_rom_read_32bit,
          &gb_cart_ram_read, &gb_cart_ram_write, &gb_error, &priv);
  
  // 10. Check gb_init() return value
  if (init_err != GB_INIT_NO_ERROR) {
    Serial.printf("FATAL: gb_init failed with error code %d\n", init_err);
    while(1) { taskYIELD(); }
  }

  // 11. call gb_get_save_size()
  cart_ram_size = gb_get_save_size(gb);
  
  // 12. allocate/load cart RAM
  if (cart_ram_size > 0) {
    cart_ram_buf = (uint8_t*)malloc(cart_ram_size);
    if (!cart_ram_buf && cart_ram_size > 8192) {
      Serial.printf("WARN: Save %u too big, using 8KB\n", cart_ram_size);
      cart_ram_size = 8192;
      cart_ram_buf = (uint8_t*)malloc(cart_ram_size);
    }
    if (cart_ram_buf) load_cart_ram();
    else { Serial.println("WARN: No save RAM"); cart_ram_size = 0; }
  }

  // 13. call gb_init_lcd()
  gb_init_lcd(gb, &lcd_draw_line);
  gb->direct.interlace = 0;
  
  // 14. set gb_rom_bank_changed callback
  gb->gb_rom_bank_changed = &gb_rom_bank_changed_callback;

  // 15/16. Launch Core 1 tasks
  xTaskCreatePinnedToCore(renderTask, "Render", 3072, NULL, 2, &renderTaskHandle, 1);
  xTaskCreatePinnedToCore(sdTask, "SDTask", 2560, NULL, 1, &sdTaskHandle, 1);

  tft.fillScreen(ILI9341_BLACK);

  // --- DIAGNOSTIC PREFLIGHT ---
  Serial.println("\n--- PREFLIGHT CHECK ---");
  Serial.printf("gb->gb_rom_read: %p\n", gb->gb_rom_read);
  Serial.printf("gb->gb_rom_read_16bit: %p\n", gb->gb_rom_read_16bit);
  Serial.printf("gb->gb_rom_read_32bit: %p\n", gb->gb_rom_read_32bit);
  Serial.printf("gb->gb_cart_ram_read: %p\n", gb->gb_cart_ram_read);
  Serial.printf("gb->gb_cart_ram_write: %p\n", gb->gb_cart_ram_write);
  Serial.printf("gb->gb_error: %p\n", gb->gb_error);
  Serial.printf("gb->gb_rom_bank_changed: %p\n", gb->gb_rom_bank_changed);
  Serial.printf("gb->display.lcd_draw_line: %p\n", gb->display.lcd_draw_line);
  
  Serial.printf("gb pointer: %p\n", gb);
  Serial.printf("bank_cache[0].data: %p\n", bank_cache[0].data);
  Serial.printf("bank_cache[0].bank_num: %d\n", bank_cache[0].bank_num);
  Serial.printf("bank_to_slot[0]: %d\n", bank_to_slot[0]);
  Serial.printf("hot_bank_num: %d\n", hot_bank_num);
  Serial.printf("hot_bank_slot: %d\n", hot_bank_slot);
  Serial.printf("prev_bank_slot: %d\n", prev_bank_slot);

  uint8_t v = gb_rom_read(gb, 0x0100);
  Serial.printf("ROM[0100] = %02X\n", v);

  uint16_t v16 = gb_rom_read_16bit(gb, 0x0100);
  Serial.printf("ROM16[0100] = %04X\n", v16);

  Serial.print("ROM Bytes [0x0100]: ");
  for(int i=0; i<8; i++) Serial.printf("%02X ", gb_rom_read(gb, 0x0100 + i));
  Serial.println();
  Serial.println("Preflight complete.\n");

  // 17. Launch emulation task EXPLICITLY on Core 0
  xTaskCreatePinnedToCore(emuTask, "Emulator", 8192, NULL, 1, NULL, 0);
  Serial.printf("All tasks launched. Render=Core1 SD=Core1 Emu=Core0\n");
}

// ═══════════════════════════════════════════════════════════════
// EMULATION TASK — Explicitly pinned to Core 0
// ═══════════════════════════════════════════════════════════════
// We do NOT use Arduino's loop() because it runs on Core 1 by
// default (CONFIG_ARDUINO_RUNNING_CORE=1 in most board packages).
// That would put the emulator, TFT task, and SD task all on
// the same core, destroying our dual-core architecture.
void emuTask(void *pvParameters) {
  Serial.printf("emuTask running on Core %d\n", xPortGetCoreID());
  reset_prof();

  for (;;) {
    uint32_t loop_start = micros();
    frame_count++;

    // Wait for Core 1 to finish previous TFT push before
    // writing to the framebuffer.
    uint32_t wait_start = micros();
    while (render_buffer != NULL) { taskYIELD(); }
    prof.core0_wait_acc += (micros() - wait_start);

    // ── Joypad ─────────────────────────────────────────────────
    gb->direct.joypad = 0xFF;
    if (digitalRead(BTN_UP)==LOW)     gb->direct.joypad_bits.up     = 0;
    if (digitalRead(BTN_DOWN)==LOW)   gb->direct.joypad_bits.down   = 0;
    if (digitalRead(BTN_LEFT)==LOW)   gb->direct.joypad_bits.left   = 0;
    if (digitalRead(BTN_RIGHT)==LOW)  gb->direct.joypad_bits.right  = 0;
    if (digitalRead(BTN_A)==LOW)      gb->direct.joypad_bits.a      = 0;
    if (digitalRead(BTN_B)==LOW)      gb->direct.joypad_bits.b      = 0;
    if (digitalRead(BTN_START)==LOW)  gb->direct.joypad_bits.start  = 0;
    if (digitalRead(BTN_SELECT)==LOW) gb->direct.joypad_bits.select = 0;

    // ── Emulate one frame ──────────────────────────────────────
    uint32_t emu_start = micros();
    frame_drawn = false;
    gb_run_frame_dualfetch(gb);
    prof.emu_time_acc += (micros() - emu_start);

    // ── Hand off to Core 1 for TFT push ───────────────────────
    if (frame_drawn) {
      render_buffer = framebuffer;
      xTaskNotifyGive(renderTaskHandle);
    }

    // ── Auto-save (Core 0 requests, Core 1 executes) ──────────
    if (frame_count % SAVE_INTERVAL_FRAMES == 0 && cart_ram_dirty) {
      save_cart_ram();
    }

    prof.loop_time_acc += (micros() - loop_start);
    prof.frames++;
    
    uint32_t fh = ESP.getFreeHeap();
    if (fh < prof.min_free_heap) prof.min_free_heap = fh;

    if (prof.frames >= 300) {
      char buf[512];
      snprintf(buf, sizeof(buf),
        "\n=== PROFILING (300 frames) ===\n"
        "Emu Core: %d\n"
        "Emulation: %u ms/f\n"
        "PPU (draw): %u ms/f\n"
        "Core0 Wait: %u ms/f\n"
        "TFT Push: %u ms/f\n"
        "SD Misses: %u\n"
        "SD Time: %u ms/miss\n"
        "Dem Reqs: %u\n"
        "Dem Wait: %u ms/f\n"
        "PF Reqs: %u\n"
        "PF Hits: %u\n"
        "PF Cancels: %u\n"
        "PF Fallbacks: %u\n"
        "PF Wait: %u ms/f\n"
        "PF Wasted: %u KB\n"
        "Save Time: %u ms\n"
        "Loop Total: %u ms/f (%.1f FPS)\n"
        "Min Free: %u\n"
        "Max Block: %u\n"
        "C1 Render Stack: %u\n"
        "C1 SD Stack: %u\n"
        "==============================",
        xPortGetCoreID(),
        prof.emu_time_acc / 300000,
        prof.ppu_time_acc / 300000,
        prof.core0_wait_acc / 300000,
        prof.tft_time_acc / 300000,
        prof.cache_misses,
        prof.cache_misses ? (prof.sd_time_acc / 1000) / prof.cache_misses : 0,
        prof.demand_requests,
        prof.demand_wait_time_acc / 300000,
        prof.prefetch_requests,
        prof.prefetch_hits,
        prof.prefetch_cancellations,
        prof.prefetch_sync_fallbacks,
        prof.prefetch_wait_time_acc / 300000,
        prof.prefetch_wasted_bytes / 1024,
        (prof.save_time_acc / 1000),
        prof.loop_time_acc / 300000,
        300000000.0f / prof.loop_time_acc,
        prof.min_free_heap,
        ESP.getMaxAllocHeap(),
        uxTaskGetStackHighWaterMark(renderTaskHandle),
        uxTaskGetStackHighWaterMark(sdTaskHandle)
      );
      Serial.println(buf);
      reset_prof();
    }
  }
}

// Arduino loop() is intentionally empty.
// The emulation runs in emuTask pinned to Core 0.
void loop() {
  vTaskDelay(portMAX_DELAY);
}

