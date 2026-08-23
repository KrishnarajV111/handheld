#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "driver/spi_master.h"
#include "esp_task_wdt.h"

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
  uint32_t yield_count;
  uint32_t pf_predictor_candidates;
  uint32_t pf_predictor_accepted;
};
static ProfileData prof;

static void reset_prof() {
  prof.emu_time_acc = 0;
  prof.ppu_time_acc = 0;
  prof.core0_wait_acc = 0;
  prof.tft_time_acc = 0;
  prof.sd_time_acc = 0;
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
  prof.min_free_heap = 0xFFFFFFFF;
  prof.yield_count = 0;
  prof.pf_predictor_candidates = 0;
  prof.pf_predictor_accepted = 0;
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

// DMG palette (Crisp high-contrast: White, Light Gray, Dark Gray, Black)
uint16_t CURRENT_PALETTE_RGB565[4] = { 0xFFFF, 0xAD55, 0x52AA, 0x0000 };

// ═══════════════════════════════════════════════════════════════
// 2-WAY SET-ASSOCIATIVE 1KB PAGE CACHE (24 SLOTS + FIXED BANK 0)
// ═══════════════════════════════════════════════════════════════
#define PAGE_SIZE       1024
#define PAGE_MASK       0x3FF
#define PAGE_SHIFT      10
#define NUM_SETS        12
#define WAYS            2
#define NUM_PAGE_SLOTS  (NUM_SETS * WAYS) // 24 dynamic slots (24 KB)
#define MAX_PAGES       2048

static uint8_t *bank0_rom = NULL; // Fixed 16KB Bank 0 in RAM

struct PageSlot {
  uint8_t *data;
  int16_t  page_num;
  uint8_t  last_used;
};

static PageSlot page_slots[NUM_PAGE_SLOTS];
static uint8_t access_counter = 0;
static struct gb_s *gb = NULL;

#include <atomic>

// Demand State for 1KB Page
enum DemandState { DEMAND_FREE = 0, DEMAND_REQUESTED = 1, DEMAND_LOADING = 2, DEMAND_READY = 3, DEMAND_ERROR = 4 };
static std::atomic<DemandState> dem_state(DEMAND_FREE);
static std::atomic<uint32_t> dem_gen(0);
static std::atomic<int> dem_page(-1);
static std::atomic<int> dem_slot(-1);

// Save State
enum SaveState { SAVE_FREE = 0, SAVE_REQUESTED = 1, SAVE_WRITING = 2, SAVE_DONE = 3, SAVE_ERROR = 4 };
static std::atomic<SaveState> save_state(SAVE_FREE);

static File     rom_file;
static uint32_t rom_total_size;
static uint32_t frame_count = 0;
static TaskHandle_t sdTaskHandle = NULL;

static uint8_t* cache_miss_page(int page_num, int victim_slot) {
  prof.cache_misses++;

  page_slots[victim_slot].page_num = -1;

  // Issue DEMAND for 1024 bytes to Core 1 sdTask
  dem_page.store(page_num, std::memory_order_relaxed);
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
    if ((micros() - t_wait) > 150000) {
      Serial.println("FATAL: SD DEMAND timeout");
      while (1) { taskYIELD(); }
    }
    taskYIELD();
  }
  prof.demand_wait_time_acc += (micros() - t_wait);
  dem_state.store(DEMAND_FREE, std::memory_order_release);

  page_slots[victim_slot].page_num = page_num;
  page_slots[victim_slot].last_used = ++access_counter;

  return page_slots[victim_slot].data;
}

static inline uint8_t* IRAM_ATTR get_page_ptr(uint32_t page_num) {
  int set = page_num % NUM_SETS;
  int slot0 = set * 2;
  int slot1 = slot0 + 1;

  if (page_slots[slot0].page_num == (int16_t)page_num) {
    page_slots[slot0].last_used = ++access_counter;
    return page_slots[slot0].data;
  }
  if (page_slots[slot1].page_num == (int16_t)page_num) {
    page_slots[slot1].last_used = ++access_counter;
    return page_slots[slot1].data;
  }

  int victim = (page_slots[slot0].last_used <= page_slots[slot1].last_used) ? slot0 : slot1;
  return cache_miss_page(page_num, victim);
}

uint8_t IRAM_ATTR gb_rom_read(struct gb_s *gb, const uint_fast32_t addr) {
  if (addr < 16384) {
    return bank0_rom[addr];
  }
  uint32_t page_num = addr >> PAGE_SHIFT;
  uint32_t offset = addr & PAGE_MASK;
  uint8_t *pdata = get_page_ptr(page_num);
  return pdata[offset];
}

uint16_t IRAM_ATTR gb_rom_read_16bit(struct gb_s *gb, const uint_fast32_t addr) {
  if (addr < 16383) {
    return (uint16_t)bank0_rom[addr] | ((uint16_t)bank0_rom[addr + 1] << 8);
  }
  uint32_t page_num = addr >> PAGE_SHIFT;
  uint32_t offset = addr & PAGE_MASK;
  if (offset < PAGE_SIZE - 1) {
    uint8_t *p = get_page_ptr(page_num);
    return (uint16_t)p[offset] | ((uint16_t)p[offset + 1] << 8);
  }
  return (uint16_t)gb_rom_read(gb, addr) | ((uint16_t)gb_rom_read(gb, addr + 1) << 8);
}

uint32_t IRAM_ATTR gb_rom_read_32bit(struct gb_s *gb, const uint_fast32_t addr) {
  if (addr < 16381) {
    return (uint32_t)bank0_rom[addr] | ((uint32_t)bank0_rom[addr + 1] << 8) |
           ((uint32_t)bank0_rom[addr + 2] << 16) | ((uint32_t)bank0_rom[addr + 3] << 24);
  }
  uint32_t page_num = addr >> PAGE_SHIFT;
  uint32_t offset = addr & PAGE_MASK;
  if (offset < PAGE_SIZE - 3) {
    uint8_t *p = get_page_ptr(page_num);
    return (uint32_t)p[offset] | ((uint32_t)p[offset + 1] << 8) |
           ((uint32_t)p[offset + 2] << 16) | ((uint32_t)p[offset + 3] << 24);
  }
  return (uint32_t)gb_rom_read(gb, addr) | ((uint32_t)gb_rom_read(gb, addr + 1) << 8) |
         ((uint32_t)gb_rom_read(gb, addr + 2) << 16) | ((uint32_t)gb_rom_read(gb, addr + 3) << 24);
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

#define TFT_W     240
#define TFT_H     320

static uint8_t scale_lut_x[TFT_W];
static uint8_t scale_lut_y[TFT_H];

static uint16_t *framebuffer = NULL;
static bool frame_drawn = false;

static uint16_t * volatile render_buffer = NULL;
static TaskHandle_t renderTaskHandle = NULL;

void gb_rom_bank_changed_callback(struct gb_s* gb, uint16_t new_bank) {
  // Unused in 1KB Sector Page Cache mode (pages are fetched dynamically on-demand)
}

// ═══════════════════════════════════════════════════════════════
// NATIVE 1:1 TFT RENDERER (160x144 CENTERED ON 240x320)
// ═══════════════════════════════════════════════════════════════
#define GB_OFFSET_X  40  // (240 - 160) / 2
#define GB_OFFSET_Y  88  // (320 - 144) / 2

// Core 1 High-Priority Task: TFT VSPI (Native 1:1 Pixel Mapping)
void renderTask(void *pvParameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (render_buffer) {
      uint32_t t0 = micros();
      
      tft.startWrite();
      tft.setAddrWindow(GB_OFFSET_X, GB_OFFSET_Y, GB_W, GB_H);
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
    
    // 1. DEMAND Priority (1024-byte page)
    DemandState d_state = dem_state.load(std::memory_order_acquire);
    if (d_state == DEMAND_REQUESTED) {
      uint32_t my_gen = dem_gen.load(std::memory_order_relaxed);
      int my_slot = dem_slot.load(std::memory_order_relaxed);
      int my_page = dem_page.load(std::memory_order_relaxed);
      
      dem_state.store(DEMAND_LOADING, std::memory_order_release);
      
      uint32_t offset = (uint32_t)my_page * PAGE_SIZE;
      size_t to_read = PAGE_SIZE;
      if (offset + to_read > rom_total_size) {
        to_read = (offset < rom_total_size) ? (rom_total_size - offset) : 0;
      }
      
      uint32_t t0 = micros();
      bool success = false;
      if (to_read > 0) {
        if (rom_file.position() != offset) {
          success = rom_file.seek(offset);
        } else {
          success = true;
        }
        if (success) {
          size_t read_bytes = rom_file.read(page_slots[my_slot].data, to_read);
          if (read_bytes == to_read) success = true;
          else success = false;
        }
      } else {
        memset(page_slots[my_slot].data, 0xFF, PAGE_SIZE);
        success = true;
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

static uint8_t color_mode = 3; // Default to Mode 3 (Direct Little-Endian BGR with Contrast Boost)

// High-contrast, reduced-glare gamma curves (deep blacks, rich midtones, non-blinding highlights)
static const uint8_t contrast_lut5[32] = {
   0,  0,  1,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 13, 14,
  15, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 27, 28, 28, 29, 29
};

static const uint8_t contrast_lut6[64] = {
   0,  0,  1,  2,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 13, 14,
  15, 16, 17, 19, 20, 21, 23, 24, 25, 27, 28, 29, 31, 32, 33, 35,
  36, 37, 39, 40, 41, 43, 44, 45, 47, 48, 49, 50, 51, 52, 53, 54,
  55, 56, 57, 57, 58, 58, 59, 59, 60, 60, 61, 61, 61, 62, 62, 62
};

void IRAM_ATTR lcd_draw_line(struct gb_s *gb, const uint8_t pixels[160],
                   const uint_fast8_t line) {
  uint32_t t0 = micros();
  frame_drawn = true;
  uint16_t *dst = &framebuffer[line * GB_W];
  
  if (gb->cgb.cgbMode) {
    for (unsigned int x = 0; x < GB_W; x++) {
      uint16_t c = gb->cgb.fixPalette[pixels[x]];
      switch (color_mode) {
        case 0: // Big-Endian RGB
          dst[x] = (c >> 8) | (c << 8);
          break;
        case 1: { // Big-Endian BGR
          uint16_t r = (c >> 11) & 0x1F;
          uint16_t g = (c >> 5) & 0x3F;
          uint16_t b = c & 0x1F;
          uint16_t sw = (b << 11) | (g << 5) | r;
          dst[x] = (sw >> 8) | (sw << 8);
          break;
        }
        case 2: // Direct Little-Endian RGB
          dst[x] = c;
          break;
        case 3: { // Direct Little-Endian BGR + High Contrast & Glare Reduction
          uint16_t r = contrast_lut5[(c >> 11) & 0x1F];
          uint16_t g = contrast_lut6[(c >> 5) & 0x3F];
          uint16_t b = contrast_lut5[c & 0x1F];
          dst[x] = (b << 11) | (g << 5) | r;
          break;
        }
      }
    }
  } else {
    for (unsigned int x = 0; x < GB_W; x++) {
      uint16_t c = CURRENT_PALETTE_RGB565[pixels[x] & 3];
      if (color_mode == 0 || color_mode == 1) {
        dst[x] = (c >> 8) | (c << 8);
      } else {
        dst[x] = c;
      }
    }
  }
  prof.ppu_time_acc += (micros() - t0);
}

void debugPrint(const char* str) {
  // Wait for any pending render to finish before touching TFT
  while (render_buffer != NULL) { delay(1); }
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(10, 140);
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
    tft.setCursor(10, 10);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setTextSize(2);
    tft.println("Select Game:");
    tft.drawFastHLine(0, 32, TFT_W, ILI9341_WHITE);

    int start = max(0, sel - 5);
    int end   = min(file_list_size, start + 12);
    for (int i = start; i < end; i++) {
      tft.setCursor(10, 44 + (i - start) * 22);
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
static struct priv_t priv;

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  btStop();

  // ── CRITICAL: Verify core assignment ───────────────────────
  Serial.printf("setup() running on Core %d\n", xPortGetCoreID());
  Serial.printf("CONFIG_ARDUINO_RUNNING_CORE = %d\n", CONFIG_ARDUINO_RUNNING_CORE);

  // ── Hardware init ──────────────────────────────────────────
  for (int x = 0; x < TFT_W; x++) scale_lut_x[x] = (uint8_t)((x * GB_W) / TFT_W); // 1.5x horizontal stretch (160 -> 240)
  for (int y = 0; y < TFT_H; y++) scale_lut_y[y] = (uint8_t)((y * GB_H) / TFT_H); // 2.22x vertical stretch (144 -> 320)

  pinMode(BTN_UP, INPUT_PULLUP);    pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);     pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_SELECT, INPUT_PULLUP);

  // Initialize VSPI (TFT) at true 40 MHz hardware SPI
  SPI.begin(18, -1, 23, -1); // SCK=18, MISO=-1, MOSI=23, SS=-1
  tft.begin(40000000);
  tft.setRotation(2); // Flipped 180° Portrait

  Serial.printf("TFT geometry: %d x %d (ILI9341 240x320 Flipped Mode @ 40MHz)\n", tft.width(), tft.height());
  tft.fillScreen(ILI9341_BLACK);

  // Initialize HSPI (SD Card) at true 20 MHz hardware SPI
  hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  
  #define SD_SPI_SPEED 20000000 
  bool ok = SD.begin(SD_CS, hspi, SD_SPI_SPEED);

  if (!ok) {
    // Hardware fallback: Retry at 10 MHz
    hspi.end();
    hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    ok = SD.begin(SD_CS, hspi, 10000000);
    
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

  // ── Allocate largest buffers FIRST to eliminate DRAM heap fragmentation ─────
  Serial.printf("sizeof(gb_s): %u\n", sizeof(struct gb_s));
  gb = (struct gb_s*)heap_caps_malloc(sizeof(struct gb_s), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!gb) { Serial.println("OOM: gb_s"); while (1); }
  memset(gb, 0, sizeof(struct gb_s));

  framebuffer = (uint16_t*)heap_caps_malloc(FB_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!framebuffer) { Serial.println("OOM: framebuffer"); while (1); }
  memset(framebuffer, 0, FB_BYTES);

  bank0_rom = (uint8_t*)heap_caps_malloc(16384, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!bank0_rom) { Serial.println("OOM: bank0_rom"); while (1); }

  for (int i = 0; i < NUM_PAGE_SLOTS; i++) {
    page_slots[i].data = (uint8_t*)heap_caps_malloc(PAGE_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!page_slots[i].data) { Serial.printf("OOM: page slot %d\n", i); while (1); }
    page_slots[i].page_num = -1;
    page_slots[i].last_used = 0;
  }
  printMem("After core allocations");

  // ── Hardware init ──────────────────────────────────────────
  pinMode(BTN_UP, INPUT_PULLUP);    pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);     pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_SELECT, INPUT_PULLUP);

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

  // 6. Permanently load Bank 0 (16KB)
  rom_file.seek(0);
  size_t b0_rd = rom_file.read(bank0_rom, 16384);
  Serial.printf("[BOOT] Bank 0 permanently loaded into RAM (%u bytes)\n", b0_rd);

  // 7. Pre-populate dynamic 1KB pages (Pages 16..39)
  for (int s = 0; s < NUM_PAGE_SLOTS; s++) {
    int p = 16 + s; // Starts from 16KB offset (Bank 1)
    uint32_t offset = (uint32_t)p * PAGE_SIZE;
    if (offset < rom_total_size) {
      rom_file.seek(offset);
      size_t rd = rom_file.read(page_slots[s].data, PAGE_SIZE);
      if (rd > 0) {
        page_slots[s].page_num = p;
        page_slots[s].last_used = (uint8_t)s;
      }
    }
  }
  Serial.printf("[BOOT] Pre-loaded %d dynamic 1KB pages into cache\n", NUM_PAGE_SLOTS);

  // 9. ONLY NOW call gb_init()
  enum gb_init_error_e init_err = gb_init(gb, &gb_rom_read, &gb_rom_read_16bit, &gb_rom_read_32bit,
          &gb_cart_ram_read, &gb_cart_ram_write, &gb_error, &priv);
  
  // 10. Check gb_init() return value
  if (init_err != GB_INIT_NO_ERROR) {
    Serial.printf("FATAL: gb_init failed with error code %d\n", init_err);
    while(1) { taskYIELD(); }
  }

  // 13. call gb_init_lcd()
  gb_init_lcd(gb, &lcd_draw_line);
  gb->direct.interlace = 0;
  gb->direct.frame_skip = gb->cgb.cgbMode ? 1 : 0;
  
  // Clear display to black before game start
  tft.fillScreen(ILI9341_BLACK);

  // 14. Launch all FreeRTOS tasks FIRST while DRAM heap is unfragmented
  xTaskCreatePinnedToCore(renderTask, "Render", 2560, NULL, 2, &renderTaskHandle, 1);
  xTaskCreatePinnedToCore(sdTask, "SDTask", 2560, NULL, 1, &sdTaskHandle, 1);
  BaseType_t res = xTaskCreatePinnedToCore(emuTask, "Emulator", 4096, NULL, 1, NULL, 0);
  Serial.printf("emuTask create result: %d (1=OK)\n", res);

  // 15. Allocate and load Cart RAM
  cart_ram_size = gb_get_save_size(gb);
  if (cart_ram_size > 0) {
    cart_ram_buf = (uint8_t*)heap_caps_malloc(cart_ram_size, MALLOC_CAP_8BIT);
    if (cart_ram_buf) {
      load_cart_ram();
      Serial.printf("[SAVE] Cart RAM allocated (%u bytes)\n", cart_ram_size);
    } else {
      Serial.println("WARN: No save RAM available");
      cart_ram_size = 0;
    }
  }

  // --- DIAGNOSTIC PREFLIGHT ---
  Serial.println("\n--- PREFLIGHT CHECK ---");
  Serial.printf("gb->gb_rom_read: %p\n", gb->gb_rom_read);
  Serial.printf("gb->gb_rom_read_16bit: %p\n", gb->gb_rom_read_16bit);
  Serial.printf("gb->gb_rom_read_32bit: %p\n", gb->gb_rom_read_32bit);
  Serial.printf("gb->gb_cart_ram_read: %p\n", gb->gb_cart_ram_read);
  Serial.printf("gb->gb_cart_ram_write: %p\n", gb->gb_cart_ram_write);
  Serial.printf("gb->gb_error: %p\n", gb->gb_error);
  Serial.printf("gb->display.lcd_draw_line: %p\n", gb->display.lcd_draw_line);
  Serial.printf("gb pointer: %p\n", gb);
  Serial.printf("bank0_rom pointer: %p\n", bank0_rom);

  uint8_t v = gb_rom_read(gb, 0x0100);
  Serial.printf("ROM[0100] = %02X\n", v);

  uint16_t v16 = gb_rom_read_16bit(gb, 0x0100);
  Serial.printf("ROM16[0100] = %04X\n", v16);

  Serial.print("ROM Bytes [0x0100]: ");
  for(int i=0; i<8; i++) Serial.printf("%02X ", gb_rom_read(gb, 0x0100 + i));
  Serial.println();
  Serial.println("Preflight complete.\n");
  Serial.printf("All tasks launched. Render=Core1 SD=Core1 Emu=Core0\n");
}

// ═══════════════════════════════════════════════════════════════
// EMULATION TASK — Explicitly pinned to Core 0
// ═══════════════════════════════════════════════════════════════
void emuTask(void *pvParameters) {
  Serial.printf("emuTask running on Core %d\n", xPortGetCoreID());
  esp_task_wdt_delete(NULL);
  reset_prof();

  for (;;) {
    uint32_t loop_start = micros();
    frame_count++;

    // Wait for Core 1 to finish previous TFT push before
    // writing to the framebuffer.
    uint32_t wait_start = micros();
    while (render_buffer != NULL) { 
      taskYIELD(); 
      prof.yield_count++; 
    }
    prof.core0_wait_acc += (micros() - wait_start);

    // ── Joypad ─────────────────────────────────────────────────
    gb->direct.joypad = 0xFF;
    bool b_up     = (digitalRead(BTN_UP)==LOW);
    bool b_down   = (digitalRead(BTN_DOWN)==LOW);
    bool b_left   = (digitalRead(BTN_LEFT)==LOW);
    bool b_right  = (digitalRead(BTN_RIGHT)==LOW);
    bool b_a      = (digitalRead(BTN_A)==LOW);
    bool b_b      = (digitalRead(BTN_B)==LOW);
    bool b_start  = (digitalRead(BTN_START)==LOW);
    bool b_select = (digitalRead(BTN_SELECT)==LOW);

    if (b_up)     gb->direct.joypad_bits.up     = 0;
    if (b_down)   gb->direct.joypad_bits.down   = 0;
    if (b_left)   gb->direct.joypad_bits.left   = 0;
    if (b_right)  gb->direct.joypad_bits.right  = 0;
    if (b_a)      gb->direct.joypad_bits.a      = 0;
    if (b_b)      gb->direct.joypad_bits.b      = 0;
    if (b_start)  gb->direct.joypad_bits.start  = 0;
    if (b_select) gb->direct.joypad_bits.select = 0;

    // ── Live Color Mode Cycler (Press START + SELECT together) ─
    static bool combo_prev = false;
    bool combo_now = (b_start && b_select);
    if (combo_now && !combo_prev) {
      color_mode = (color_mode + 1) % 4;
      Serial.printf("\n>>> Switched Color Profile to Mode %d <<<\n", color_mode);
    }
    combo_prev = combo_now;

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

    // ── Advance Real-Time Clock (MBC3) every frame ────────────
    if (gb->mbc == 3) {
      gb_tick_rtc(gb);
    }

    // Ultra-low latency yield (no 1ms delay every frame)
    if (frame_count % 15 == 0) {
      vTaskDelay(1);
    } else {
      taskYIELD();
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
        "PF Predictor: Cands=%u Acc=%u\n"
        "PF Reqs: %u\n"
        "PF Hits: %u\n"
        "PF Cancels: %u\n"
        "PF Fallbacks: %u\n"
        "PF Wait: %u ms/f\n"
        "PF Wasted: %u KB\n"
        "Save Time: %u ms\n"
        "Yields: %u\n"
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
        prof.pf_predictor_candidates,
        prof.pf_predictor_accepted,
        prof.prefetch_requests,
        prof.prefetch_hits,
        prof.prefetch_cancellations,
        prof.prefetch_sync_fallbacks,
        prof.prefetch_wait_time_acc / 300000,
        prof.prefetch_wasted_bytes / 1024,
        (prof.save_time_acc / 1000),
        prof.yield_count,
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

