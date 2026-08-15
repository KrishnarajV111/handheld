#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <WiFi.h>

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
  uint32_t prefetch_hits;
  uint32_t prefetch_aborts;
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
  prof.cache_misses = 0;
  prof.prefetch_hits = 0;
  prof.prefetch_aborts = 0;
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
// Core 0 can read from SD (HSPI) at the exact same moment that
// Core 1 pushes pixels to the TFT (VSPI). True parallel I/O.
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
// │    MOSI        →  GPIO 16   ← (was 23, MOVE THIS)      │
// │    MISO        →  GPIO 17   ← (was 19, MOVE THIS)      │
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
#define SD_MOSI  16
#define SD_MISO  17
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
#define CACHE_SLOTS  4
#define MAX_BANKS    512

struct CacheSlot {
  uint8_t  *data;
  int16_t  bank_num;
};

static CacheSlot bank_cache[CACHE_SLOTS];
static uint8_t  *cache_base = NULL;
static int8_t    bank_to_slot[MAX_BANKS];

// [PHASE 3] Cartridge-aware replacement: Active & Prev slots
static uint8_t *hot_bank_ptr  = NULL;
static int      hot_bank_num  = -1;
static int      hot_bank_slot = -1;
static int      prev_bank_slot = -1;

// [PHASE 5] Predictive Prefetch State
enum PrefetchState { PF_FREE = 0, PF_LOADING = 1, PF_READY = 2 };
static volatile PrefetchState pf_state = PF_FREE;
static volatile int pf_request_bank = -1;
static volatile int pf_active_slot = -1;

static File     rom_file;
static uint32_t rom_total_size;
static uint32_t frame_count = 0;

// SD reads go through HSPI. Core 1 uses VSPI for TFT.
// No mutex needed — they are physically separate buses!
static void cache_load_bank(int slot, int bank) {
  int old_bank = bank_cache[slot].bank_num;
  if (old_bank >= 0 && old_bank < MAX_BANKS)
    bank_to_slot[old_bank] = -1;

  uint32_t offset = (uint32_t)bank * BANK_SIZE;
  size_t to_read = BANK_SIZE;
  if (offset + to_read > rom_total_size) {
    to_read = rom_total_size - offset;
    memset(bank_cache[slot].data + to_read, 0xFF, BANK_SIZE - to_read);
  }

  uint32_t t0 = micros();
  rom_file.seek(offset);
  rom_file.read(bank_cache[slot].data, to_read);
  prof.sd_time_acc += (micros() - t0);

  bank_cache[slot].bank_num = bank;
  if (bank < MAX_BANKS) bank_to_slot[bank] = slot;
}

// [PHASE 4] ROM Access Hot-Path: Minimal branches, manually inlined.
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
  if (pf_request_bank == bank) {
    if (pf_state == PF_READY && pf_active_slot > 0) {
      prof.prefetch_hits++;
      int slot = pf_active_slot;
      bank_to_slot[bank] = slot;
      pf_state = PF_FREE; // Slot is officially part of normal cache now
      
      prev_bank_slot = hot_bank_slot;
      hot_bank_ptr = bank_cache[slot].data;
      hot_bank_num = bank;
      hot_bank_slot = slot;
      return hot_bank_ptr;
    } else if (pf_state == PF_LOADING) {
      // Core 1 is currently loading it. Wait a tiny bit (synchronous fallback).
      uint32_t t0 = micros();
      while (pf_state == PF_LOADING && (micros() - t0) < 3000) { taskYIELD(); }
      
      if (pf_state == PF_READY) {
        prof.prefetch_hits++;
        int slot = pf_active_slot;
        bank_to_slot[bank] = slot;
        pf_state = PF_FREE;
        
        prev_bank_slot = hot_bank_slot;
        hot_bank_ptr = bank_cache[slot].data;
        hot_bank_num = bank;
        hot_bank_slot = slot;
        return hot_bank_ptr;
      } else {
        prof.prefetch_aborts++;
      }
    }
  }

  // [PHASE 3] Deterministic Cartridge-Aware Replacement
  // Exclude Bank 0 (Slot 0), Active Bank, Prev Bank, and any slot actively LOADING by Core 1.
  int victim_slot = 1;
  for (int i = 1; i < CACHE_SLOTS; i++) {
    if (i != hot_bank_slot && i != prev_bank_slot) {
      if (pf_state == PF_LOADING && i == pf_active_slot) continue;
      victim_slot = i;
      break;
    }
  }
  
  cache_load_bank(victim_slot, bank);
  
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
// CART RAM — All SD access on Core 0 via HSPI
// ═══════════════════════════════════════════════════════════════
#define SAVE_INTERVAL_FRAMES 300
static char save_path[100];
static uint8_t *cart_ram_buf = NULL;
static size_t   cart_ram_size = 0;
static bool     cart_ram_dirty = false;

void save_cart_ram() {
  if (!cart_ram_dirty || cart_ram_size == 0) return;
  uint32_t t0 = micros();
  File f = SD.open(save_path, FILE_WRITE);
  if (f) { f.write(cart_ram_buf, cart_ram_size); f.close(); }
  prof.save_time_acc += (micros() - t0);
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
// Core 0 emulates and writes to the framebuffer.
// Core 1 pushes the completed framebuffer to the TFT.
// Because TFT (VSPI) and SD (HSPI) are on separate physical
// buses, a cache miss on Core 0 NEVER blocks the TFT push,
// and the TFT push NEVER blocks an SD read. True parallel I/O.
#define GB_W      160
#define GB_H      144
#define FB_PIXELS (GB_W * GB_H)
#define FB_BYTES  (FB_PIXELS * 2)
#define OFFSET_X  ((320 - GB_W) / 2)
#define OFFSET_Y  ((240 - GB_H) / 2)

static uint16_t *framebuffer = NULL;
static bool frame_drawn = false;

// Render handoff: Core 0 sets this, Core 1 clears it after push.
static uint16_t * volatile render_buffer = NULL;
static TaskHandle_t renderTaskHandle = NULL;

// [PHASE 5] Callback from Walnut when MBC bank changes
void gb_rom_bank_changed_callback(struct gb_s* gb, uint16_t new_bank) {
  if (new_bank == hot_bank_num) return;
  if (new_bank < MAX_BANKS && bank_to_slot[new_bank] >= 0) return; 

  if (pf_state == PF_READY) {
    if (pf_request_bank == new_bank) return;
    pf_state = PF_FREE; // Discard stale completed prefetch
  }

  if (pf_state == PF_FREE) {
    int victim = 1;
    for (int i = 1; i < CACHE_SLOTS; i++) {
      if (i != hot_bank_slot && i != prev_bank_slot) { victim = i; break; }
    }
    // Safely invalidate victim metadata on Core 0
    int old = bank_cache[victim].bank_num;
    if (old >= 0 && old < MAX_BANKS) bank_to_slot[old] = -1;
    bank_cache[victim].bank_num = -1;
    
    pf_request_bank = new_bank;
    pf_active_slot = victim;
    if (renderTaskHandle) xTaskNotify(renderTaskHandle, 0x02, eSetBits);
  }
}

// Core 1 Service Task: Handles both TFT VSPI and Prefetch HSPI
void renderTask(void *pvParameters) {
  for (;;) {
    uint32_t flags = 0;
    xTaskNotifyWait(0x00, 0xFFFFFFFF, &flags, portMAX_DELAY);

    if ((flags & 0x01) && render_buffer) {
      uint32_t t0 = micros();
      tft.startWrite();
      tft.setAddrWindow(OFFSET_X, OFFSET_Y, GB_W, GB_H);
      tft.writePixels((uint16_t*)render_buffer, FB_PIXELS);
      tft.endWrite();
      prof.tft_time_acc += (micros() - t0);
      render_buffer = NULL;
    }

    if (flags & 0x02) {
      if (pf_state == PF_FREE && pf_request_bank >= 0 && pf_active_slot > 0) {
        pf_state = PF_LOADING;
        int req = pf_request_bank;
        int slot = pf_active_slot;
        
        uint32_t offset = (uint32_t)req * BANK_SIZE;
        size_t to_read = BANK_SIZE;
        if (offset + to_read > rom_total_size) {
          to_read = rom_total_size - offset;
        }
        
        uint32_t t0 = micros();
        rom_file.seek(offset);
        rom_file.read(bank_cache[slot].data, to_read);
        prof.sd_time_acc += (micros() - t0);
        
        if (pf_request_bank == req) {
          bank_cache[slot].bank_num = req;
          pf_state = PF_READY;
        } else {
          pf_state = PF_FREE; // Cancelled mid-flight
        }
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
  WiFi.mode(WIFI_OFF);
  btStop();

  // ── Allocate big chunks first ──────────────────────────────
  gb = (struct gb_s*)malloc(sizeof(struct gb_s));
  if (!gb) { Serial.println("OOM: gb_s"); while (1); }
  memset(gb, 0, sizeof(struct gb_s));
  Serial.printf("gb_s: %u bytes\n", sizeof(struct gb_s));

  framebuffer = (uint16_t*)malloc(FB_BYTES);
  if (!framebuffer) { Serial.println("OOM: FB"); while (1); }
  memset(framebuffer, 0, FB_BYTES);

  cache_base = (uint8_t*)malloc(CACHE_SLOTS * BANK_SIZE);
  if (!cache_base) { Serial.println("OOM: Cache"); while (1); }
  memset(bank_to_slot, -1, sizeof(bank_to_slot));
  for (int i = 0; i < CACHE_SLOTS; i++) {
    bank_cache[i].data = cache_base + (i * BANK_SIZE);
    bank_cache[i].bank_num = -1;
  }
  hot_bank_slot = 1;
  prev_bank_slot = 2;

  Serial.printf("Free: %u  Largest: %u\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  // ── Hardware init ──────────────────────────────────────────
  pinMode(BTN_UP, INPUT_PULLUP);    pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);     pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_SELECT, INPUT_PULLUP);

  // Initialize VSPI (TFT) at 40 MHz
  tft.begin(40000000);
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  // Initialize HSPI (SD Card) — SEPARATE physical bus!
  hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  debugPrint("Init SD (HSPI)...");
  // [PHASE 2] SD Optimization: Increased from 4MHz to 8MHz
  #define SD_SPI_SPEED 8000000 
  if (!SD.begin(SD_CS, hspi, SD_SPI_SPEED)) {
    debugPrint("SD Failed!");
    while (1);
  }
  Serial.println("SD on HSPI: OK");

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

  gb_init(gb, &gb_rom_read, &gb_rom_read_16bit, &gb_rom_read_32bit,
          &gb_cart_ram_read, &gb_cart_ram_write, &gb_error, &priv);
  gb->gb_rom_bank_changed = &gb_rom_bank_changed_callback;

  cart_ram_size = gb_get_save_size(gb);
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

  gb_init_lcd(gb, &lcd_draw_line);
  gb->direct.interlace = 0;
  cache_load_bank(0, 0);

  // Launch Core 1 render task — owns VSPI exclusively
  xTaskCreatePinnedToCore(renderTask, "Render", 3072, NULL, 1,
                          &renderTaskHandle, 1);

  tft.fillScreen(ILI9341_BLACK);
  Serial.printf("Game start. Free: %u  Largest: %u\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  reset_prof();
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP — Core 0
// ═══════════════════════════════════════════════════════════════
// Core 0: emulation + SD reads (HSPI)
// Core 1: TFT push (VSPI)
// These run on PHYSICALLY SEPARATE SPI buses. A cache miss on
// Core 0 NEVER stalls the TFT, and the TFT NEVER stalls SD.
void loop() {
  uint32_t loop_start = micros();
  frame_count++;

  // Wait for Core 1 to finish previous TFT push before
  // writing to the framebuffer. If emulation (~15ms) is
  // slower than TFT push (~9ms), this wait is ZERO.
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
    xTaskNotify(renderTaskHandle, 0x01, eSetBits);
  }

  // ── Auto-save (Core 0 only, via HSPI) ─────────────────────
  if (frame_count % SAVE_INTERVAL_FRAMES == 0 && cart_ram_dirty) {
    save_cart_ram();
  }

  prof.loop_time_acc += (micros() - loop_start);
  prof.frames++;
  
  uint32_t fh = ESP.getFreeHeap();
  if (fh < prof.min_free_heap) prof.min_free_heap = fh;

  if (prof.frames >= 300) {
    char buf[256];
    snprintf(buf, sizeof(buf),
      "\n=== PROFILING (300 frames) ===\n"
      "Emulation: %u ms/f\n"
      "PPU (draw): %u ms/f\n"
      "Core0 Wait: %u ms/f\n"
      "TFT Push: %u ms/f\n"
      "SD Misses: %u\n"
      "SD Time: %u ms/miss\n"
      "PF Hits: %u\n"
      "PF Aborts: %u\n"
      "Save Time: %u ms\n"
      "Loop Total: %u ms/f (%.1f FPS)\n"
      "Min Free: %u\n"
      "Max Block: %u\n"
      "Core1 Stack: %u\n"
      "==============================",
      prof.emu_time_acc / 300000,
      prof.ppu_time_acc / 300000,
      prof.core0_wait_acc / 300000,
      prof.tft_time_acc / 300000,
      prof.cache_misses,
      prof.cache_misses ? (prof.sd_time_acc / 1000) / prof.cache_misses : 0,
      prof.prefetch_hits,
      prof.prefetch_aborts,
      (prof.save_time_acc / 1000),
      prof.loop_time_acc / 300000,
      300000000.0f / prof.loop_time_acc,
      prof.min_free_heap,
      ESP.getMaxAllocHeap(),
      uxTaskGetStackHighWaterMark(renderTaskHandle)
    );
    Serial.println(buf);
    reset_prof();
  }
}

