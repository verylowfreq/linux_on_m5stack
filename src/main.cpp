#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include <M5Stack.h>
#include <esp_task_wdt.h>
#include <driver/adc.h>

#include "debug.h"
#include "config.h"

#include "sdfile.h"

int sd_chipselect = TFCARD_CS_PIN;

sdfilecontext_t sdfilectx;

M5Display& Lcd = M5.Lcd;

bool spibus_is_ready(void) {
#ifdef USE_SDFILE_SDSPI
  return true;
#endif
#ifdef USE_SDFILE_SDFAT
  return true;
#endif
}

class Stopwatch_Seconds {
public:
  unsigned long last_millis;
  unsigned long fraction_millis;
  uint32_t seconds;

  void start(void) {
    this->seconds = 0;
    this->last_millis = millis();
  }

  void update(void) {
    unsigned long elapsed = millis() - this->last_millis;
    if (elapsed > 1000) {
      elapsed += this->fraction_millis;
      uint32_t secs = elapsed / 1000;
      uint32_t frac = elapsed - secs * 1000;
      this->seconds += secs;
      this->fraction_millis = frac;
      this->last_millis = millis();
    }
  }

  uint32_t get_seconds(void) {
    return this->seconds;
  }
};


class KeyboardFace {
public:
  const uint8_t i2c_addr = 0x08;
  const uint8_t interrupt_pin = 5;

  bool available;

  KeyboardFace(void) {
    this->available = false;
  }

  bool init(void) {
    return true;
  }

  bool get_input(char* dst) {
    if (!this->available) {
      return false;
    }

    bool recved = false;
    Wire.requestFrom(this->i2c_addr, (uint8_t)1);
    while (Wire.available()) {
      int ch = Wire.read();
      if (ch >= 0) {

        if (ch == 0x0a) {
          // NOTE: Enterキーが 0x0d, 0x0aを送るファームウェアの版が過去にあったようなので、念のため0x0Aは無視する
          break;

        } else {
          *dst = ch;
          recved = true;
          break;
        }
      } else {
        break;
      }
    }
    this->available = false;
    return recved;
  }

  void set_interupt(bool val) {
    this->available = val;
  }
};

KeyboardFace kbdface;

void on_kbdface_interupt(void) {
  kbdface.set_interupt(true);
}


#include "ramdevice.h"
#include "ramcache.h"
#include "ramcache_impl.h"

const uint32_t RAMSIZE_DEFINE_IN_DTB = 64 * 1024 * 1024;
// NOTE: Some additional space is needed.
#define RAM_ADDITIONAL_SPACE 128
uint32_t ram_amt = 16 * 1024 * 1024 - RAM_ADDITIONAL_SPACE;

RAMDevice __ram(ram_amt + RAM_ADDITIONAL_SPACE, "/ram.img");

CachedRAM ram(__ram);


#define MINIRV32_CUSTOM_MEMORY_BUS

void MINIRV32_MEMBUSWRITE_ASSERT(bool success, uint32_t arg1, uint32_t arg2 = 0xffffffff) {
  if (!success) {
    PANIC("RAM write access FAILED !! arg1 = 0x%08x, arg2 = 0x%08x", arg1, arg2);
  }
}
void MINIRV32_MEMBUSREAD_ASSERT(bool success, uint32_t addr) {
  if (!success) {
    PANIC("RAM read access FAILED !! addr = 0x%08x", addr);
  }
}


void MINIRV32_STORE4(uint32_t offset, uint32_t val) {
  MINIRV32_MEMBUSWRITE_ASSERT(ram.write_u32(offset, val), offset, val);
}
void MINIRV32_STORE2(uint32_t offset, uint16_t val) {
  MINIRV32_MEMBUSWRITE_ASSERT(ram.write_u16(offset, val), offset, val);
}
void MINIRV32_STORE1(uint32_t offset, uint8_t val) {
  MINIRV32_MEMBUSWRITE_ASSERT(ram.write_u8(offset, val), offset, val);
}

uint32_t MINIRV32_LOAD4(uint32_t offset) {
  uint32_t data;
  MINIRV32_MEMBUSREAD_ASSERT(ram.read_u32(offset, &data), offset);
  return data;
}
uint16_t MINIRV32_LOAD2(uint32_t offset) {
  uint16_t data;
  MINIRV32_MEMBUSREAD_ASSERT(ram.read_u16(offset, &data), offset);
  return data;
}
uint8_t MINIRV32_LOAD1(uint32_t offset) {
  uint8_t data;
  MINIRV32_MEMBUSREAD_ASSERT(ram.read_u8(offset, &data), offset);
  return data;
}


uint32_t MINIRV32_LOAD4_PROGRAM(uint32_t offset) {
  uint32_t data;
  MINIRV32_MEMBUSREAD_ASSERT(ram.read_u32(offset, &data), offset);
  return data;
}

int fail_on_all_faults = 0;
bool do_sleep = false;
bool fixed_update = true;
int time_divisor = 1;
bool single_step = false;
int64_t instct = -1;


static int64_t SimpleReadNumberInt( const char * number, int64_t defaultNumber );
static uint64_t GetTimeMicroseconds();
static void ResetKeyboardInput();
static void CaptureKeyboardInput();
static uint32_t HandleException( uint32_t ir, uint32_t retval );
static uint32_t HandleControlStore( uint32_t addy, uint32_t val );
static uint32_t HandleControlLoad( uint32_t addy );
static void HandleOtherCSRWrite( /*uint8_t * */ CachedRAM* image, uint16_t csrno, uint32_t value );
static void MiniSleep();
static int IsKBHit();
static int ReadKBByte();

// This is the functionality we want to override in the emulator.
//  think of this as the way the emulator's processor is connected to the outside world.
#define MINIRV32WARN( x... ) Serial.printf( x );
#define MINIRV32_DECORATE  static
#define MINI_RV32_RAM_SIZE ram_amt
#define MINIRV32_IMPLEMENTATION
#define MINIRV32_POSTEXEC( pc, ir, retval ) { if( retval > 0 ) { if( fail_on_all_faults ) { Serial.printf( "FAULT\n" ); return 3; } else retval = HandleException( ir, retval ); } }
#define MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, val ) if( HandleControlStore( addy, val ) ) return val;
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL( addy, rval ) rval = HandleControlLoad( addy );
#define MINIRV32_OTHERCSR_WRITE( csrno, value ) HandleOtherCSRWrite( &image, csrno, value );


#include "mini-rv32ima.h"

#include "default64mbdtc.h"

struct MiniRV32IMAState __core;
struct MiniRV32IMAState* core = &__core;

// #define UPDATE_DUMPSTATE_TICKS 4096
#define UPDATE_DUMPSTATE_TICKS 16384
bool enable_dumpstate_over_serial = false;

static void DumpState( struct MiniRV32IMAState * core, /*uint8_t * */ CachedRAM* ram_image );


static bool copy_file_to_ram(const char* srcfilepath, uint32_t dstaddr) {
  DEBUG("Copy %s to RAM 0x%08x", srcfilepath, dstaddr);
  sdfile_t file;
  if (!sdfile_open_read(&sdfilectx, &file, srcfilepath)) {
    ERROR("Cannot open %s", srcfilepath);
    return false;
  }
  

  uint32_t chunksize = 8 * 1024;
  uint8_t* buf = (uint8_t*)malloc(chunksize);
  if (!buf) {
    PANIC("Cannot allocate memory for copying file %d, %d bytes.", chunksize);
  }
  size_t remains = 0;
  sdfile_get_filesize(&file, &remains);
  uint32_t totalrdlen = 0;
  DEBUG("Start copying total %d bytes in %d units of %d bytes chunk", remains, (remains / chunksize), chunksize);
  while (remains > 0) {
    Serial.print(".");
    Lcd.print(".");
    if (remains < chunksize) {
      chunksize = remains;
    }
    
    size_t rdlen = sdfile_read(&file, chunksize, buf);
    if (rdlen == 0) {
      // No remains.
      break;
    } else if (rdlen < 0) {
      Serial.println();
      PANIC("Failed to read file (%d)", rdlen);
    }
    totalrdlen += rdlen;
    delay(0);

    if (!ram.directwrite(dstaddr, buf, chunksize, false)) {
      PANIC("Failed to write data %d bytes", chunksize);
    }
    dstaddr += rdlen;
    delay(0);

    ram.flush();
    delay(0);
  }
  assert(ram.purge_and_reload());
  free(buf);
  // srcfile.close();
  sdfile_close(&file);
  Serial.println();

  return true;
}

static
bool update_ramsize_in_dtb(uint32_t dtb_addr, uint32_t newramsize) {
  DEBUG("Update RAM size in DTB");
  
  // Update system ram size in DTB (but if and only if we're using the default DTB)
  // Warning - this will need to be updated if the skeleton DTB is ever modified.

  // Replace [ 0x03, 0xff, 0xc0, 0x00 ] as uint32_t BIG ENDIANNESS

  uint32_t offset = 0x13c;
  uint32_t targetaddr = dtb_addr + offset;

  uint8_t orig_data[4] = { 0, 0, 0, 0 };
  assert(ram.read_block(targetaddr, orig_data, 4));
  uint32_t orig_value = orig_data[0] << 24 | orig_data[1] << 16 | orig_data[2] << 8 | orig_data[3];
  DEBUG("RAM size in DTB is 0x%08x(%d) bytes", orig_value, orig_value);

  uint8_t data[4] = {
    (uint8_t)(ram_amt >> 24),
    (uint8_t)((ram_amt >> 16) & 0xff),
    (uint8_t)((ram_amt >> 8) & 0xff),
    (uint8_t)(ram_amt & 0xff)
  };
  assert(ram.write_block(targetaddr, data, 4));
  DEBUG("  Replaced with 0x%08x(%d)", ram_amt, ram_amt);

  return true;
}

void draw_banner(void) {
  spibus_is_ready(); 
  Lcd.clear();
  Lcd.setTextSize(3);
  Lcd.fillRect(0, 0, 320, 8 * 4, TFT_BLUE);
  Lcd.textbgcolor = TFT_BLUE;
  Lcd.textcolor = TFT_WHITE;
  Lcd.setCursor(8 * 2, 4);
  Lcd.println("Linux on M5Stack");
  Lcd.textbgcolor = TFT_BLACK;
  Lcd.setTextSize(2);
  Lcd.println();
  Lcd.println("BUILD " __TIME__ " " __DATE__);
  Lcd.println();
  Lcd.println("RISC-V RV32IMA Linux runs "
                 "on ESP32 without PSRAM.   ");
  Lcd.println();
  Lcd.println("REF: cnlohr/mini-rv32ima  "
                 "                 on GitHub");
  Lcd.println();
  Lcd.printf("Battery: %d%%", M5.Power.getBatteryLevel());
  Lcd.println();
  Lcd.setTextSize(1);
  Lcd.println();
  spibus_is_ready(); 
}

Stopwatch_Seconds stopwatch;

void draw_header(void) {
  uint32_t elapsed_sec = stopwatch.get_seconds();
  int curx = Lcd.getCursorX();
  int cury = Lcd.getCursorY();
  Lcd.fillRect(0, 0, 320, 16, TFT_BLUE);
  Lcd.setCursor(0, 0);
  Lcd.setTextColor(TFT_WHITE);
  Lcd.printf("[%8d] PC:%08x, Cycle:%10u%10u", elapsed_sec, core->pc, core->cycleh, core->cyclel);
  Lcd.setCursor(0, 8);
  float hitrate1 = (float)ram.hitcount / (float)ram.counter * 100.0f;
  float missrate = (float)ram.reassigncount / (float)ram.counter * 100.0f;
  Lcd.printf("RAM$:hit=%7.4f%%,miss=%7.4f%%", hitrate1, missrate);
  Lcd.setCursor(curx, cury);
  Lcd.setTextColor(TFT_WHITE);
}

void config_over_serial(void) {
  while (true) {
    M5.update();
    if (M5.BtnB.wasPressed()) {
      return;
    }
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      Serial.println(input);

      if (input.equals("help") || input.length() == 0) {
        Serial.println("Available commands: dumpstate, nodumpstate");

      } else if (input.equals("nodumpstate")) {
        enable_dumpstate_over_serial = false;
        Serial.println("OK.");

      } else if (input.equals("dumpstate")) {
        enable_dumpstate_over_serial = true;
        Serial.println("OK.");
      
      } else if (input.equals("run")) {
        Serial.println("OK.");
        return;
      
      } else {
        Serial.println("Unknown command. 'help' to show help");
      }
    }
    delay(10);
  }
}


void test_keyboardface(void) {
  // Keyboard Face test
  if (true) {
    attachInterrupt(kbdface.interrupt_pin, on_kbdface_interupt, FALLING);
    while (true) {
      if (IsKBHit()) {
        int ch = ReadKBByte();
        if (ch >= 0) {
          Serial.print((char)ch);
          Lcd.print((char)ch);
        } else {
          Serial.print('!');
          Lcd.print('!');
        }
      }
      delay(10);
    }
  }
}

void test_sdcard(void) {

  DEBUG("Init SD");

  sdfilecontext_t sdfilectx;
  if (!sdfile_init(&sdfilectx)) {
    PANIC("Failed to sdfile_init()");
  }

  DEBUG("Open file");
  sdfile_t file;
  if (!sdfile_open_read(&sdfilectx, &file, "/boot.bin")) {
    PANIC("Failed to sdfile_open_read()");
  }

  sdfile_set_bufsize(&file, 16384);

  size_t buflen = 8192 * 2;
  uint8_t* buf = (uint8_t*)malloc(buflen);
  assert(buf);
  uint32_t totalrdlen = 0;
  unsigned long start = millis();
  while (true) {
    size_t rdlen = sdfile_read(&file, buflen, buf);
    if (rdlen <= 0) {
      break;
    }
    totalrdlen += rdlen;
  }
  unsigned long elapsed = millis() - start;
  DEBUG("Read %d bytes in %lu msec = %f B/s", totalrdlen, elapsed, (float)totalrdlen / elapsed * 1000.0f);

  PANIC("Finished.");
}


bool save_vmstate(const char* filepath, struct MiniRV32IMAState* src) {
  sdfile_t file;
  if (!sdfile_open_write(&sdfilectx, &file, filepath)) {
    ERROR("Failed to open %s", filepath);
    return false;
  }
  // sdfile_seek(&file, 0);
  size_t len = sizeof(struct MiniRV32IMAState);
  size_t wrlen = sdfile_write(&file, len, src);
  if (wrlen != len) {
    ERROR("Failed to sdfile_write(). expected %u but actual %u", len, wrlen);
    sdfile_close(&file);
    return false;
  }
  DEBUG("OK.");
  sdfile_close(&file);
  return true;
}

bool restore_vmstate(const char* filepath, struct MiniRV32IMAState* dst) {
  sdfile_t file;
  if (!sdfile_open_read(&sdfilectx, &file, filepath)) {
    ERROR("Failed to open %s", filepath);
    return false;
  }
  sdfile_seek(&file, 0);
  size_t len = sizeof(struct MiniRV32IMAState);
  size_t rdlen = sdfile_read(&file, len, dst);
  if (rdlen != len) {
    ERROR("Failed to sdfile_read(). expected %u but actual %u", len, rdlen);
    return false;
  }
  DEBUG("OK.");
  sdfile_close(&file);
  return true;
}

const char* vmstatefilepath = "/cpustate.bin";

bool hibernate(void) {

  ram.flush();

  if (!save_vmstate(vmstatefilepath, core)) {
    ERROR("Failed to save_vmstate()");
    return false;
  }

  DEBUG("Hibernate OK.");
  DEBUG("Ready to shutdown.");;

  // On success, no return
  while (true) {
    delay(1000);
  }
}

bool resume(void) {
  DEBUG("Init SD");
  if (!sdfile_init(&sdfilectx)) {
    PANIC("Failed to init SDCard");
  }
  sdfile_set_spibus_ready(&sdfilectx, spibus_is_ready);

  DEBUG("Init RAM...");
  uint32_t ramsize = __ram.init(false, RAMDEVICE_SDFILE_BUFLEN);
  if (ramsize == 0) {
    PANIC("Failed to init RAM.");
  }
  
  DEBUG("Init RAM cache.");
  if (!ram.init()) {
    PANIC("Failed to init RAM cache.");
  }
  DEBUG("  RAM %d bytes ready.", ramsize);

  if (!restore_vmstate(vmstatefilepath, core)) {
    PANIC("Failed to restore vm state.");
  }
  
	CaptureKeyboardInput();

  return true;
}

bool startnew(void) {
  
  DEBUG("Init SD");
  if (!sdfile_init(&sdfilectx)) {
    PANIC("Failed to init SDCard");
  }
  sdfile_set_spibus_ready(&sdfilectx, spibus_is_ready);

  DEBUG("Init RAM...");
  uint32_t ramsize = __ram.init(false, RAMDEVICE_SDFILE_BUFLEN);
  if (ramsize == 0) {
    PANIC("Failed to init RAM.");
  }
  
  DEBUG("Init RAM cache.");
  if (!ram.init()) {
    PANIC("Failed to init RAM cache.");
  }
  DEBUG("  RAM %d bytes ready.", ramsize);
  
  DEBUG("Copy image...");
  {
    unsigned long start_millis = millis();
    if (!copy_file_to_ram("/boot.bin", 0)) {
      PANIC("Failed to copy_file_to_ram()");
    }
    DEBUG("  copied in %ld [msec].", millis() - start_millis);
  }

  DEBUG("Copy DTB...");
  uint32_t dtb_addr;
  {
    uint32_t dtbsize = sizeof(default64mbdtb);
    uint32_t dtb_startaddr = ram_amt - dtbsize - sizeof(struct MiniRV32IMAState);
    if (!ram.write_block(dtb_startaddr, (uint8_t*)default64mbdtb, dtbsize)) {
      PANIC("Failed to copy DTB.");
    }
    dtb_addr = dtb_startaddr;
    ram.flush();
  }
  

	CaptureKeyboardInput();

  DEBUG("Init core...");
	// The core lives at the end of RAM.
	core->pc = MINIRV32_RAM_IMAGE_OFFSET;
	core->regs[10] = 0x00; //hart ID
	core->regs[11] = dtb_addr?(dtb_addr+MINIRV32_RAM_IMAGE_OFFSET):0; //dtb_pa (Must be valid pointer) (Should be pointer to dtb)
	core->extraflags |= 3; // Machine-mode.

  if (ram_amt != RAMSIZE_DEFINE_IN_DTB) {
    if (!update_ramsize_in_dtb(dtb_addr, ram_amt)) {
      PANIC("Failed to update_ramsize_in_dtb() dtb=addr=%08x, ram_amt=%u", dtb_addr, ram_amt);
    }
  }

  return true;
}

// ---- Defined in M5Stack library's Power.cpp

#define CURRENT_100MA  (0x01 << 0)
#define CURRENT_200MA  (0x01 << 1)
#define CURRENT_400MA  (0x01 << 2)
#define CURRENT_800MA  (0x01 << 3)
#define CURRENT_1600MA  (0x01 << 4)

void setup() {
  esp_log_level_set("*", ESP_LOG_INFO);
  adc_power_acquire();

  M5.begin(true, false, true, true);

  M5.Power.setVinMaxCurrent(CURRENT_100MA);

  // if (!psramInit()) {
  //   Lcd.print("ERROR: Cannot initialize PSRAM.");
  //   PANIC("Cannot initialize PSRAM.");
  // }

  // if (!psramFound()) {
  //   Lcd.print("ERROR: PSRAM not found.");
  //   PANIC("PSRAM not found.");
  // }

  if (!kbdface.init()) {
    DEBUG("Failed to init() Keyboard Face unit.");
  }  
  attachInterrupt(kbdface.interrupt_pin, on_kbdface_interupt, FALLING);
  // DEBUG: Keyboard FACE test
  // test_keyboardface();

  Serial.println("Press Enter to start...");
  draw_banner();
  Lcd.println("Press left button to resume...");
  Lcd.println("Press center button to start fresh...");

  // config_over_serial();

  bool mode_resume = false;
  bool mode_startnew = false;

  while (true) {
    M5.update();
    if (M5.BtnA.wasPressed()) {
      DEBUG("Resume mode");
      mode_resume = true;
      break;
    } else if (M5.BtnB.wasPressed()) {
      DEBUG("Start new");
      mode_startnew = true;
      break;
    }
    delay(10);
  }

  Lcd.clear();
  Lcd.setCursor(0, 0);
  Lcd.println("Start");

  DEBUG("Start...");

  // DEBUG:
  // test_sdcard();

  DEBUG("Disable WDT");
  disableCore0WDT();
  disableCore1WDT();

  if (mode_startnew) {
    // Start fresh VM
    startnew();
  } else if (mode_resume) {
    // Resume from previous state file on microSD.
    resume();
  } else {
    PANIC("Invalid state. neither mode_resume and mode_startnew was selected.");
  }

  DEBUG("Initialized.");

  Lcd.clear();
  Lcd.setCursor(0, 16);

}


void loop() {
  {
    stopwatch.start();

    unsigned long prev_update_header = 0;
    uint64_t rt;
    uint64_t lastTime = (fixed_update)?0:(GetTimeMicroseconds()/time_divisor);
    int instrs_per_flip = single_step?1:1024;
    for( rt = 0; rt < instct+1 || instct < 0; rt += instrs_per_flip )
    {
      esp_task_wdt_reset();

      stopwatch.update();

      if (millis() - prev_update_header >= 300) {
        if (enable_dumpstate_over_serial) {
          DEBUG("time=%d[sec], cycle h=%d, l=%d", stopwatch.get_seconds(), core->cycleh, core->cyclel);
          DumpState(core, &ram);
        }
        draw_header();
        prev_update_header = millis();
      }

      M5.update();
      if (M5.BtnA.pressedFor(500)) {
        DEBUG("Hibernating...");
        if (!hibernate()) {
          ERROR("Failed to hibernate. Continue to execution.");
          while (true) {
            M5.update();
            if (M5.BtnA.isReleased()) {
              break;
            }
            delay(10);
          }
        }
      }

      uint64_t * this_ccount = ((uint64_t*)&core->cyclel);
      uint32_t elapsedUs = 0;
      if( fixed_update )
        elapsedUs = *this_ccount / time_divisor - lastTime;
      else
        elapsedUs = GetTimeMicroseconds()/time_divisor - lastTime;
      lastTime += elapsedUs;

      if( single_step )
        DumpState( core, &ram);

      int ret = MiniRV32IMAStep( core, ram, 0, elapsedUs, instrs_per_flip ); // Execute upto 1024 cycles before breaking out.
      switch( ret )
      {
        // 実行を停止する
        case 0:
          break;
        // 正常に実行を完了
        case 1:
          if( do_sleep ) {
            MiniSleep();
          }
          // 実行済み命令数を更新する
          *this_ccount += instrs_per_flip;
          break;
        // 実行を停止する
        case 3:
          instct = 0;
          break;
        // リセットが要求された
        case 0x7777:
          DEBUG("Reboot by syscon 0x7777");
          ESP.restart();	//syscon code for restart
          break;
        // シャットダウンが要求された
        case 0x5555:
          DEBUG( "POWEROFF@0x%08x%08x\n", core->cycleh, core->cyclel );
          PANIC("HALT."); //syscon code for power-off
        // 未定義の実行失敗
        default:
          DEBUG( "Unknown failure\n" );
          break;
      }
    }

    DEBUG("Mainloop stopped.");
    DumpState(core, &ram);
    PANIC("Unexpected halt.");
  }

}


static void DumpState( struct MiniRV32IMAState * core, CachedRAM* ram )
{
  if (!enable_dumpstate_over_serial) {
    return;
  }

	uint32_t pc = core->pc;
	uint32_t pc_offset = pc - MINIRV32_RAM_IMAGE_OFFSET;
	uint32_t ir = 0;

	printf( "PC: %08x ", pc );
	if( pc_offset >= 0 && pc_offset < ram_amt - 3 )
	{
    ram->read_u32(pc_offset, &ir);
		printf( "[0x%08x] ", ir ); 
	}
	else
		printf( "[xxxxxxxxxx] " ); 
	uint32_t * regs = core->regs;
	printf( "Z:%08x ra:%08x sp:%08x gp:%08x tp:%08x t0:%08x t1:%08x t2:%08x s0:%08x s1:%08x a0:%08x a1:%08x a2:%08x a3:%08x a4:%08x a5:%08x ",
		regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7],
		regs[8], regs[9], regs[10], regs[11], regs[12], regs[13], regs[14], regs[15] );
	printf( "a6:%08x a7:%08x s2:%08x s3:%08x s4:%08x s5:%08x s6:%08x s7:%08x s8:%08x s9:%08x s10:%08x s11:%08x t3:%08x t4:%08x t5:%08x t6:%08x\n",
		regs[16], regs[17], regs[18], regs[19], regs[20], regs[21], regs[22], regs[23],
		regs[24], regs[25], regs[26], regs[27], regs[28], regs[29], regs[30], regs[31] );
}


static void CaptureKeyboardInput()
{
  
}

static void ResetKeyboardInput()
{
}

static void MiniSleep()
{
	// Sleep(1);
  delay(1);
}

static uint64_t GetTimeMicroseconds()
{
  static uint32_t prev = 0;
  static uint32_t upper = 0;
  
  uint32_t now = micros();
  if (now < prev) {
    upper += 1;
  }
  uint64_t ret = ((uint64_t)upper << 32) | (uint64_t)now;
  prev = now;
  return now;
}

/* return 1 if data is ready or 0 to not received. */
static int IsKBHit()
{
  bool kbhit_uart = Serial.available() > 0;
  bool kbhit_kbd = kbdface.available;
	return kbhit_uart || kbhit_kbd ? 1 : 0;
}

static int ReadKBByte()
{
  if (Serial.available()) {
    return Serial.read();
  }

  if (kbdface.available) {
    char ch;
    if (kbdface.get_input(&ch)) {
      return ch;
    } else {
      return -1;
    }
  } else {
    return -1;
  }
}


//////////////////////////////////////////////////////////////////////////
// Rest of functions functionality
//////////////////////////////////////////////////////////////////////////

static uint32_t HandleException( uint32_t ir, uint32_t code )
{
  if (code == 0x80000007) {
    // Timer interupt. It's okay.
    return code;
  }
  if (code == 0x00000009) {
    // ECALL. It's okay.
    return code;
  }

  DEBUG("Exception occured: ir=0x%08x, code=0x%08x(%d)", ir, code ,code);
  DumpState(core, &ram);

	// Weird opcode emitted by duktape on exit.
	if( code == 3 )
	{
		// Could handle other opcodes here.
	}
	return code;
}

void PrintLCD(char ch) {
  const uint8_t SPACE_FOR_BOTTOM = 8 * 2;
  const uint8_t TOP_OF_CONSOLETEXT = 16;
  const uint8_t HEIGHT_OF_CLEAR_AHEAD = 8 * 6;
  // 画面の下端に到達していたら上へ戻す
  if (Lcd.getCursorY() + SPACE_FOR_BOTTOM > Lcd.height()) {
    Lcd.setCursor(0, TOP_OF_CONSOLETEXT);
  }
  // 行送りをする場合は先の行を少し消す
  if (Lcd.getCursorX() == 0) {
    int pos_x = Lcd.getCursorX();
    int pos_y = Lcd.getCursorY();
    Lcd.fillRect(0, pos_y, 320, HEIGHT_OF_CLEAR_AHEAD, TFT_BLACK);
    Lcd.setCursor(pos_x, pos_y);
  }
  Lcd.print(ch);
}


static uint32_t HandleControlStore( uint32_t addy, uint32_t val )
{
	if( addy == 0x10000000 ) //UART 8250 / 16550 Data Buffer
	{
		Serial.printf( "%c", val );
		// fflush( stdout );
    PrintLCD((char)val);
	}
	return 0;
}


static uint32_t HandleControlLoad( uint32_t addy )
{
	// Emulating a 8250 / 16550 UART
	if( addy == 0x10000005 )
		return 0x60 | IsKBHit();
	else if( addy == 0x10000000 && IsKBHit() )
		return ReadKBByte();
	return 0;
}

static void HandleOtherCSRWrite( /*uint8_t * */ CachedRAM* image, uint16_t csrno, uint32_t value )
{
	if( csrno == 0x136 )
	{
		Serial.printf( "%d", value ); //fflush( stdout );
    Lcd.printf( "%d", value );
    PrintLCD(' ');
	}
	if( csrno == 0x137 )
	{
		Serial.printf( "%08x", value ); //fflush( stdout );
    Lcd.printf( "%d", value );
    PrintLCD(' ');
	}
	else if( csrno == 0x138 )
	{
		//Print "string"
    uint32_t addr = value - MINIRV32_RAM_IMAGE_OFFSET;
    uint32_t tail = ram_amt;
    while (addr < tail) {
      char ch = 0;
      assert(image->read_u8(addr, (uint8_t*)&ch));
      Serial.print(ch);
      PrintLCD(ch);
      addr++;
    }
	}
}

static int64_t SimpleReadNumberInt( const char * number, int64_t defaultNumber )
{
	if( !number || !number[0] ) return defaultNumber;
	int radix = 10;
	if( number[0] == '0' )
	{
		char nc = number[1];
		number+=2;
		if( nc == 0 ) return 0;
		else if( nc == 'x' ) radix = 16;
		else if( nc == 'b' ) radix = 2;
		else { number--; radix = 8; }
	}
	char * endptr;
	uint64_t ret = strtoll( number, &endptr, radix );
	if( endptr == number )
	{
		return defaultNumber;
	}
	else
	{
		return ret;
	}
}


void memdump(RAMDevice& ram, uint32_t startaddr, uint32_t size) {
  Serial.println("----Memory dump----");
  for (uint32_t addr = startaddr; addr < startaddr + size; addr += 16) {
    Serial.printf("%08X: ", addr);
    for (int i = 0; i < 16; i++) {
      uint8_t val = 0xff;
      assert(ram.read_u8(addr, &val));
      Serial.printf(" %02x", val);
    }
    Serial.printf("    ");
    for (int i = 0; i < 16; i++) {
      uint8_t val = 0xff;
      assert(ram.read_u8(addr, &val));
      if (val < ' ') {
        val = '.';
      } else if (val >= 0x7f) {
        val = '+';
      }
      Serial.printf("%c", val);
    }
    Serial.println();
  }
  Serial.println("----");
}
