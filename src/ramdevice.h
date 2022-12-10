#pragma once

#include <stdint.h>
#include <SdFat.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "debug.h"
#include "config.h"

#include "sdfile.h"

class RAMDevice {
public:
  const char* filepath;
  uint32_t size;
  sdfile_t file;
  bool is_initialized;

  RAMDevice(uint32_t size, const char* filepath) {
    this->is_initialized = false;
    this->size = size;
    this->filepath = (const char*)calloc(strlen(filepath) + 1, 1);
    assert(this->filepath);
    memcpy((void*)this->filepath, filepath, strlen(filepath));
  }

  uint32_t init(bool zerofill, size_t sdfile_buffersize) {
    if (!sdfile_open_readwrite(&sdfilectx, &this->file, this->filepath)) {
      ERROR("Error. Cannot open %s for RAM image.\n", this->filepath);
      this->size = 0;
      return 0;
    }

#ifdef USE_SDFILE_SDSPI
    if (sdfile_buffersize > 0) {
      DEBUG("RAMDevice set buffer %d bytes", sdfile_buffersize);
      sdfile_set_bufsize(&this->file, sdfile_buffersize);
    }
#endif

    DEBUG("Requested RAM size is %d bytes", this->size);
    size_t filesize = 0;
    sdfile_get_filesize(&this->file, &filesize);
    DEBUG("Current filesize is %d bytes", filesize);
    sdfile_seek(&this->file, 0);
    if (zerofill || (filesize < this->size)) {
      uint32_t remains = this->size;
      uint32_t chunksize = 8 * 1024;
      uint8_t* chunk = (uint8_t*)calloc(chunksize, 1);
      assert(chunk);
      while (remains > 0) {
        if (remains < chunksize) {
          chunksize = remains;
        }
        size_t wrlen = sdfile_write(&this->file, chunksize, chunk);
        delay(0);
        Serial.print(".");
        M5.Lcd.print(".");
        if (wrlen != chunksize) {
          ERROR("Error. SD write failed at %d\n while RAM initialize.\n", this->size - remains);
          free(chunk);
          return 0;
        }
        remains -= chunksize;
      }
      Serial.println();
      free(chunk);
      sdfile_flush(&this->file);
      sdfile_seek(&this->file, 0);
    }
    this->is_initialized = true;
    return this->size;
  }

  bool read_block(uint32_t addr, uint8_t* dst, uint32_t len) {
    sdfile_seek(&this->file, addr);
    return sdfile_read(&this->file, len, dst) == len;
  }

  bool write_block(uint32_t addr, uint8_t* src, uint32_t len) {
    sdfile_seek(&this->file, addr);
    return sdfile_write(&this->file, len, src) == len;
  }

  bool read_u8(uint32_t addr, uint8_t* dst) {
    return this->read_block(addr, dst, 1);
  }

  bool write_u8(uint32_t addr, uint8_t data) {
    return this->write_block(addr, &data, 1);
  }

  bool read_u16(uint32_t addr, uint16_t* dst) {
    return this->read_block(addr, (uint8_t*)dst, 2);
  }

  bool write_u16(uint32_t addr, uint16_t data) {
    return this->write_block(addr, (uint8_t*)&data, 2);
  }

  bool read_u32(uint32_t addr, uint32_t* dst) {
    return this->read_block(addr, (uint8_t*)dst, 4);
  }

  bool write_u32(uint32_t addr, uint32_t data) {
    return this->write_block(addr, (uint8_t*)&data, 4);
  }
};
