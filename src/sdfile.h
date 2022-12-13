#pragma once

#include <stdbool.h>
#include <stdint.h>


#include "config.h"
#include "debug.h"


#ifdef USE_SDFILE_SDFAT

#include <SdFat.h>

#endif
#ifdef USE_SDFILE_SDSPI

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#endif

#ifdef USE_SDFILE_SDFAT
typedef struct {
    bool is_initialized;
    SdFs sd;
} sdfilecontext_t;

#endif

#ifdef USE_SDFILE_SDSPI
typedef struct {
    sdmmc_card_t* sd;
    bool (*spibus_status)(void);
    bool is_initialized;
} sdfilecontext_t;
#endif

#ifdef USE_SDFILE_SDFAT
typedef struct {
    FsFile file;
} sdfile_t;
#endif

#ifdef USE_SDFILE_SDSPI
typedef struct {
    sdfilecontext_t* context;
    FILE* file;
} sdfile_t;
#endif

// Initialize this library
bool sdfile_init(sdfilecontext_t* self, uint32_t spi_freq_khz);

bool sdfile_open_read(sdfilecontext_t* ctx, sdfile_t* self, const char* filepath);

bool sdfile_open_readwrite(sdfilecontext_t* ctx, sdfile_t* self, const char* filepath);

bool sdfile_close(sdfile_t* self);

bool sdfile_get_filesize(sdfile_t* self, size_t* dst);

bool sdfile_seek(sdfile_t* self, size_t offset);

size_t sdfile_read(sdfile_t* self, size_t len, void* dst);

size_t sdfile_write(sdfile_t* self, size_t len, void* src);

void sdfile_flush(sdfile_t* self);

bool sdfile_set_bufsize(sdfile_t* self, size_t buflen);

void sdfile_set_spibus_ready(sdfilecontext_t* self, bool (*func)(void));

#ifdef USE_SDFILE_SDFAT

bool sdfile_init(sdfilecontext_t* self, uint32_t spi_freq_khz) {
    if (self->is_initialized) {
        return true;
    }

    DEBUG("sdfile uses SdFat.");
    if (!self->sd.begin(SD_CS_PIN, spi_freq_khz * 1000UL)) {
        ERROR("Failed to sd.begin() with code %d", self->sd.sdErrorCode());
        return false;
    }

    self->is_initialized = true;
    
    return true;
}

bool sdfile_open_read(sdfilecontext_t* ctx, sdfile_t* self, const char* filepath) {
    self->file = ctx->sd.open(filepath, O_READ);
    if (!self->file) {
        ERROR("Failed to open() %s", filepath);
        return false;
    }
    self->file.seek(0);
    return true;
}

bool sdfile_open_readwrite(sdfilecontext_t* ctx, sdfile_t* self, const char* filepath) {
    self->file = ctx->sd.open(filepath, O_RDWR);
    if (!self->file) {
        ERROR("Failed to open() %s", filepath);
        return false;
    }
    self->file.seek(0);
    return true;
}

bool sdfile_open_write(sdfilecontext_t* ctx, sdfile_t* self, const char* filepath) {
    self->file = ctx->sd.open(filepath, O_WRITE | O_CREAT);
    if (!self->file) {
        ERROR("Failed to open() %s", filepath);
        return false;
    }
    // self->file.seek(0);
    return true;
}

bool sdfile_close(sdfile_t* self) {
    return self->file.close();
}

bool sdfile_get_filesize(sdfile_t* self, size_t* dst) {
    if (!self || !self->file.isOpen()) {
        return false;
    }
    *dst = self->file.fileSize();
    return true;
}

bool sdfile_seek(sdfile_t* self, size_t offset) {
    return self->file.seek(offset);
}

size_t sdfile_read(sdfile_t* self, size_t len, void* dst) {
    int rdlen = self->file.read(dst, len);
    if (rdlen >= 0) {
        return (size_t)rdlen;
    } else {
        DEBUG("SdFat::read() returned %d", rdlen);
        return rdlen;
    }
}

size_t sdfile_write(sdfile_t* self, size_t len, void* src) {
    size_t wrlen = self->file.write(src, len);
    return wrlen;
}

void sdfile_flush(sdfile_t* self) {
    self->file.flush();
}

bool sdfile_set_bufsize(sdfile_t* self, size_t buflen) {
    // Not supported.
    return false;
}

void sdfile_set_spibus_ready(sdfilecontext_t* self, bool (*func)(void)) {
    return;
}

#endif

#ifdef USE_SDFILE_SDSPI

static
void wait_for_spibus_free(sdfilecontext_t* self) {
    if (self->spibus_status) {
        while (!self->spibus_status()) {
            delay(1);
        }
    }
}

bool sdfile_init(sdfilecontext_t* self, uint32_t spi_freq_khz) {
    if (self->is_initialized) {
        return true;
    }

    DEBUG("sdfile uses sdspi in ESP-IDF.");
    wait_for_spibus_free(self);

    esp_err_t ret;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = VSPI_HOST;
    host.max_freq_khz = spi_freq_khz;

    spi_bus_config_t spibuscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 400
    };

    ret = spi_bus_initialize((spi_host_device_t)host.slot, &spibuscfg, 2);
    if (ret != ESP_OK) {
        ERROR("spi_bus_initialize() failed. %d (%s)", ret, esp_err_to_name(ret));
    }

    sdspi_device_config_t slotcfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slotcfg.gpio_cs = (gpio_num_t)SD_CS_PIN;
    slotcfg.host_id = (spi_host_device_t)host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mountcfg = {
        .format_if_mount_failed = false,
        .max_files = SDFILE_MAX_FILES,
        .allocation_unit_size = 16 * 1024
    };

    DEBUG("Mount SDCard");

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slotcfg, &mountcfg, &card);
    if (ret != ESP_OK) {
        ERROR("Failed: esp_vfs_fat_sdspi_mount()  with code %d %s", ret, esp_err_to_name(ret));
        return false;
    }
    self->sd = card;
    self->spibus_status = nullptr;

    sdmmc_card_print_info(stdout, card);
    
    return true;
}

static
bool sdfile_open_with_mode(sdfilecontext_t* ctx, sdfile_t* self, const char* filepath, const char* mode) {
    char filepath2[255];
    strcpy(filepath2, "/sdcard");
    strcpy(filepath2 + strlen(filepath2), filepath);

    wait_for_spibus_free(ctx);

    self->file = fopen(filepath2, mode);
    if (!self->file) {
        ERROR("fopen() failed %s", filepath2);
        return false;
    }
    self->context = ctx;
    return sdfile_set_bufsize(self, SDFILE_SDSPI_DEFUALT_BUFLEN);
}

bool sdfile_open_read(sdfilecontext_t* ctx, sdfile_t* self, const char* filepath) {
    return sdfile_open_with_mode(ctx, self, filepath, "rb");
}

bool sdfile_open_readwrite(sdfilecontext_t* ctx, sdfile_t* self, const char* filepath) {
    return sdfile_open_with_mode(ctx, self, filepath, "r+b");
}

bool sdfile_close(sdfile_t* self) {
    wait_for_spibus_free(self->context);
    return fclose(self->file) == 0;
}

bool sdfile_get_filesize(sdfile_t* self, size_t* dst) {
    wait_for_spibus_free(self->context);
    long orig_pos = ftell(self->file);
    fseek(self->file, 0, SEEK_END);
    long filesize = ftell(self->file);
    fseek(self->file, orig_pos, SEEK_SET);
    *dst = filesize;
    return true;
}

bool sdfile_seek(sdfile_t* self, size_t offset) {
    wait_for_spibus_free(self->context);
    fseek(self->file, offset, SEEK_SET);
    return true;
}

size_t sdfile_read(sdfile_t* self, size_t len, void* dst) {
    wait_for_spibus_free(self->context);
    // return fread(dst, 1, len, self->file);
    size_t rdlen = fread(dst, 1, len, self->file);
    if (rdlen == len) {
        return rdlen;
    } else {
        DEBUG("fread() returned %u but expected %u", rdlen, len);
        return rdlen;
    }
}

size_t sdfile_write(sdfile_t* self, size_t len, void* src) {
    wait_for_spibus_free(self->context);
    return fwrite(src, 1, len, self->file);
}


void sdfile_flush(sdfile_t* self) {
    wait_for_spibus_free(self->context);
    fflush(self->file);
}

bool sdfile_set_bufsize(sdfile_t* self, size_t buflen) {
    return setvbuf(self->file, nullptr, _IOFBF, buflen) == 0;
}


void sdfile_set_spibus_ready(sdfilecontext_t* self, bool (*func)(void)) {
    self->spibus_status = func;
}

#endif
