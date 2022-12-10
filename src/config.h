#pragma once

#define RAMCACHE_BLOCKCOUNT      224
#define RAMCACHE_BLOCKSIZE      1024
#define RAMCACHE_BLOCKSIZE_MASK (RAMCACHE_BLOCKSIZE - 1)


// Use linked list LRU Cache Block
//   No effect.
// #define USE_CACHEBLOCKINFO

// Use memmove() instead of insert sort
//   No effect.
// #define USE_LRU_MEMMOVE

// Block that data read only will be reassign first.
#define USE_CACHEBLOCK_WEIGHT


// #define SD_FAT_TYPE 1  // Use FAT16 and FAT32
#define SDFAT_FILE_TYPE 3  // Use FAT16, FAT32 and exFAT

#define SPI_SPEED SD_SCK_MHZ(26)
#define SD_CS_PIN 4  // == TFCARD_CS_PIN
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
// 1, 10, 20, 40 are available.
// #define SDFILE_SDMMC_FREQ (40 * 1000)
// #define SDFILE_MAX_FILES 5

// Use SdFat library to access SD card.
#define USE_SDFILE_SDFAT
// Not worked. Use sdspi api in ESP-IDF to access SD card.
// #define USE_SDFILE_SDSPI

#define RAMDEVICE_SDFILE_BUFLEN 4096

// #define SDFILE_SDSPI_DEFUALT_BUFLEN 4096
