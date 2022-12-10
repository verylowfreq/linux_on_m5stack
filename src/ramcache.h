#pragma once

#include <stdint.h>
#include <SdFat.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#ifndef ERROR
#define ERROR(...) ((void)0)
#endif
#ifndef DEBUG
#define DEBUG(...) ((void)0)
#endif

#include "ramdevice.h"


class CacheBlockInfo;

enum CacheBlockFlags {
    CACHEBLOCK_CLEARD = 0,
    CACHEBLOCK_WRITTEN = 1 << 0,
    CACHEBLOCK_FETCHED = 1 << 1
};

class CacheBlock {
public:
    RAMDevice* ram;
    uint32_t size;
    uint8_t* data;
    uint32_t startaddr;
    uint32_t lastaccess;
    CacheBlockInfo* info;
    uint32_t flags; // CacheBlockFlags bitflags

    static constexpr uint32_t SECTOR_SIZE = 512;


    CacheBlock(void) : ram(nullptr), size(0), data(nullptr), startaddr(0), lastaccess(0), info(nullptr), flags(CACHEBLOCK_CLEARD) { }

    bool init(RAMDevice& ram, uint32_t size);

    void set_cacheblockinfo(CacheBlockInfo& info);

    bool assign(uint32_t startaddr);

    bool writeback(void);

    bool purge_and_reload(void);

    bool read_bytes(uint32_t addr, uint8_t* dst, uint32_t len, uint32_t counter, bool is_fetch);

    bool write_bytes(uint32_t addr, uint8_t* src, uint32_t len, uint32_t counter);
};



#ifdef USE_CACHEBLOCKINFO
class CacheBlockInfo {
public:
    CacheBlock* self;
    CacheBlockInfo* prev;
    CacheBlockInfo* next;

    CacheBlockInfo(CacheBlockInfo* prev, CacheBlock* self, CacheBlockInfo* next) :
        prev(prev), self(self), next(next) { }

    CacheBlockInfo(CacheBlock* self) : self(self) { }
};

class CacheBlockInfoList {
public:
    CacheBlockInfo* head;
    CacheBlockInfo* tail;
    CacheBlockInfo* list;
    size_t count;

    bool init(size_t count) {
        this->head = nullptr;
        this->tail = nullptr;
        this->count = count;
        this->list = (CacheBlockInfo*)calloc(sizeof(CacheBlockInfo), this->count);
        if (!this->list) {
            ERROR("Failed to malloc() for CacheBlockInfoList %d bytes", sizeof(CacheBlockInfo) * this->count);
            return false;
        }
        return true;
    }

    /** 初期化のためにリストへひとつブロックを登録する */
    bool push_for_init(CacheBlock* block) {
        if (!this->head) {
            // 先頭要素として登録する
            CacheBlockInfo* info = this->get_first_free_element_for_init();
            assert(info);
            info->self = block;
            info->prev = nullptr;
            info->next = nullptr;
            block->info = info;
            this->head = info;
            this->tail = info;
            return true;
        }
        CacheBlockInfo* cur = this->head;
        for (size_t i = 1; i < this->count; i++) {
            if (cur->next) {
                cur = cur->next;
                continue;
            } else {  // (cur->next == nullptr)
                CacheBlockInfo* info = this->get_first_free_element_for_init();
                assert(info);
                info->self = block;
                info->prev = cur;
                info->next = nullptr;
                block->info = info;
                cur->next = info;
                this->tail = info;

                return true;
            }
        }
        return false;
    }

    CacheBlockInfo* get_first_free_element_for_init(void) {
        for (size_t i = 0; i < this->count; i++) {
            if (this->list[i].self == nullptr) {
                return &this->list[i];
            }
        }
        return nullptr;
    }

    bool move_to_head(CacheBlock* block) {
        CacheBlockInfo* target = block->info;

        if (this->head == target) {
            // すでに先頭要素なので、更新作業は必要なし
            return true;
        }
        // targetを先頭要素として登録する
        CacheBlockInfo* orig_head = this->head;
        orig_head->prev = target;
        this->head = target;
        if (this->tail == target) {
            // DEBUG("Move tail to head");
            // targetは末尾の要素だったので、末尾要素への参照を更新する
            // assert(this->tail);
            CacheBlockInfo* newtail = this->tail->prev;
            // assert(this->tail->prev);
            newtail->next = nullptr;
            target->next = orig_head;
            this->tail = newtail;

        } else {
            // DEBUG("Move intermediate to head");
            // targetは末尾ではなく中間の要素だったので、元の場所の前後の参照を張り替える
            CacheBlockInfo* target_prev = target->prev;
            // assert(target_prev);
            CacheBlockInfo* target_next = target->next;
            // assert(target_next);
            target_prev->next = target_next;
            target_next->prev = target_prev;
            target->prev = nullptr;
            target->next = orig_head;
        }

        return true;
    }

    CacheBlock* get_tail(void) {
        return this->tail->self;
    }
};
#endif


class CachedRAM {
public:
    RAMDevice& ram;
    static constexpr uint32_t blockcount = RAMCACHE_BLOCKCOUNT;
    static constexpr uint32_t blocksize = RAMCACHE_BLOCKSIZE;
    static constexpr uint32_t blocksize_mask = RAMCACHE_BLOCKSIZE_MASK;

    CacheBlock** blocks;
    uint32_t counter;
    uint32_t hitcount;
    uint32_t reassigncount;

#ifdef USE_CACHEBLOCKINFO
    CacheBlockInfoList infolist;
#endif

    CachedRAM(RAMDevice& ram) : ram(ram) { }

    bool init(void);

    void flush(void);

    void sort_all(void);

    // 更新されたブロックをひとつ指定し、リスト全体をソートし直す
    // 更新されたブロックの新しい位置を返す
    uint32_t sort_after_update(uint32_t updated_item_index);

    // Do binary search [ head <= index <= end ]
    int bsearch(uint32_t addr, uint32_t head, uint32_t end);

    int search_hit_block(uint32_t addr);

#ifndef USE_CACHEBLOCKINFO
    int search_block_for_reassign(void);
#endif

    int get_or_reassign_cacheblock(uint32_t addr);


    // FIXME: バイトアライメントとページアライメントに合致している前提で高速化している
    //        つまり、1,2,4バイトのアクセスは、キャッシュブロックをまたぐことはないという前提

    bool read_bytes(uint32_t addr, uint8_t* dst, uint32_t len, bool is_fetch = false);

    bool write_bytes(uint32_t addr, uint8_t* src, uint32_t len);

    bool read_u8(uint32_t addr, uint8_t* dst);
    bool write_u8(uint32_t addr, uint8_t val);
    bool read_u16(uint32_t addr, uint16_t* dst);
    bool write_u16(uint32_t addr, uint16_t val);
    bool read_u32(uint32_t addr, uint32_t* dst);
    // 命令フェッチ用の4バイト読み込み
    bool read_u32_instrfetch(uint32_t addr, uint32_t* dst);
    bool write_u32(uint32_t addr, uint32_t val);

    bool write_block(uint32_t addr, uint8_t* src, size_t len);
    bool read_block(uint32_t addr, uint8_t* dst, size_t len);

    bool purge_and_reload(void);

    /** キャッシュを経由せずに直接書き込む。
     * キャッシュの更新をしなかった場合、キャッシュの内容はRAMと一貫性が保てないので、取り扱いに注意。
     */
    bool directwrite(uint32_t startaddr, uint8_t* src, size_t len, bool refresh_cache = true);
};
