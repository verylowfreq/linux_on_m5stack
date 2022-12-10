#pragma once

#include <cstring>
#include <cstdint>

#include "debug.h"
#include "config.h"
#include "ramcache.h"


// ---- Forward declarations

class RAMDevice;
class CacheBlockInfo;


bool CacheBlock::init(RAMDevice& ram, uint32_t size) {
    this->ram = &ram;
    this->size = size;
    this->data = (uint8_t*)malloc(this->size);
    if (!this->data) {
        ERROR("Failed to malloc() %d bytes for RAM cache.", this->size);
        return false;
    }
    this->startaddr = 0;
    this->lastaccess = 0;
    this->flags = CACHEBLOCK_CLEARD;
    return true;
}

void CacheBlock::set_cacheblockinfo(CacheBlockInfo& info) {
    this->info = &info;
}

// Not used.
// bool is_hit(uint32_t targetaddr) const {
//     return this->startaddr == (targetaddr & ~this->blocksize_mask);
// }

bool CacheBlock::assign(uint32_t startaddr) {
    assert(this->writeback());
    this->startaddr = startaddr;
    assert(this->ram->read_block(this->startaddr, this->data, this->size));
    // this->is_written = false;
    this->flags = 0;
    return true;
}

bool CacheBlock::writeback(void) {
    if (this->flags & CACHEBLOCK_WRITTEN) {
        assert(this->ram->write_block(this->startaddr, this->data, this->size));

        this->flags = CACHEBLOCK_CLEARD;
    }
    return true;
}

bool CacheBlock::purge_and_reload(void) {
    // this->is_written = false;
    this->flags = CACHEBLOCK_CLEARD;
    this->assign(this->startaddr);
    return true;
}

bool CacheBlock::read_bytes(uint32_t addr, uint8_t* dst, uint32_t len, uint32_t counter, bool is_fetch) {
    // NOTE: 実際には4バイト、2バイト、1バイトのどれかしか要求されない
    memcpy(dst, &this->data[addr - this->startaddr], len);
#ifndef USE_CACHEBLOCKINFO
    this->lastaccess = counter;
#endif
    if (is_fetch) {
        this->flags |= CACHEBLOCK_FETCHED;
    }
    return true;
}


bool CacheBlock::write_bytes(uint32_t addr, uint8_t* src, uint32_t len, uint32_t counter) {
    // NOTE: 実際には4バイト、2バイト、1バイトのどれかしか要求されない
    memcpy(&this->data[addr - this->startaddr], src, len);
#ifndef USE_CACHEBLOCKINFO
    this->lastaccess = counter;
#endif
    this->flags |= CACHEBLOCK_WRITTEN;
    return true;
}

#ifdef USE_LRU_MEMMOVE

/** リスト内の要素を移動する
 * @returns 空いた領域の先頭インデックス
 */
static
uint32_t move_items(size_t len, CacheBlock** list, uint32_t head, uint32_t tail, int delta) {
    CacheBlock** dst = &list[head + delta];
    CacheBlock** src = &list[head];
    size_t movelen = tail - head + 1;
    memmove(dst, src, movelen * sizeof(CacheBlock*));
    if (delta > 0) {
        return head;
    } else {
        return tail;
    }
}

#endif


bool CachedRAM::init(void) {
    const size_t blocklistlen = sizeof(CacheBlock*) * this->blockcount;
    this->blocks = (CacheBlock**)malloc(blocklistlen);
    if (!this->blocks) {
        PANIC("Failed to malloc() for block list %d bytes (for %d blocks)", blocklistlen, this->blockcount);
    }
    const size_t blockpoollen = sizeof(CacheBlock) * this->blockcount;
    CacheBlock* blockpool = (CacheBlock*)malloc(blockpoollen);
    if (!blockpool) {
        PANIC("Failed to malloc() for block pool %d bytes.", blockpoollen);
    }
    for (int i = 0; i < this->blockcount; i++) {
        this->blocks[i] = &blockpool[i];
        assert(this->blocks[i]);
        if (!this->blocks[i]->init(this->ram, this->blocksize)) {
            PANIC("Failed to init() RAM cache %d", i);
        }
    }
    this->counter = 0;
    this->hitcount = 0;
    this->reassigncount = 0;
#ifdef USE_CACHEBLOCKINFO
    if (!this->infolist.init(this->blockcount)) {
        PANIC("Failed to init() CacheBlockInfoList");
    }
#endif
    for (int i = 0; i < this->blockcount; i++) {
        uint32_t addr = i * this->blocksize;
        if (!this->blocks[i]->assign(addr)) {
            PANIC("Failed to assign() RAM cache (i=%d,addr=%08x)", i, addr);
        }
#ifdef USE_CACHEBLOCKINFO
        if (!this->infolist.push_for_init(this->blocks[i])) {
            PANIC("Failed to push_for_init() at %d", i);
        }
#endif
    }
    this->sort_all();

    return true;
}

void CachedRAM::flush(void) {
    for (int i = 0; i < this->blockcount; i++) {
        this->blocks[i]->writeback();
    }
}

uint32_t CachedRAM::sort_after_update(uint32_t updated_item_index) {
#ifndef USE_LRU_MEMMOVE
    uint32_t target = this->blocks[updated_item_index]->startaddr;
    this->sort_all();
    return this->search_hit_block(target);

#else
    const uint32_t head = 0;
    const uint32_t tail = this->blockcount - 1;
    // 挿入されたアイテム以外はすべてソート済みであることを利用した最適化

    const uint32_t updated_value = this->blocks[updated_item_index]->startaddr;

    if (updated_item_index != 0) {
        const uint32_t head_value = this->blocks[head]->startaddr;
        if (updated_value <= head_value) {
            // 先頭にはないが、先頭が最適位置の場合
            CacheBlock* tmp = this->blocks[updated_item_index];
            move_items(this->blockcount, this->blocks, head, updated_item_index - 1, 1);
            this->blocks[head] = tmp;
            return head;
        }
    }

    if (updated_item_index != tail) {
        const uint32_t tail_value = this->blocks[tail]->startaddr;
        if (tail_value <= updated_value) {
            // 末尾にはないが、末尾が最適位置の場合
            CacheBlock* tmp = this->blocks[updated_item_index];
            move_items(this->blockcount, this->blocks, updated_item_index + 1, tail, -1);
            this->blocks[tail] = tmp;
            return tail;
        }
    }

    if (updated_item_index > 0) {
        const uint32_t prev_value = this->blocks[updated_item_index - 1]->startaddr;
        if (updated_value < prev_value) {
            // 2番目以降の位置で、先頭に向けて検索する
             for (int i = updated_item_index; i > 0; i--) {
                uint32_t cur_value = this->blocks[i - 1]->startaddr;
                if (updated_value > cur_value) {
                    uint32_t newpos = i;
                    CacheBlock* tmp = this->blocks[updated_item_index];
                    move_items(this->blockcount, this->blocks, i, updated_item_index - 1, 1);
                    this->blocks[newpos] = tmp;
                    return newpos;
                }
             }
        }
    }

    if (updated_item_index < tail) {
        const uint32_t next_value = this->blocks[updated_item_index + 1]->startaddr;
        if (next_value < updated_value) {
            // 末尾より2番目以前で、末尾に向けて検索する
            for (int i = updated_item_index; i < tail; i++) {
                uint32_t cur_value = this->blocks[i + 1]->startaddr;
                if (updated_value < cur_value) {
                    CacheBlock* tmp = this->blocks[updated_item_index];
                    move_items(this->blockcount, this->blocks, updated_item_index + 1, i, -1);
                    this->blocks[i] = tmp;
                    return i;
                }
            }
        }
    }

    // すでに最適位置にある

    return updated_item_index;
#endif
}

void CachedRAM::sort_all(void) {
    uint32_t head = 0;
    uint32_t end = this->blockcount - 1;
    for (int i = head; i <= end; i++) {
        uint32_t j = i;
        while (j >= head + 1) {
            uint32_t cur = this->blocks[j]->startaddr;
            uint32_t prev = this->blocks[j - 1]->startaddr;
            if (prev <= cur) {
                break;
            } else {
                // Swap item
                CacheBlock* tmp = this->blocks[j];
                this->blocks[j] = this->blocks[j - 1];
                this->blocks[j - 1] = tmp;
                j -= 1;
            }
        }
    }
}

// Do binary search [ head <= index <= end ]
int CachedRAM::bsearch(uint32_t addr, uint32_t head, uint32_t end) {
    uint32_t target = addr & ~RAMCACHE_BLOCKSIZE_MASK;
    while (true) {
        uint32_t pivotidx = (end - head) / 2 + head;
        uint32_t pivotvalue = this->blocks[pivotidx]->startaddr;
        if (pivotvalue == target) {
            return pivotidx;
        } else if (head == pivotidx) {
            // 区間の先頭と中央が一致することはありうる
            // この場合、後ろに一つだけ要素があるので、それをチェックする
            if (this->blocks[pivotidx + 1]->startaddr == target) {
                return pivotidx + 1;
            } else {
                return -1;
            }
        } else if (end == pivotidx) {
            // 前寄りの中央を取るので、終端要素が中央となることはないが、念のため
            return -1;
        } else if (pivotvalue < target) {
            head = pivotidx;
            continue;
        } else { // (target < pivotvalue)
            end = pivotidx;
            continue;
        }
    }
}

int CachedRAM::search_hit_block(uint32_t addr) {
    return this->bsearch(addr, 0, this->blockcount - 1);
}

#ifndef USE_CACHEBLOCKINFO
int CachedRAM::search_block_for_reassign(void) {
    uint32_t now = this->counter;
    int startidx = 0;
    int tailidx = this->blockcount;
    int candidate = startidx;
    uint32_t candidate_delta = 0;
    // データ読込のみのブロックの候補
    int candidate_dataread = 0;
    // データ読込のみのブロックでの差分
    uint32_t candidate_delta_dataread = 0;
    for (int i = startidx; i < tailidx; i++) {
        uint32_t elapsed = now - this->blocks[i]->lastaccess;
        if ((this->blocks[i]->flags & (CACHEBLOCK_WRITTEN | CACHEBLOCK_FETCHED)) == 0) {
            // データとしての読み込みアクセスしかないブロック
            if (elapsed > candidate_delta_dataread) {
                candidate_dataread = i;
                candidate_delta_dataread = elapsed;
            }
        } else {
            // 書き込まれたか命令フェッチされたブロック
            if (elapsed > candidate_delta) {
                candidate = i;
                candidate_delta = elapsed;
            }
        }
    }

#ifdef USE_CACHEBLOCK_WEIGHT
    // なんらかの重みをつけた比較を元に、どちらの候補を採用するかを決定する

    if (candidate_delta / 4 < candidate_delta_dataread) {
        return candidate_dataread;
    } else {
        return candidate;
    }
#else
    if (candidate_delta > candidate_delta_dataread) {
        return candidate;
    } else {
        return candidate_dataread;
    }
#endif
}
#endif

int CachedRAM::get_or_reassign_cacheblock(uint32_t addr) {
    int blkid = this->search_hit_block(addr);
    if (blkid >= 0) {
        // DEBUG("Already cached %08x", addr);
        this->hitcount += 1;
#ifdef USE_CACHEBLOCKINFO
        assert(this->infolist.move_to_head(this->blocks[blkid]));
#endif
        return blkid;
    } else {
        // DEBUG("Reassign for %08x", addr);
        this->reassigncount += 1;
#ifdef USE_CACHEBLOCKINFO
        CacheBlock* block_for_reassign = this->infolist.get_tail();
        uint32_t startaddr = addr & ~RAMCACHE_BLOCKSIZE_MASK;
        assert(block_for_reassign->assign(startaddr));
        assert(this->infolist.move_to_head(block_for_reassign));
        this->sort_blocks();
        blkid = this->search_hit_block(addr);
        return blkid;
#else
        blkid = this->search_block_for_reassign();
        uint32_t startaddr = addr & ~(this->blocksize - 1);
        assert(this->blocks[blkid]->assign(startaddr));
        // this->sort_blocks();
        blkid = this->sort_after_update(blkid);
        // blkid = this->search_hit_block(addr);
        return blkid;
#endif
    }
}


// FIXME: バイトアライメントとページアライメントに合致している前提で高速化している
//        つまり、1,2,4バイトのアクセスは、キャッシュブロックをまたぐことはないという前提

bool CachedRAM::read_bytes(uint32_t addr, uint8_t* dst, uint32_t len, bool is_fetch) {
    int blkid = this->get_or_reassign_cacheblock(addr);
    return this->blocks[blkid]->read_bytes(addr, dst, len, this->counter++, is_fetch);
}

bool CachedRAM::write_bytes(uint32_t addr, uint8_t* src, uint32_t len) {
    int blkid = this->get_or_reassign_cacheblock(addr);
    return this->blocks[blkid]->write_bytes(addr, src, len, this->counter++);
}

bool CachedRAM::read_u8(uint32_t addr, uint8_t* dst) {
    return this->read_bytes(addr, dst, 1);
}
bool CachedRAM::write_u8(uint32_t addr, uint8_t val) {
    return this->write_bytes(addr, &val, 1);
}
bool CachedRAM::read_u16(uint32_t addr, uint16_t* dst) {
    return this->read_bytes(addr, (uint8_t*)dst, 2);
}
bool CachedRAM::write_u16(uint32_t addr, uint16_t val) {
    return this->write_bytes(addr, (uint8_t*)&val, 2);
}
bool CachedRAM::read_u32(uint32_t addr, uint32_t* dst) {
    return this->read_bytes(addr, (uint8_t*)dst, 4);
}
// 命令フェッチ用の4バイト読み込み
bool CachedRAM::read_u32_instrfetch(uint32_t addr, uint32_t* dst) {
    return this->read_bytes(addr, (uint8_t*)dst, 4, true);
}
bool CachedRAM::write_u32(uint32_t addr, uint32_t val) {
    return this->write_bytes(addr, (uint8_t*)&val, 4);
}

bool CachedRAM::write_block(uint32_t addr, uint8_t* src, size_t len) {
    // NOTE: 遅い実装だが、メインのエミュレーター処理から呼ばれることはないので許容する
    for (size_t i = 0; i < len; i++) {
        int blkid = this->get_or_reassign_cacheblock(addr);
        assert(this->blocks[blkid]->write_bytes(addr++, src++, 1, this->counter++));
    }
    return true;
}
bool CachedRAM::read_block(uint32_t addr, uint8_t* dst, size_t len) {
    // NOTE: 遅い実装だが、メインのエミュレーター処理から呼ばれることはないので許容する
    for (size_t i = 0; i < len; i++) {
        int blkid = this->get_or_reassign_cacheblock(addr);
        assert(this->blocks[blkid]->read_bytes(addr++, dst++, 1, this->counter++, false));
    }
    return true;
}

bool CachedRAM::purge_and_reload(void) {
    // すべてのブロックをRAMから読み込みなおす
    for (int i = 0; i < RAMCACHE_BLOCKCOUNT; i++) {
        assert(this->blocks[i]->purge_and_reload());
    }
    return true;
}

/** キャッシュを経由せずに直接書き込む。
 * キャッシュの更新をしなかった場合、キャッシュの内容はRAMと一貫性が保てないので、取り扱いに注意。
 */
bool CachedRAM::directwrite(uint32_t startaddr, uint8_t* src, size_t len, bool refresh_cache) {
    // すべてのブロックを書き込みさせる
    for (int i = 0; i < RAMCACHE_BLOCKCOUNT; i++) {
        assert(this->blocks[i]->writeback());
    }

    if (src) {
        // RAMへ直接書き込む
        assert(this->ram.write_block(startaddr, src, len));
    }

    if (refresh_cache) {
        assert(this->purge_and_reload());
    }

    return true;
}
