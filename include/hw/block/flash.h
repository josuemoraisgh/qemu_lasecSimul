#ifndef HW_FLASH_H
#define HW_FLASH_H

/* NOR flash devices */

#include "exec/hwaddr.h"
#include "qom/object.h"

/* m25p80.c */

/* E120 (EVIDENCE.md, 2026-09-05): reads `bytes` bytes starting at `offset` out of the flash
 * chip's own coherent, operational RAM copy (`Flash::storage` -- the same array
 * flash_write8()/flash_erase() mutate directly and synchronously; BlockBackend I/O is only the
 * asynchronous, fire-and-forget persistence path behind it, never awaited here). Bytes reflect
 * every program/erase already applied by the guest, with no dependency on whether that async
 * persistence has reached the backing file yet.
 *
 * `dev` must be a realized device of type "m25p80-generic" or a registered subtype (e.g.
 * "gd25q32", the model this fork actually instantiates for ESP32 -- see
 * hw/xtensa/esp32.c:esp32_machine_init_spi_flash()). The `Flash` struct itself stays private to
 * m25p80.c; callers only ever see this DeviceState* handle.
 *
 * Bytes are copied into `destination` (caller-owned, at least `bytes` long) rather than handing
 * back a pointer into `Flash::storage` -- `Flash::storage` can be reallocated by nothing in this
 * device model today, but a raw pointer would still carry an implicit, undocumented lifetime
 * contract this API deliberately avoids.
 *
 * Must be called with the BQL held (asserted internally) -- `Flash::storage` is mutated by guest
 * SPI command processing under the same lock, with no additional synchronization of its own.
 *
 * Returns false and sets `*errp` on an invalid range (offset+bytes overflow, or exceeding the
 * flash's own configured size) instead of silently truncating or reading out of bounds. Callers
 * that already validated MMU-derived offsets against a known-good cache layout may pass errp=NULL
 * only if they treat a false return as a caller bug they still must not read from `destination`
 * after (this function never partially fills `destination` on failure). */
bool m25p80_read_array(DeviceState *dev, uint64_t offset, uint64_t bytes,
                       void *destination, Error **errp);

/* pflash_cfi01.c */

#define TYPE_PFLASH_CFI01 "cfi.pflash01"
OBJECT_DECLARE_SIMPLE_TYPE(PFlashCFI01, PFLASH_CFI01)


PFlashCFI01 *pflash_cfi01_register(hwaddr base,
                                   const char *name,
                                   hwaddr size,
                                   BlockBackend *blk,
                                   uint32_t sector_len,
                                   int width,
                                   uint16_t id0, uint16_t id1,
                                   uint16_t id2, uint16_t id3,
                                   int be);
BlockBackend *pflash_cfi01_get_blk(PFlashCFI01 *fl);
MemoryRegion *pflash_cfi01_get_memory(PFlashCFI01 *fl);
void pflash_cfi01_legacy_drive(PFlashCFI01 *dev, DriveInfo *dinfo);

/* pflash_cfi02.c */

#define TYPE_PFLASH_CFI02 "cfi.pflash02"
OBJECT_DECLARE_SIMPLE_TYPE(PFlashCFI02, PFLASH_CFI02)


PFlashCFI02 *pflash_cfi02_register(hwaddr base,
                                   const char *name,
                                   hwaddr size,
                                   BlockBackend *blk,
                                   uint32_t sector_len,
                                   int nb_mappings,
                                   int width,
                                   uint16_t id0, uint16_t id1,
                                   uint16_t id2, uint16_t id3,
                                   uint16_t unlock_addr0,
                                   uint16_t unlock_addr1,
                                   int be);

/* nand.c */
DeviceState *nand_init(BlockBackend *blk, int manf_id, int chip_id);
void nand_setpins(DeviceState *dev, uint8_t cle, uint8_t ale,
                  uint8_t ce, uint8_t wp, uint8_t gnd);
void nand_getpins(DeviceState *dev, int *rb);
void nand_setio(DeviceState *dev, uint32_t value);
uint32_t nand_getio(DeviceState *dev);
uint32_t nand_getbuswidth(DeviceState *dev);

#define NAND_MFR_TOSHIBA    0x98
#define NAND_MFR_SAMSUNG    0xec
#define NAND_MFR_FUJITSU    0x04
#define NAND_MFR_NATIONAL   0x8f
#define NAND_MFR_RENESAS    0x07
#define NAND_MFR_STMICRO    0x20
#define NAND_MFR_HYNIX      0xad
#define NAND_MFR_MICRON     0x2c

/* onenand.c */
void *onenand_raw_otp(DeviceState *onenand_device);

/* ecc.c */
typedef struct {
    uint8_t cp;     /* Column parity */
    uint16_t lp[2]; /* Line parity */
    uint16_t count;
} ECCState;

uint8_t ecc_digest(ECCState *s, uint8_t sample);
void ecc_reset(ECCState *s);
extern const VMStateDescription vmstate_ecc_state;

#endif

