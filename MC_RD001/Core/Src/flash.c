/*
 * flash.c
 *
 *  Created on: Nov 4, 2025
 *      Author: TAMRD
 */
#include "flash.h"
#include <string.h>
#include "motor_ctr.h"

static uint8_t flash_buf[FLASH_CFG_REGION_SIZE];

/* ================= Basic ================= */
bool Flash_EraseSimple(void)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
        .Sector       = FLASH_USER_SECTOR,
        .NbSectors    = 1
    };
    uint32_t err = 0;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &err);
    HAL_FLASH_Lock();
    return (st == HAL_OK && err == 0xFFFFFFFF);
}

bool Flash_WriteSimple(uint32_t addr, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t*)data;
    HAL_FLASH_Unlock();
    for (size_t i = 0; i < len; i += 4) {
        uint32_t word = 0xFFFFFFFF;
        memcpy(&word, p + i, (len - i >= 4) ? 4 : (len - i));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

bool FLASH_Clear(uint32_t dest, uint32_t numbytes)
{
    if (numbytes == 0)
        return true;

    uint32_t start = dest;
    uint32_t end   = dest + numbytes - 1;

    // Giới hạn trong 1kB đầu
    if (start < FLASH_USER_BASE_ADDR)
        return false;
    if (end >= FLASH_USER_BASE_ADDR + FLASH_CFG_REGION_SIZE)
        return false;
    if (end < start)
        return false;

    // 1) Đọc 1kB hiện tại vào buffer
    memcpy(flash_buf, (uint8_t*)FLASH_USER_BASE_ADDR, FLASH_CFG_REGION_SIZE);

    // 2) Set 0xFF vùng cần clear trong buffer
    uint32_t off_start = start - FLASH_USER_BASE_ADDR;
    uint32_t off_end   = end   - FLASH_USER_BASE_ADDR;
    for (uint32_t i = off_start; i <= off_end; i++) {
        flash_buf[i] = 0xFF;
    }

    // 3) Tắt ngắt + erase + ghi lại 1kB
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool ok = true;

    if (!Flash_EraseSimple()) {
        ok = false;
    } else if (!Flash_WriteSimple(FLASH_USER_BASE_ADDR,
                                  flash_buf,
                                  FLASH_CFG_REGION_SIZE))
    {
        ok = false;
    }

    __set_PRIMASK(primask);

    return ok;
}

bool FLASH_Update(uint32_t dest, const void *src, uint32_t numbytes)
{
    if (src == NULL || numbytes == 0)
        return false;

    uint32_t start = dest;
    uint32_t end   = dest + numbytes - 1;

    // Giới hạn trong 1kB đầu
    if (start < FLASH_USER_BASE_ADDR)
        return false;
    if (end >= FLASH_USER_BASE_ADDR + FLASH_CFG_REGION_SIZE)
        return false;
    if (end < start)
        return false;

    // 1) Chuẩn bị: clear vùng cần ghi nhưng giữ lại phần còn lại
    if (!FLASH_Clear(dest, numbytes)) {
        return false;
    }

    // 2) Ghi dữ liệu mới (không cần erase lại)
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool ok = Flash_WriteSimple(dest, src, numbytes);

    __set_PRIMASK(primask);

    return ok;
}

bool Flash_ReadSimple(uint32_t addr, void *buf, size_t len)
{
    memcpy(buf, (const void*)addr, len);
    return true;
}

void Handle_GL1(void)
{
	Coils_Database[4] &= ~(1 << 0);
	Holding_Registers_Database[3] = (uint16_t) Axis.X.position;
	Holding_Registers_Database[4] = (uint16_t) Axis.Y.position;
	Coils_Database[3] |= (1 << 0);
}
void Handle_GL2(void)
{
	Coils_Database[4] &= ~(1 << 1);
	Holding_Registers_Database[5] = (uint16_t) Axis.X.position;
	Holding_Registers_Database[6] = (uint16_t) Axis.Y.position;
	Coils_Database[3] |= (1 << 1);
}
void Handle_GL3(void)
{
	Coils_Database[4] &= ~(1 << 2);
	Holding_Registers_Database[7] = (uint16_t) Axis.X.position;
	Holding_Registers_Database[8] = (uint16_t) Axis.Y.position;
	Coils_Database[3] |= (1 << 2);
}
void Handle_CV1(void)
{
	Coils_Database[4] &= ~(1 << 3);
	Holding_Registers_Database[9] = (uint16_t) Axis.X.position;
	Holding_Registers_Database[10] = (uint16_t) Axis.Y.position;
	Coils_Database[3] |= (1 << 3);
}
void Handle_CV2(void)
{
	Coils_Database[4] &= ~(1 << 4);
	Holding_Registers_Database[11] = (uint16_t) Axis.X.position;
	Holding_Registers_Database[12] = (uint16_t) Axis.Y.position;
	Coils_Database[3] |= (1 << 4);
}
void Handle_CV3(void)
{
	Coils_Database[4] &= ~(1 << 5);
	Holding_Registers_Database[13] = (uint16_t) Axis.X.position;
	Holding_Registers_Database[14] = (uint16_t) Axis.Y.position;
	Coils_Database[3] |= (1 << 5);
}
void Handle_GL_save(void)
{
	Coils_Database[4] &= ~(1 << 6);
	FLASH_Update(FLASH_USER_BASE_ADDR, &Holding_Registers_Database[3], 12);
}
void Handle_CV_save(void)
{
	Coils_Database[4] &= ~(1 << 7);
	FLASH_Update(FLASH_USER_BASE_ADDR+12, &Holding_Registers_Database[9], 12);
}

///* ================= Save check (header + CRC) ================= */
///* ================= CRC32 ================= */
//uint32_t FLASH_CalcCRC32(const void *data, size_t len)
//{
//    const uint8_t *p = (const uint8_t*)data;
//    uint32_t crc = 0xFFFFFFFF;
//    for (size_t i = 0; i < len; ++i) {
//        crc ^= p[i];
//        for (int k = 0; k < 8; k++)
//            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
//    }
//    return ~crc;
//}
//bool Flash_EraseChecked(void)
//{
//    return Flash_EraseSimple();
//}
//
//bool Flash_SaveChecked(const void *data, size_t len)
//{
//    FlashHeader_t hdr;
//    hdr.magic  = FLASH_MAGIC;
//    hdr.length = len;
//    hdr.crc32  = FLASH_CalcCRC32(data, len);
//
//    if (sizeof(hdr) + len > FLASH_USER_SECTOR_SIZE)
//        return false;
//
//    if (!Flash_EraseChecked()) return false;
//
//    HAL_FLASH_Unlock();
//    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_BASE_ADDR, *((uint32_t*)&hdr)) != HAL_OK) {
//        HAL_FLASH_Lock();
//        return false;
//    }
//    /* Ghi toàn bộ header */
//    if (!Flash_WriteSimple(FLASH_USER_BASE_ADDR, &hdr, sizeof(hdr))) {
//        HAL_FLASH_Lock();
//        return false;
//    }
//    /* Ghi payload */
//    if (!Flash_WriteSimple(FLASH_USER_BASE_ADDR + sizeof(hdr), data, len)) {
//        HAL_FLASH_Lock();
//        return false;
//    }
//    HAL_FLASH_Lock();
//    return true;
//}
//
//bool Flash_LoadChecked(void *buf, size_t maxlen, size_t *out_len)
//{
//    const FlashHeader_t *hdr = (const FlashHeader_t*)FLASH_USER_BASE_ADDR;
//    if (hdr->magic != FLASH_MAGIC) return false;
//    if (hdr->length == 0 || hdr->length > maxlen) return false;
//
//    const uint8_t *payload = (const uint8_t*)(FLASH_USER_BASE_ADDR + sizeof(FlashHeader_t));
//    uint32_t crc = FLASH_CalcCRC32(payload, hdr->length);
//    if (crc != hdr->crc32) return false;
//
//    memcpy(buf, payload, hdr->length);
//    if (out_len) *out_len = hdr->length;
//    return true;
//}

