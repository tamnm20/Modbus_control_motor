/*
 * flash.h
 *
 *  Created on: Nov 4, 2025
 *      Author: TAMRD
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define FLASH_USER_SECTOR        FLASH_SECTOR_7
#define FLASH_USER_BASE_ADDR     ((uint32_t)0x08060000)
#define FLASH_USER_SECTOR_SIZE   (128UL * 1024UL)
#define FLASH_CFG_REGION_SIZE    (256U)

typedef struct {
    uint16_t x;
    uint16_t y;
} Point16_t;

typedef struct {
    Point16_t glassCorner1;
    Point16_t glassCorner2;
    Point16_t glassCorner3;
} G_CornerData_t;

typedef struct {
    Point16_t coverCorner1;
    Point16_t coverCorner2;
    Point16_t coverCorner3;
} C_CornerData_t;

extern uint8_t Inputs_Database[50];
extern uint8_t Coils_Database[25];
extern uint16_t Holding_Registers_Database[50];

bool Flash_EraseSimple(void);
bool Flash_WriteSimple(uint32_t addr, const void *data, size_t len);
bool FLASH_Clear(uint32_t dest, uint32_t numbytes);
bool FLASH_Update(uint32_t dest, const void *src, uint32_t numbytes);
bool Flash_ReadSimple(uint32_t addr, void *buf, size_t len);

void Handle_GL1(void);
void Handle_GL2(void);
void Handle_GL3(void);
void Handle_GL_save(void);
void Handle_CV1(void);
void Handle_CV2(void);
void Handle_CV3(void);
void Handle_CV_save(void);

///* ===== Header cho kiểu "có check" ===== */
//typedef struct __attribute__((packed)) {
//    uint32_t magic;      // Nhận dạng (0x464C5348 = 'FLSH')
//    uint32_t length;     // Chiều dài dữ liệu thực
//    uint32_t crc32;      // CRC32 của payload
//} FlashHeader_t;
//#define FLASH_MAGIC  (0x464C5348UL)
///* ===== CRC ===== */
//uint32_t FLASH_CalcCRC32(const void *data, size_t len);
///* ====== Kiểu 2: có header + CRC ====== */
//bool Flash_SaveChecked(const void *data, size_t len);
//bool Flash_LoadChecked(void *buf, size_t maxlen, size_t *out_len);
//bool Flash_EraseChecked(void);


#endif /* INC_FLASH_H_ */
