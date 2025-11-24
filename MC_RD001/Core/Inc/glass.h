/*
 * glass.h
 *
 *  Created on: Nov 10, 2025
 *      Author: TAMRD
 */

#ifndef INC_GLASS_H_
#define INC_GLASS_H_

#include "flash.h"
#include "modbus_task.h"
bool LoadCornerData(CornerData_t *out);
typedef struct {
    float x;
    float y;
} Point2D_t;

#define GRID_SIZE   14
#define NUM_CELLS   (GRID_SIZE * GRID_SIZE)  // 196
#define PANEL_COUNT         4
#define PANEL_BITS_WORDS  7

typedef struct {
    CornerData_t geom;                       // 12 bytes
    uint32_t    quality_bits[PANEL_BITS_WORDS]; // 196 bit = OK/NG
} PanelPacked_t;

extern PanelPacked_t Glass[PANEL_COUNT];
extern Home_state_t* Home_state;

void Panel_Init(PanelPacked_t *p);
void Panel_InitAll(void);
Point2D_t Panel_GetCellCenter(const PanelPacked_t *p, uint8_t i, uint8_t j);
void Panel_CheckQuality(PanelPacked_t *panel);

typedef enum {
    SCAN_IDLE,
    SCAN_MOVING,
    SCAN_WAIT_STABLE,
    SCAN_READING,
    SCAN_NEXT_CELL,
    SCAN_COMPLETE,
    SCAN_NEXT_TRAY
} ScanState_t;

typedef struct {
    ScanState_t state;
    PanelPacked_t *panel;

    uint8_t current_i;
    uint8_t current_j;

    uint32_t stable_timer;
    uint32_t read_timer;

    bool scan_running;

    uint8_t start_tray;
    uint8_t end_tray;
    uint8_t current_tray;
} PanelScanner_t;

void PanelScanner_Init(void);
void PanelScanner_Start(PanelPacked_t *panel);
void PanelScanner_StartRange(uint8_t start_tray, uint8_t end_tray);
void PanelScanner_Update(void);  // Gọi trong main loop
bool PanelScanner_IsBusy(void);
void PanelScanner_Stop(void);
void Map_QualityBits_To_Inputs(const uint32_t *quality_bits, uint8_t count_words);

#endif /* INC_GLASS_H_ */
