/*
 * glass.c
 *
 *  Created on: Nov 10, 2025
 *      Author: TAMRD
 */

#include <stdint.h>
#include "glass.h"
#include "axis_task.h"

PanelPacked_t Glass[PANEL_COUNT];
static PanelScanner_t scanner = {0};
#define STABLE_DELAY_MS  50
#define SENSOR_TIMEOUT_MS 100

bool LoadCornerData(CornerData_t *out)
{
    if (out == NULL) return false;

    const uint16_t *src = (const uint16_t *)FLASH_USER_BASE_ADDR;

    for (int i = 0; i < 12; i++) {
        ((uint16_t*)out)[i] = src[i];
        Holding_Registers_Database[3+i] = src[i];
    }
    return true;
}
static inline uint16_t panel_idx(uint8_t i, uint8_t j)
{
    return (uint16_t)(j * GRID_SIZE + i);
}
void Set_InputBit(uint16_t bit_index)
{
    if (bit_index >= 196) return;

    uint16_t byte = 1 + (bit_index / 8);
    uint8_t  bit  = bit_index % 8;

    Inputs_Database[byte] |= (1u << bit);
}

// Clear bit
void Clear_InputBit(uint16_t bit_index)
{
    if (bit_index >= 196) return;

    uint16_t byte = 1 + (bit_index / 8);
    uint8_t  bit  = bit_index % 8;

    Inputs_Database[byte] &= ~(1u << bit);
}

// Get bit
uint8_t Get_InputBit(uint16_t bit_index)
{
    if (bit_index >= 196) return 0;

    uint16_t byte = 1 + (bit_index / 8);
    uint8_t  bit  = bit_index % 8;

    return (Inputs_Database[byte] >> bit) & 1u;
}
static inline void panel_set_ok(PanelPacked_t *p, uint8_t i, uint8_t j)
{
    uint16_t idx = panel_idx(i,j);
    uint8_t  w   = idx / 32;
    uint8_t  b   = idx % 32;
    p->quality_bits[w] |= (1u << b);
    Set_InputBit(idx);
}

static inline void panel_set_ng(PanelPacked_t *p, uint8_t i, uint8_t j)
{
    uint16_t idx = panel_idx(i,j);
    uint8_t  w   = idx / 32;
    uint8_t  b   = idx % 32;
    p->quality_bits[w] &= ~(1u << b);
    Clear_InputBit(idx);
}

static inline uint8_t panel_is_ok(const PanelPacked_t *p, uint8_t i, uint8_t j)
{
    uint16_t idx = panel_idx(i,j);
    uint8_t  w   = idx / 32;
    uint8_t  b   = idx % 32;
    return (p->quality_bits[w] >> b) & 0x01u;
}

void Panel_Init(PanelPacked_t *p)
{
    LoadCornerData(&p->geom);

    for (int k = 0; k < PANEL_BITS_WORDS; k++) {
        p->quality_bits[k] = 0x00000000u;
    }
}

void Panel_InitAll(void)
{
    for (int i = 1; i <= 25; i++)
        Inputs_Database[i] = 0;
    for(uint8_t i = 0; i < PANEL_COUNT; i++)
    {
        Panel_Init(&Glass[i]);
    }
}

Point2D_t Panel_GetCellCenter(const PanelPacked_t *p, uint8_t i, uint8_t j)
{
    Point2D_t pt;
    float x1 = p->geom.glassCorner1.x;
    float y1 = p->geom.glassCorner1.y;
    float x2 = p->geom.glassCorner2.x;
    float y2 = p->geom.glassCorner2.y;
    float x3 = p->geom.glassCorner3.x;
    float y3 = p->geom.glassCorner3.y;

    pt.x = x1 + i * (x2 - x1) / 13.0f + j * (x3 - x1) / 13.0f;
    pt.y = y1 + i * (y2 - y1) / 13.0f + j * (y3 - y1) / 13.0f;
    return pt;
}

static bool Sensor_ReadQuality(void)
{
    // TODO: thay bằng code đọc cảm biến thực tế
    //  - Đọc giá trị ADC / tín hiệu digital
    //  - Xử lý ngưỡng hoặc logic phân loại OK/NG
    // Tạm thời: random mô phỏng
    static uint32_t seed = 1234567;
    seed = seed * 1103515245 + 12345;
    return (seed >> 16) & 1;  // ngẫu nhiên 0 hoặc 1
}

/* ========================================
 * INIT
 * ======================================== */
void PanelScanner_Init(void)
{
    scanner.state = SCAN_IDLE;
    scanner.panel = NULL;
    scanner.current_i = 0;
    scanner.current_j = 0;
    scanner.scan_running = false;
    scanner.start_tray = 0;
    scanner.end_tray = 0;
    scanner.current_tray = 0;
}
void PanelScanner_StartRange(uint8_t start_tray, uint8_t end_tray)
{
    if(scanner.scan_running)
        return;

    // Validate
    if(start_tray >= PANEL_COUNT) start_tray = 0;
    if(end_tray >= PANEL_COUNT) end_tray = PANEL_COUNT - 1;
    if(start_tray > end_tray) start_tray = end_tray;

    // Setup range
    scanner.start_tray = start_tray;
    scanner.end_tray = end_tray;
    scanner.current_tray = start_tray;

    // Init tấm đầu tiên
    for (int i = 1; i <= 25; i++){
        Inputs_Database[i] = 0;
    }
    Panel_Init(&Glass[scanner.current_tray]);

    // Setup scan
    scanner.panel = &Glass[scanner.current_tray];
    scanner.current_i = 0;
    scanner.current_j = 0;
    scanner.state = SCAN_MOVING;
    scanner.scan_running = true;

    // Di chuyển đến cell đầu tiên
    Point2D_t pos = Panel_GetCellCenter(scanner.panel, 0, 0);
    Axis_MoveTo2D(pos.x, pos.y, 10000.0f);
}
/* ========================================
 * START SCAN
 * ======================================== */
void PanelScanner_Start(PanelPacked_t *panel)
{
    if(scanner.scan_running)
        return;

    scanner.panel = panel;
    scanner.current_i = 0;
    scanner.current_j = 0;
    scanner.state = SCAN_MOVING;
    scanner.scan_running = true;

    Point2D_t pos = Panel_GetCellCenter(panel, 0, 0);
    Axis_MoveTo2D(pos.x, pos.y, 10000.0f);
}

/* ========================================
 * UPDATE - GỌI TRONG MAIN LOOP
 * ======================================== */
void PanelScanner_Update(void)
{
    if(!scanner.scan_running)
        return;

    switch(scanner.state)
    {
        case SCAN_IDLE:
            break;

        case SCAN_MOVING:
            if(Axis.X.state == MOTOR_IDLE && Axis.Y.state == MOTOR_IDLE)
            {
                scanner.stable_timer = HAL_GetTick();
                scanner.state = SCAN_WAIT_STABLE;
            }
            break;

        case SCAN_WAIT_STABLE:
            if(HAL_GetTick() - scanner.stable_timer >= STABLE_DELAY_MS)
            {
                scanner.read_timer = HAL_GetTick();
                scanner.state = SCAN_READING;
            }
            break;

        case SCAN_READING:
            {
                bool ok = Sensor_ReadQuality();
                if(ok)
                    panel_set_ok(scanner.panel, scanner.current_i, scanner.current_j);
                else
                    panel_set_ng(scanner.panel, scanner.current_i, scanner.current_j);
                scanner.state = SCAN_NEXT_CELL;
            }
            break;

        case SCAN_NEXT_CELL:
            scanner.current_i++;

            if(scanner.current_i >= GRID_SIZE)
            {
                scanner.current_i = 0;
                scanner.current_j++;

                if(scanner.current_j >= GRID_SIZE)
                {
                    scanner.state = SCAN_COMPLETE;
                    break;
                }
            }
            {
                Point2D_t pos = Panel_GetCellCenter(scanner.panel,
                                                    scanner.current_i,
                                                    scanner.current_j);
                Axis_MoveTo2D(pos.x, pos.y, 10000.0f);
                scanner.state = SCAN_MOVING;
            }
            break;

        case SCAN_COMPLETE:
            if(scanner.current_tray < scanner.end_tray)
            {
                scanner.state = SCAN_NEXT_TRAY;
            }
            else
            {
                scanner.scan_running = false;
                scanner.state = SCAN_IDLE;
                Home_state->bits.Scan = 0;
            }
            break;
        case SCAN_NEXT_TRAY:
			scanner.current_tray++;
		    for (int i = 1; i <= 25; i++){
		        Inputs_Database[i] = 0;
		    }
			Home_state->all &= ~(0b1111<<2);
			Home_state->all |= (1<<(scanner.current_tray+2));
			scanner.panel = &Glass[scanner.current_tray];
			scanner.current_i = 0;
			scanner.current_j = 0;
			{
				Point2D_t pos = Panel_GetCellCenter(scanner.panel, 0, 0);
				Axis_MoveTo2D(pos.x, pos.y, 10000.0f);
				scanner.state = SCAN_MOVING;
			}
			break;
        default:
            break;
    }
}

void Map_QualityBits_To_Inputs(const uint32_t *quality_bits, uint8_t count_words)
{
    uint32_t total_bits = 196;

    for (int i = 1; i <= 25; i++)
        Inputs_Database[i] = 0;

    for (uint32_t bit_index = 0; bit_index < total_bits; bit_index++)
    {
        uint32_t src_word = bit_index / 32;
        uint32_t src_bit  = bit_index % 32;

        if (src_word >= count_words)
            break;   // nếu quality_bits truyền vào ít hơn 7 phần tử

        uint8_t bit_val = (quality_bits[src_word] >> src_bit) & 1u;

        uint32_t dst_byte = 1 + (bit_index / 8);
        uint32_t dst_bit  = bit_index % 8;

        Inputs_Database[dst_byte] |= (bit_val << dst_bit);
    }
}

/* ========================================
 * HELPERS
 * ======================================== */
bool PanelScanner_IsBusy(void)
{
    return scanner.scan_running;
}

void PanelScanner_Stop(void)
{
    // Emergency stop
    Motor_Stop(AXIS_X);
    Motor_Stop(AXIS_Y);

    scanner.scan_running = false;
    scanner.state = SCAN_IDLE;
}


