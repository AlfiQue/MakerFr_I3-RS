/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __MMU2_H__
#define __MMU2_H__

#include "../../inc/MarlinConfig.h"

#if ENABLED(PRUSA_MMU2)


#define MMU_CMD_NONE 0
#define MMU_CMD_T0   0x10
#define MMU_CMD_T1   0x11
#define MMU_CMD_T2   0x12
#define MMU_CMD_T3   0x13
#define MMU_CMD_T4   0x14
#define MMU_CMD_L0   0x20
#define MMU_CMD_L1   0x21
#define MMU_CMD_L2   0x22
#define MMU_CMD_L3   0x23
#define MMU_CMD_L4   0x24
#define MMU_CMD_C0   0x30
#define MMU_CMD_U0   0x40
#define MMU_CMD_E0   0x50
#define MMU_CMD_E1   0x51
#define MMU_CMD_E2   0x52
#define MMU_CMD_E3   0x53
#define MMU_CMD_E4   0x54
#define MMU_CMD_R0 0x60


class MMU2 {

    public:
        MMU2() {};

        void init();
        void reset();
        void mmuLoop();

    private:
        bool rx_str_P(const char* str);
        void tx_str_P(const char* str);
        void tx_printf_P(const char* format, ...);
        void clear_rx_buffer();
        bool rx_ok();

        bool rx_start();
        
        void command(uint8_t cmd);
        bool get_response(void);
        void load_to_nozzle();

        
        bool mmu_enabled = false;
        bool mmu_ready = false;
        uint8_t mmu_cmd = 0;
        int8_t mmu_state = 0;
        uint8_t mmu_extruder = 0;
        uint8_t tmp_extruder = 0;
        int8_t mmu_finda = -1;
        int16_t mmu_version = -1;
        int16_t mmu_buildnr = -1;
        uint32_t mmu_last_request = 0;
        uint32_t mmu_last_response = 0;
        char rx_buffer[16];
        char tx_buffer[16];

};

extern MMU2 mmu2;

#endif

#endif // __MMU2_H__
