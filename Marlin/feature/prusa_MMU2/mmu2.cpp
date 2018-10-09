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

#include "../../inc/MarlinConfig.h"

#if ENABLED(PRUSA_MMU2)

#include "mmu2.h"


#define MMU_TODELAY 100
#define MMU_TIMEOUT 10
#define MMU_CMD_TIMEOUT 300000ul //5min timeout for mmu commands (except P0)
#define MMU_P0_TIMEOUT 3000ul //timeout for P0 command: 3seconds

#define MMU_REQUIRED_FW_BUILDNR 83
#define MMU_BAUD    115200


//HardwareSerial marlinUART = Serial2;
#define marlinUART   Serial2


void MMU2::init() {

    #ifdef PRUSA_MMU2_HWRESET
    // TODO use macros for this
        WRITE(PRUSA_MMU2_RST_PIN, HIGH);
        SET_OUTPUT(PRUSA_MMU2_RST_PIN);
    #endif

    marlinUART.begin(MMU_BAUD);

    safe_delay(10);
    reset();
    rx_buffer[0] = '\0';
    mmu_state = -1;
}

void MMU2::reset() {

}

void MMU2::mmuLoop() {

    int filament = 0;
    
    switch (mmu_state) {
        case 0:
            return;
        
        case -1:
            if (rx_start()) {
                #ifdef PRUSA_MMU2_DEBUG
                		SERIAL_ECHOLNPGM("MMU => 'start'");
                        SERIAL_ECHOLNPGM("MMU <= 'S1'");
                #endif
    
                // send "read version" request
                tx_str_P(PSTR("S1\n"));

                mmu_state = -2;
            } else if (millis() > 30000) {
                SERIAL_ECHOLNPGM("MMU not responding - DISABLED");
                mmu_state = 0;        
            }
            return;

        case -2:
            if (rx_ok()) {
                SERIAL_ECHOLN(rx_buffer);                    
                sscanf(rx_buffer, "%uok\n", &mmu_version);
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOPAIR("MMU => ", mmu_version);
                    SERIAL_ECHOLNPGM("ok'");                    
                    SERIAL_ECHOLNPGM("MMU <= 'S2'");                    
                #endif
                tx_str_P(PSTR("S2\n")); // read build number
                mmu_state = -3;
            }
            return;

        case -3:
            if (rx_ok()) {
                sscanf(rx_buffer, "%uok\n", &mmu_buildnr);
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOPAIR("MMU => ", mmu_buildnr);
                    SERIAL_ECHOLNPGM("ok'");                    
                    SERIAL_ECHOLNPGM("MMU <= 'P0'");                    
                #endif
                tx_str_P(PSTR("P0\n")); // read finda
                mmu_state = -4;
            }

            return;

        case -4:
            if (rx_ok()) {
                sscanf(rx_buffer, "%hhuok\n", &mmu_finda);
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOPAIR("MMU => ", mmu_finda);
                    SERIAL_ECHOLNPGM("ok'");                    
                    SERIAL_ECHOLNPGM("MMU - ENABLED");                    
                #endif                
                mmu_enabled = true;
                mmu_state = 1;
            }

            return;

        case 1:
            if (mmu_cmd) { // command request
                if ((mmu_cmd >= MMU_CMD_T0) && (mmu_cmd <= MMU_CMD_T4)) {
                    filament = mmu_cmd - MMU_CMD_T0;
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPAIR("MMU <= T", filament);
                    #endif //PRUSA_MMU2_DEBUG
                    tx_printf_P(PSTR("T%d\n"), filament);
                    mmu_state = 3; // wait for response
                }
                else if ((mmu_cmd >= MMU_CMD_L0) && (mmu_cmd <= MMU_CMD_L4))
                {
                    filament = mmu_cmd - MMU_CMD_L0;
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPAIR("MMU <= L", filament);
                    #endif //PRUSA_MMU2_DEBUG
                    tx_printf_P(PSTR("L%d\n"), filament);
                    mmu_state = 3; // wait for response
                }
                else if (mmu_cmd == MMU_CMD_C0)
                {
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPGM("MMU <= 'C0'");
                    #endif //PRUSA_MMU2_DEBUG
                    tx_str_P(PSTR("C0\n")); //send 'continue loading'
                    mmu_state = 3;
                }
                else if (mmu_cmd == MMU_CMD_U0)
                {
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPGM("MMU <= 'U0'");
                    #endif //PRUSA_MMU2_DEBUG
                    tx_str_P(PSTR("U0\n")); //send 'unload current filament'
                    mmu_state = 3;
                }
                else if ((mmu_cmd >= MMU_CMD_E0) && (mmu_cmd <= MMU_CMD_E4))
                {
                    int filament = mmu_cmd - MMU_CMD_E0;
                    #ifdef PRUSA_MMU2_DEBUG				
                        SERIAL_ECHOLNPAIR("MMU <= E", filament);
                    #endif //PRUSA_MMU2_DEBUG
                    tx_printf_P(PSTR("E%d\n"), filament); //send eject filament
                    mmu_state = 3; // wait for response
                }
                else if (mmu_cmd == MMU_CMD_R0)
                {
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLN("MMU <= 'R0'");
                    #endif //PRUSA_MMU2_DEBUG
                    tx_str_P(PSTR("R0\n")); //send recover after eject
                    mmu_state = 3; // wait for response
                }
                mmu_cmd = 0;
            }
            else if ((mmu_last_response + 300) < millis()) //request every 300ms
            {
                #ifdef PRUSA_MMU2_DEBUG
//                    SERIAL_ECHOLNPGM("MMU <= 'P0'");
                #endif //PRUSA_MMU2_DEBUG
                tx_str_P(PSTR("P0\n")); //send 'read finda' request
                mmu_state = 2;
            }
		    return;

        case 2: //response to command P0
            if (rx_ok())
            {
                sscanf(rx_buffer, "%hhuok\n", &mmu_finda);
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOPAIR("MMU => ", mmu_finda);
                    SERIAL_ECHOLNPGM("ok'");
                #endif //PRUSA_MMU2_DEBUG
//                SERIAL_ECHOLNPAIR(PSTR("Eact: %d\n"), int(e_active()));
                
                // TODO
                #if ENABLED(PRUSA_MMU2_CHECK_FINDA)
                if (!mmu_finda) {
//                    fsensor_stop_and_save_print();
//                    enquecommand_front_P(PSTR("FSENSOR_RECOVER")); //then recover
 //                   if (lcd_autoDeplete) enquecommand_front_P(PSTR("M600 AUTO")); //save print and run M600 command
 //                   else enquecommand_front_P(PSTR("M600")); //save print and run M600 command
               }
               #endif
                mmu_state = 1;
                if (mmu_cmd == 0)
                    mmu_ready = true;
            }
            else if ((mmu_last_request + MMU_P0_TIMEOUT) < millis())
            { //resend request after timeout (30s)
                mmu_state = 1;
            }
            return;

        case 3: //response to mmu commands
            if (rx_ok())
            {
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOLNPGM("MMU => 'ok'");
                #endif //PRUSA_MMU2_DEBUG
                mmu_ready = true;
                mmu_state = 1;
            }
            else if ((mmu_last_request + MMU_CMD_TIMEOUT) < millis())
            { //resend request after timeout (5 min)
                mmu_state = 1;
            }
            return;                
    }
}


bool MMU2::rx_start() {
    if (rx_str_P(PSTR("start\n"))) {
        mmu_last_response = millis();
        return true;
    }

    return false;
}


/*
    check if the data received ends with the given string.
    */
bool MMU2::rx_str_P(const char* str) {
    uint8_t i = strlen(rx_buffer);

    while (marlinUART.available()) {
//        SERIAL_ECHOLNPAIR("available() ", marlinUART.available());
        rx_buffer[i++] = marlinUART.read();
        rx_buffer[i] = '\0';
        #ifdef PRUSA_MMU2_DEBUG
            SERIAL_CHAR(rx_buffer[i-1]);
        #endif

        if (i == sizeof(rx_buffer)) {
            #ifdef PRUSA_MMU2_DEBUG
                SERIAL_ECHOLNPGM("rx buffer overrun");
            #endif
            break;
        }
    }

    uint8_t len = strlen_P(str);

    if (i < len) {
        return false;
    }

    str += len;

    while(len--) {
        char c0 = pgm_read_byte(str--);
        char c1 = rx_buffer[i--];
        #ifdef PRUSA_MMU2_DEBUG
//            SERIAL_CHAR(c0);
//            SERIAL_CHAR(c1);
//            SERIAL_ECHOPGM("\n");
        #endif
        if (c0 == c1) {
            continue;
        }
		if ((c0 == '\r') && (c1 == '\n')) {      //match cr as lf
			continue;
        }
		if ((c0 == '\n') && (c1 == '\r')) {     //match lf as cr
			continue;
        }
        
        return false;
    }

    return true;
}


void MMU2::tx_str_P(const char* str) {
    clear_rx_buffer();

    uint8_t len = strlen_P(str);

    for (uint8_t i = 0; i < len; i++) {
        marlinUART.write(pgm_read_byte(str++));
    }

    rx_buffer[0] = '\0';
}

void MMU2::tx_printf_P(const char* format, ...) {
    va_list args;
    va_start(args, format);
    clear_rx_buffer();
    vsprintf(tx_buffer, format, args);
    uint8_t len = strlen(tx_buffer);

    for (uint8_t i = 0; i < len; i++) {
        marlinUART.write(tx_buffer[i]);
    }
    va_end(args);

    rx_buffer[0] = '\0';

}


void MMU2::clear_rx_buffer() {
    while(marlinUART.available() > 0) {
        marlinUART.read();
    }

    rx_buffer[0] = '\0';
}

bool MMU2::rx_ok() {
    if (rx_str_P(PSTR("ok\n"))) {
        mmu_last_response = millis();
        return true;
    }

    return false;
}

void MMU2::command(uint8_t cmd) {

}

bool MMU2::get_response(void) {

}

void MMU2::load_to_nozzle() {

}


MMU2 mmu2;


#endif // MMU2
