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

#include "Marlin.h"
#include "MarlinConfig.h"
#include "ultralcd.h"
#include "buzzer.h"
#include "nozzle.h"
#include "temperature.h"
#include "planner.h"

#if ENABLED(PRUSA_MMU2)

#include "mmu2.h"
#include <SoftwareSerial.h> //AlfiQue

#define MMU_TODELAY 100
#define MMU_TIMEOUT 10
#define MMU_CMD_TIMEOUT 60000ul //5min timeout for mmu commands (except P0)
#define MMU_P0_TIMEOUT 3000ul //timeout for P0 command: 3seconds

#ifdef PRUSA_MMU2_MODE_12V
    #define MMU_REQUIRED_FW_BUILDNR 132
#else
    #define MMU_REQUIRED_FW_BUILDNR 126
#endif

#define MMU_BAUD 115200

SoftwareSerial mmuSerial(19,18); // RX, TX //AlfiQue


void MMU2::init() {

mmuSerial.begin(MMU_BAUD);
    #ifdef PRUSA_MMU2_HWRESET
    // TODO use macros for this
        WRITE(PRUSA_MMU2_RST_PIN, HIGH);
        SET_OUTPUT(PRUSA_MMU2_RST_PIN);
    #endif

    
delay(1000);
    SERIAL_ECHOLNPGM("MMU init");
    delay(5000);
    mmu2.toolChange(3);
    //mmuSerial.println("T3");
    //delay(5000);
    safe_delay(10);
    reset();
    rx_buffer[0] = '\0';
    state = -1;
}

void MMU2::reset() {
        #ifdef PRUSA_MMU2_DEBUG
                SERIAL_ECHOLNPGM("MMU <= reset");
        #endif

    #ifdef PRUSA_MMU2_HWRESET
        WRITE(PRUSA_MMU2_RST_PIN, LOW);
        safe_delay(20);
        WRITE(PRUSA_MMU2_RST_PIN, HIGH);
    #endif
}

void MMU2::mmuLoop() {

    int filament = 0;
    
    switch (state) {
        case 0:
            return;
        
        case -1:
            if (rx_start()) {
              SERIAL_ECHOLNPGM("MMU state -1 2");
                #ifdef PRUSA_MMU2_DEBUG
                		SERIAL_ECHOLNPGM("MMU => 'start'");
                        SERIAL_ECHOLNPGM("MMU <= 'S1'");
                #endif
    
                // send "read version" request
                tx_str_P(PSTR("S1\n"));

                state = -2;
            } else if (millis() > 3000000) {
                SERIAL_ECHOLNPGM("MMU not responding - DISABLED");
                state = 0;        
            }
            return;

        case -2:
            if (rx_ok()) {
                sscanf(rx_buffer, "%uok\n", &version);
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOLNPAIR("MMU => ", version);
                    SERIAL_ECHOLNPGM("MMU <= 'S2'");                    
                #endif
                tx_str_P(PSTR("S2\n")); // read build number
                state = -3;
            }
            return;

        case -3:
            if (rx_ok()) {
                sscanf(rx_buffer, "%uok\n", &buildnr);
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOLNPAIR("MMU => ", buildnr);

                #endif
                checkVersion();
                #ifdef PRUSA_MMU2_MODE_12V
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPGM("MMU <= 'M1'");                    
                    #endif
                    tx_str_P(PSTR("M1\n")); // switch to stealth mode
                    state = -5;
                #else
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPGM("MMU <= 'P0'");                    
                    #endif
                    tx_str_P(PSTR("P0\n")); // read finda
                    state = -4;
                #endif
            }

            return;

        case -5:
            if (rx_ok()) {
                sscanf(rx_buffer, "%uok\n", &buildnr);
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOLNPAIR("MMU => ", buildnr);

                #endif
                checkVersion();
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOLNPGM("MMU <= 'P0'");                    
                #endif
                tx_str_P(PSTR("P0\n")); // read finda
                state = -4;
            }

            return;

        case -4:
            if (rx_ok()) {
                sscanf(rx_buffer, "%hhuok\n", &finda);
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOLNPAIR("MMU => ", finda);
                    SERIAL_ECHOLNPGM("MMU - ENABLED");                    
                #endif                
                enabled = true;
                state = 1;
            }

            return;

        case 1:
        SERIAL_ECHOLNPGM("MMU tool change 6");
            if (cmd) { // command request

              SERIAL_ECHOLNPGM("MMU tool change 7");
              SERIAL_ECHOLNPAIR("MMU cmd : ",cmd);
                if ((cmd >= MMU_CMD_T0) && (cmd <= MMU_CMD_T4)) {
                    filament = cmd - MMU_CMD_T0;
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPAIR("MMU <= T", filament);
                    #endif //PRUSA_MMU2_DEBUG
                    tx_printf_P(PSTR("T%d\n"), filament);
                    state = 3; // wait for response
                }
                else if ((cmd >= MMU_CMD_L0) && (cmd <= MMU_CMD_L4))
                {
                    filament = cmd - MMU_CMD_L0;
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPAIR("MMU <= L", filament);
                    #endif //PRUSA_MMU2_DEBUG
                    tx_printf_P(PSTR("L%d\n"), filament);
                    state = 3; // wait for response
                }
                else if (cmd == MMU_CMD_C0)
                {
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPGM("MMU <= 'C0'");
                    #endif //PRUSA_MMU2_DEBUG
                    tx_str_P(PSTR("C0\n")); //send 'continue loading'
                    state = 3;
                }
                else if (cmd == MMU_CMD_U0)
                {
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPGM("MMU <= 'U0'");
                    #endif //PRUSA_MMU2_DEBUG
                    tx_str_P(PSTR("U0\n")); //send 'unload current filament'
                    state = 3;
                }
                else if ((cmd >= MMU_CMD_E0) && (cmd <= MMU_CMD_E4))
                {
                    int filament = cmd - MMU_CMD_E0;
                    #ifdef PRUSA_MMU2_DEBUG				
                        SERIAL_ECHOLNPAIR("MMU <= E", filament);
                    #endif //PRUSA_MMU2_DEBUG
                    tx_printf_P(PSTR("E%d\n"), filament); //send eject filament
                    state = 3; // wait for response
                }
                else if (cmd == MMU_CMD_R0)
                {
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLN("MMU <= 'R0'");
                    #endif //PRUSA_MMU2_DEBUG
                    tx_str_P(PSTR("R0\n")); //send recover after eject
                    state = 3; // wait for response
                }
                last_cmd = cmd;
                cmd = MMU_CMD_NONE;
            }
            else if ((last_response + 300) < millis()) //request every 300ms
            {
                #ifdef PRUSA_MMU2_DEBUG
//                    SERIAL_ECHOLNPGM("MMU <= 'P0'");
                #endif //PRUSA_MMU2_DEBUG
                tx_str_P(PSTR("P0\n")); //send 'read finda' request
                state = 2;
            }
		    return;

        case 2: //response to command P0
            if (rx_ok())
            {
                sscanf(rx_buffer, "%hhuok\n", &finda);
                #ifdef PRUSA_MMU2_DEBUG
//                    SERIAL_ECHOPAIR("MMU => ", finda);
//                    SERIAL_ECHOLNPGM("ok'");
                #endif //PRUSA_MMU2_DEBUG
//                SERIAL_ECHOLNPAIR(PSTR("Eact: %d\n"), int(e_active()));
                
                // TODO
                #if ENABLED(PRUSA_MMU2_CHECK_FINDA)
                if (!finda) {
                    // TODO
//                    fsensor_stop_and_save_print();
//                    enquecommand_front_P(PSTR("FSENSOR_RECOVER")); //then recover
 //                   if (lcd_autoDeplete) enquecommand_front_P(PSTR("M600 AUTO")); //save print and run M600 command
 //                   else enquecommand_front_P(PSTR("M600")); //save print and run M600 command
               }
               #endif
                state = 1;
                if (cmd == 0)
                    ready = true;
            }
            else if ((last_request + MMU_P0_TIMEOUT) < millis())
            { //resend request after timeout (30s)
                state = 1;
            }
            return;

        case 3: //response to mmu commands
            if (rx_ok())
            {
                #ifdef PRUSA_MMU2_DEBUG
                    SERIAL_ECHOLNPGM("MMU => 'done'");
                #endif //PRUSA_MMU2_DEBUG
                ready = true;
                state = 1;
                last_cmd = MMU_CMD_NONE;
            }
            else if ((last_request + MMU_CMD_TIMEOUT) < millis())
            { //resend request after timeout (5 min)
                if (last_cmd) {
                    #ifdef PRUSA_MMU2_DEBUG
                        SERIAL_ECHOLNPGM("MMU retry");
                    #endif //PRUSA_MMU2_DEBUG

                    cmd = last_cmd;
                    last_cmd = MMU_CMD_NONE;
                }
                state = 1;
            }
            return;                
    }
}


/**
 * Check if MMU was started
 */
bool MMU2::rx_start() {
    if (rx_str_P(PSTR("start\n"))) {
        last_response = millis();
        return true;
    }

    return false;
}


/**
 * Check if the data received ends with the given string.
*/
bool MMU2::rx_str_P(const char* str) {
    uint8_t i = strlen(rx_buffer);

    while (mmuSerial.available()) {
        SERIAL_ECHOLNPAIR("available() ", mmuSerial.available());//alfique
        rx_buffer[i++] = mmuSerial.read();
        SERIAL_ECHOLNPAIR(" | ", rx_buffer);//alfique
        rx_buffer[i] = '\0';
        #ifdef PRUSA_MMU2_DEBUG
            SERIAL_CHAR(rx_buffer[i-1]);//alfique
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


/**
 * Transfer data to MMU
 */ 
void MMU2::tx_str_P(const char* str) {
    clear_rx_buffer();

    uint8_t len = strlen_P(str);

    for (uint8_t i = 0; i < len; i++) {
        mmuSerial.write(pgm_read_byte(str++));
    }

    rx_buffer[0] = '\0';

    last_request = millis();

}


/**
 * Transfer data to MMU
 */ 
void MMU2::tx_printf_P(const char* format, int argument = -1) {
    clear_rx_buffer();
    uint8_t len = sprintf_P(tx_buffer, format, argument);

    for (uint8_t i = 0; i < len; i++) {
        mmuSerial.write(tx_buffer[i]);
    }

    rx_buffer[0] = '\0';

    last_request = millis();

}
/*
void MMU2::tx_printf_P(const char* format, ...) {
    va_list args;
    va_start(args, format);
    clear_rx_buffer();
    uint8_t len = vsprintf_P(tx_buffer, format, args);

    for (uint8_t i = 0; i < len; i++) {
        mmuSerial.write(tx_buffer[i]);
    }
    va_end(args);

    rx_buffer[0] = '\0';

    last_request = millis();

}
*/

/**
 * Empty the rx buffer
 */ 
void MMU2::clear_rx_buffer() {
    while(mmuSerial.available() > 0) {
      SERIAL_ECHOLNPGM("MMU ava2");
        mmuSerial.read();
    }

    rx_buffer[0] = '\0';
}

/**
 * Check if we received 'ok' from MMU
 */ 
bool MMU2::rx_ok() {
    if (rx_str_P(PSTR("ok\n"))) {
        last_response = millis();
        return true;
    }

    return false;
}


/**
 * Check if MMU has compatible firmware
 */
void MMU2::checkVersion() {
 	if (buildnr < MMU_REQUIRED_FW_BUILDNR) {
        SERIAL_ERROR_START();
        SERIAL_ECHOPGM("MMU2 firmware version invalid. Required version >= ");
        SERIAL_ECHOLN(MMU_REQUIRED_FW_BUILDNR);
        kill(MSG_MMU2_WRONG_FIRMWARE);
    }
}


/**
 * Handle tool change
 */
void MMU2::toolChange(uint8_t index) {

    if (index != extruder) {

        KEEPALIVE_STATE(IN_HANDLER);
        command(MMU_CMD_T0 + index);
        SERIAL_ECHOLNPGM("MMU tool change 3");
        manageResponse(true, true);
SERIAL_ECHOLNPGM("MMU tool change 3");
        KEEPALIVE_STATE(IN_HANDLER);
SERIAL_ECHOLNPGM("MMU tool change 4");
        command(MMU_CMD_C0);
        extruder = index; //filament change is finished
        active_extruder = 0;
SERIAL_ECHOLNPGM("MMU tool change 5");
        SERIAL_ECHO_START();
        SERIAL_ECHOLNPAIR(MSG_ACTIVE_EXTRUDER, int(extruder));

        KEEPALIVE_STATE(NOT_BUSY);
    }

/*
    TODO
              if (*(strchr_pointer + index) == '?')// for single material usage with mmu
              {
                  load_to_nozzle();
              }
              */
}


void MMU2::command(uint8_t mmu_cmd) {
    cmd = mmu_cmd;
    ready = false;
}

bool MMU2::getResponse(void) {

	while (cmd != MMU_CMD_NONE) {
        idle();
	}

	while (!ready) {
        idle();
		if (state != 3) {
			break;
        }
	}

	bool ret = ready;
	ready = false;

	return ret;
}


void MMU2::manageResponse(bool move_axes, bool turn_off_nozzle) {
SERIAL_ECHOLNPGM("MMU tool change 01");
	bool response = false;
	mmu_print_saved = false;
    point_t park_point = NOZZLE_PARK_POINT;
    SERIAL_ECHOLNPGM("MMU tool change 02");
    float resume_position[XYZE];
    SERIAL_ECHOLNPGM("MMU tool change 03");
	float resume_hotend_temp;
SERIAL_ECHOLNPGM("MMU tool change 04");
	while(!response) {
    SERIAL_ECHOLNPGM("MMU tool change 05");
        response = getResponse(); //wait for "ok" from mmu
            
		if (!response) { //no "ok" was received in reserved time frame, user will fix the issue on mmu unit
    SERIAL_ECHOLNPGM("MMU tool change 06");
		    if (!mmu_print_saved) { //first occurence, we are saving current position, park print head in certain position and disable nozzle heater
SERIAL_ECHOLNPGM("MMU tool change 07");
				planner.synchronize();
 
        		mmu_print_saved = true;
                SERIAL_ECHOLNPGM("MMU not responding");

                resume_hotend_temp = thermalManager.degTargetHotend(active_extruder);
                COPY(resume_position, current_position);

                if (move_axes && all_axes_homed()) {
                    Nozzle::park(2, park_point /*= NOZZLE_PARK_POINT*/);
                }

                if (turn_off_nozzle) {
                    //set nozzle target temperature to 0
                    thermalManager.setTargetHotend(0, active_extruder);
                }
			
                LCD_MESSAGEPGM(MSG_MMU2_NOT_RESPONDING);
                buzzer.tone(100, 659);//BUZZ(200, 40); //AlfiQue
                buzzer.tone(200, 698);//BUZZ(200, 40); //AlfiQue
                buzzer.tone(100, 659);//BUZZ(200, 40); //AlfiQue
                buzzer.tone(300, 440);//BUZZ(200, 40); //AlfiQue
                buzzer.tone(100, 659);//BUZZ(200, 40); //AlfiQue

                KEEPALIVE_STATE(PAUSED_FOR_USER);

            }
		}
		else if (mmu_print_saved) {
			SERIAL_ECHOLNPGM("MMU starts responding\n");
            KEEPALIVE_STATE(IN_HANDLER);

 			  if (turn_off_nozzle) {
                 if (resume_hotend_temp > 0.0) {
                    thermalManager.setTargetHotend(resume_hotend_temp, active_extruder);
                    LCD_MESSAGEPGM(MSG_HEATING);
                    buzzer.tone(200, 40);//BUZZ(200, 40); //AlfiQue


                    while (!thermalManager.degTargetHotend(active_extruder)) {
                        safe_delay(1000);
                    }
                }
			  }			  
			  if (move_axes && all_axes_homed()) {
                LCD_MESSAGEPGM(MSG_MMU2_RESUME);
                buzzer.tone(200, 40);//BUZZ(200, 40); //AlfiQue
                buzzer.tone(200, 40);//BUZZ(200, 40); //AlfiQue

                // Move XY to starting position, then Z
                do_blocking_move_to_xy(resume_position[X_AXIS], resume_position[Y_AXIS], NOZZLE_PARK_XY_FEEDRATE);

                // Move Z_AXIS to saved position
                do_blocking_move_to_z(resume_position[Z_AXIS], NOZZLE_PARK_Z_FEEDRATE);

			  }
			  else {
                buzzer.tone(200, 40);//BUZZ(200, 40); //AlfiQue
                buzzer.tone(200, 40);//BUZZ(200, 40); //AlfiQue
                LCD_MESSAGEPGM(MSG_MMU2_RESUME);
			  }
          }
    }
}

//void MMU2::load_to_nozzle() {
//
//}


MMU2 mmu2;


#endif // MMU2
