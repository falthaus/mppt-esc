/*

    mppt-esc: MPPT for a solar-powered model glider, acting on the throttle
    Copyright (C) 2024 Felix Althaus

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

/*
  mppt-esc.ino

  This file contains all the hardware-related and background functionality, such as
  RC signal acquisition, SA voltage measurement, output pulse generation, etc.

  It also periodically calls the mppt_update() user function (after a start-up delay).
*/

/*
  mppt.h

  This file contains the configuration parameters for the MPPTr controller.

*/



#include "Arduino.h"




/*
  MPPT configuration parameters
*/
const uint16_t RCMIN_US = 1000;      // [us] minimum valid input pulse length
const uint16_t RCMAX_US = 2000;      // [us] maximum valid input pulse length
const uint16_t THRMIN_US = 1000;     // [us] minimum valid throttle pulse length
const uint16_t THRMAX_US = 2000;     // [us] maximum valid throttle pulse length
const uint16_t INIT_WAIT_MS = 5000;  // [ms] time to wait for ESC initialization
const uint16_t V_MPPT_MV = 7000;     // [mV] target MPPT S/A voltage
const uint16_t THR_INC_US = 16;      // [us] incremental change to throttle signal
const uint16_t F_CTRL_HZ = 10;       // [Hz] MPPT update rate

// Note: F_CTRL and THR_INC are chosen to go from off to full throttle within 1s





/*
  MPPT function declarations
*/
void mppt_init(void);
uint16_t mppt_update(uint16_t in0_us, uint16_t in1_us, uint16_t v_sa_mv);
