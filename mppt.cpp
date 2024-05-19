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
  mppt.cpp

  This file contains the user-defined MPPT logic for initialization and periodic
  updating of the controller.

*/



#include "mppt.h"



// global variables (if any)
int led_state = LOW;

const int RXLED = 17;   // the RX LED can be accessed as a Arduino pin
                        // Note: does not apply for the TXLED,
                        // that needs TXLED1 and TXLED0 macros
uint16_t thr_us;





/*
  mmpt_init()

  User-defined initialization function for MPPT controller.
  This is called once after reset.

  Arguments:
    None

  Returns:
    None

*/
void mppt_init(void)
{

  pinMode(RXLED, OUTPUT);   // Set RX LED as an output
                            // TX LED is set as an output behind the scenes
  digitalWrite(RXLED, LOW);

  thr_us = THRMIN_US;

  // perform any other algorithm initialization here...

}






/*
  mmpt_update()

  User-defined update function for the MPPT controller.
  After a user-defined start-up delay (see mppt.h) this is run periodically
  at a user-defined rate (see mppt.h)

  Arguments:
    in0_us    pulse width of RC input #0 in [us]
    in1_us    pulse width of RC input #1 in [us]
    v_sa_mv   S/A voltage in [mV]

  Returns:
    thr_us    ESC throttle command in [us]

*/
uint16_t mppt_update(uint16_t in0_us, uint16_t in1_us, uint16_t v_sa_mv)
{

  if (in0_us < 1300)             // check if RX signal is off, with margin
  {

    thr_us = 1100;

  }
  else
  {

	  if (v_sa_mv < V_MPPT_MV)
    {
		  if(thr_us < 1900)
      {
        thr_us = thr_us + THR_INC_US;  // increase throttle until max.
      }
    }
	  else
    {
		  if(thr_us > 1500)
      {
        thr_us = thr_us - THR_INC_US;  // decrease throttle down to min.
      }
    }

  }


  /*
    WARNING w.r.t. using 'Serial.print()' or 'Serial.println()'

    Any use of 'Serial.print()' or 'Serial.println()' e.g. to print out debug
    information must be removed from the code when the MPPT is used in flight.

    If no USB cable is plugged in and no USB connection is open, 'Serial.print()'
    and 'Serial.println()' may be blocking, i.e. they will freeze the program.

    I.a.w. the Arduino documentation this is supposed to be circumvented by checking
    with 'if(Serial)' and only use 'Serial' if this returns true. For some unknown
    reasons this check however always returns true on the Arduino Pro Micro.

    The hardware 'Serial1' may be used at any time as it doesn't need any form of
    connection, it however blocks program execution for as long as it takes to
    send the data (i.e. program timings may no longer be correct)
  */


  // blink RXLED as a "keep-alive" signal:
  if(led_state == LOW)
    led_state = HIGH;
  else
    led_state = LOW;
  digitalWrite(RXLED, led_state);

  return( constrain(thr_us, THRMIN_US, THRMAX_US) );

}
