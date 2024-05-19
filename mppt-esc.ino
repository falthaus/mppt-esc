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
  Peripheral Allocation (ATmega32U4):

  Timer0   Arduino millis()
           see https://www.arduino.cc/reference/en/language/functions/time/millis/
  Timer1   Servo output pulse generation
  Timer3   Servo input pulse measurement
  ADC      S/A voltage acquisition
  USART1   Debug/Telemetry (optional)

  IO Allocation (ATmega32U4):
  ATmega32U4 I/O      Arduino I/O       Function

  PD0 / INT0          D2                RC pulse input #0
  PD1 / INT1          D3                RC pulse input #1
  PD3 / TX1           D1                Debug UART (optional)
  PB6 / OC1B          D10               ESC RC pulse output
  PF4 / ADC4          A3                S/A voltage input

*/



#include <util/atomic.h>
#include "mppt.h"



// Exponential Moving Average (EMA) Filter for S/A voltage smoothing
//
// EMA fractional resolution
const uint32_t EMA_Q = 8;
// EMA Smoothing Factor
// Step response is approx tau = -5 / log(1-ALPHA) filter steps
// Alpha is selected to keep the setup response below 20 cycles to allow
// the filter to settle to the new value before the MPPT is updated
const uint32_t EMA_ALPHA = (uint32_t)(0.30*(1UL<<EMA_Q));

// MPPT update rate
const int32_t MPPT_INTERVAL_US = ((1000000UL / F_CTRL_HZ));

// ADC sampling rate must be min 20 x MPPT interval to allow the EMA filter to
// settle to the new value before the MPPT is updated
const int32_t ADC_INTERVAL_US  = MPPT_INTERVAL_US / 20;

// Hardware-related Parameters
const uint16_t SERVO_PERIOD_US = 20000UL;   // RC servo pulse period
const uint32_t AVREF_MV = 3300UL;           // ADC reference voltage (internal 3.3v)
const uint32_t ADC_NBITS = 10UL;            // ADC resolution
const uint32_t VDIV = 3;                    // S/A voltage is externally divided
const int SA_AIN = A3;                      // ADC input for S/A voltage



int32_t current_micros = 0;
int32_t mppt_previous_micros = 0;
int32_t adc_previous_micros = 0;

enum mode_t {MODE_INIT, MODE_RUN};
enum mode_t mode = MODE_INIT;

// All variables shared with interrupt service routines (ISRs) must be declared
// as 'volatile' as otherwise the compiler may optimize away some (or all) operations on them
volatile uint16_t rc0_re;
volatile uint16_t rc0_fe;
volatile uint16_t rc1_re;
volatile uint16_t rc1_fe;

uint16_t rcin0;
uint16_t rcin1;
uint16_t thr_mppt;





void setup()
{

  // Serial setup
  Serial.begin(57600);    // USB serial
  Serial1.begin(57600);   // UART serial port


  // External interrupts (INTx) are used for servo input measurement
  //
  EIFR  = (1<<INTF1) | (1<<INTF0);  // clear interrupt flags as per datasheet recommendation
  EICRA = (1<<ISC10) | (1<<ISC00);  // any change triggers an interrupt
  GPIOR0 = 0x00;                    // register for flags to signal between ISRs and main loop
  EIMSK = (1<<INT0) | (1<<INT1);    // enable INT0 and INT1 interrupts
  rcin0 = RCMIN_US;
  rcin1 = RCMIN_US;

  // Timer3 is used for servo input measurement (together with external interrupts)
  //
  TCCR3A = 0x00;       // free-running timer
  TCCR3B = (1<<CS30);  // 8 MHz / 1 = 8 MHZ input clock
  TCCR3C = 0x00;

  // Timer1 is used for servo signal output generation
  // 16bit timer -> max period = 65536 ticks
  //                           = 65.536 ms @ 1 MHz input clock
  //                           =  8.182 ms @ 8 MHZ input clock
  //
  // WGM (Waveform Generation Mode) #14: "Fast PWM" with range 0..ICR1
  //
  DDRB |= (1<<PIN6);                                // OC1B is at Pin PB6, and must be an output
  TCCR1A = (1<<COM1B1) | (1<<WGM11);                // set on compare for OC1B, WGM #14
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);     // WGM #14, 8 MHz / 8 = 1 MHz input clock
  ICR1 = SERVO_PERIOD_US;                           // PWM period, i.e. servo signal period
  //
  thr_mppt = THRMIN_US;

  OCR1B = constrain(thr_mppt, THRMIN_US, THRMAX_US);

  // ADC setup
  analogReference(DEFAULT);   // use 3.3V reference voltage for ADC

  mppt_init();    // init MPPT

}






void loop()
{

  static uint32_t v_adc;
  static uint32_t v_adc_filtered;
  static uint16_t v_sa_mv;


  // Process servo input signal #0 (if available)
  if(GPIOR0 & (1<<0))
  {
    // interrupts need to be switched off temporarily while accessing variables shared with ISRs
    // ATMOIC_BLOCK() takes care of this, and then re-enables interrupts ("FORCEON")
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
      rcin0 = (rc0_fe - rc0_re);            // only do the minimum while interrupts are disabled
      GPIOR0 &= ~(1<<0);
    }
    rcin0 = (rcin0 + 4) >> 3;               // complete conversion to [us] (incl. rounding)
    rcin0 = constrain(rcin0, RCMIN_US, RCMAX_US); // constrain to valid signal range
  }


  // Process servo input signal #1 (if available)
  if(GPIOR0 & (1<<1))
  {
    // interrupts need to be switched off temporarily while accessing variables shared with ISRs
    // ATMOIC_BLOCK() takes care of this, and then re-enables interrupts ("FORCEON")
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
      rcin1 = (rc1_fe - rc1_re);            // only do the minimum while interrupts are disabled
      GPIOR0 &= ~(1<<1);
    }
    rcin1 = (rcin1 + 4) >> 3;               // complete conversion to [us] (incl. rounding)
    rcin1 = constrain(rcin1, RCMIN_US, RCMAX_US); // constrain to valid signal range
  }


  current_micros = micros();    // get current elapsed time in [us]


  // acquire ADC every ADC_INTERVAL_US:
  if (current_micros - adc_previous_micros >= ADC_INTERVAL_US)
  {
    adc_previous_micros = current_micros;

    // Take ADC measurement
    v_adc = (uint32_t)analogRead(SA_AIN);

    // Apply exponential moving average filtering filtering (EMA)
    // i.e. y[i] = alpha*x[i] + (1-alpha)*y[i-1]
    // All calculations are done in integer math. This is much, much faster than float,
    // and it can use the Atmega32U4's integrated hardware multiplier
    // Rescaling is done with bit-shifts. To avoid truncation an offset is added,
    // to achieve proper nearest-integer rounding
    // The final result then needs to be rescaled again before using it further
    v_adc_filtered = EMA_ALPHA*(v_adc<<EMA_Q) + ((1UL<<EMA_Q)-EMA_ALPHA)*(v_adc_filtered);
    v_adc_filtered = (v_adc_filtered + (1UL<<(EMA_Q-1))) >> EMA_Q;
  }


  // Initialization: skip running MPPT algorithm for INIT_WAIT ms
  if(mode == MODE_INIT)
  {
    if(current_micros > (int32_t)(INIT_WAIT_MS*1000L))
    {
      // switch to RUN mode, i.e. executing MPPT algorithm regularly
      mode = MODE_RUN;
      // at this time, current_micros is also large enough w.r.t. mppt_previous_micros
      // such that mppt algorithm is run directly after INIT_WAIT_MS time
    }
    else
    {
      // stay in INIT mode
      mode = MODE_INIT;
    }

  }

  // run MPPT algorithm every MPPT_INTERVAL_US
  else if(mode == MODE_RUN)
  {

    if(current_micros - mppt_previous_micros >= MPPT_INTERVAL_US)
    {
      mppt_previous_micros = current_micros;

      // convert ADC output to S/A voltage in millivolt (incl. rounding):
      v_sa_mv = (uint16_t)((((v_adc_filtered + (1UL<<(EMA_Q-1))) >> EMA_Q)*AVREF_MV*VDIV
                + (1UL<<(ADC_NBITS-1))) >> ADC_NBITS);

      // run MPPT cycle:
      thr_mppt = mppt_update(rcin0, rcin1, v_sa_mv);

      // limit mppt output and update throttle signal output:
      OCR1B = constrain(thr_mppt, THRMIN_US, THRMAX_US);

    }

    mode = MODE_RUN;    // keep in RUN mode

  }

  else
  {
    // invalid case, restore to RUN state to keep operating
    mode = MODE_RUN;
  }

}





// INT0 @ PD0
ISR(INT0_vect)
{
  uint16_t tmp;

  tmp = TCNT3;                // save timer value as soon as possible in the ISR

  if(PIND & (1<<PIN0))        // we detected a rising edge of the signal (start of impulse)
  {
    rc0_re = tmp;
    GPIOR0 &= ~(1<<0);        // invalidate output
  }
  else                        // we detected a falling edge of the signal (end of impulse)
  {
    rc0_fe = tmp;
    GPIOR0 |=  (1<<0);        // notify main loop the new data is ready
  }

}



// INT1 @ PD1
ISR(INT1_vect)
{
  uint16_t tmp;

  tmp = TCNT3;                // save timer value as soon as possible in the ISR

  if(PIND & (1<<PIN1))        // we detected a rising edge of the signal (start of impulse)
  {
    rc1_re = tmp;
    GPIOR0 &= ~(1<<1);        // invalidate output
  }
  else                        // we detected a falling edge of the signal (end of impulse)
  {
    rc1_fe = tmp;
    GPIOR0 |=  (1<<1);        // notify main loop the new data is ready
  }

}


/*
Note w.r.t. RC pulse input interrupt handlers

From the ATMega32U4 datasheet:
  [...] All interrupts have a separate Interrupt Vector in the Interrupt Vector table.
  The interrupts have priority in accordance with their Interrupt Vector position.
  The lower the Interrupt Vector address, the higher the priority.

This is what is in fact seen between INT0 and INT1, when RX RC pulses start at the same time
(which is quite common with receivers these days): INT1 always gets served later, and thus the
measured pulse length is shorter (i.e. measurement is biased).

To void this, both ISRs could be combined into a single one, where the time is saved first and
then logic is implemented to determine which input and edge occurred.
This may be done using ISR_ALIAS() or ISR_ALIASOF(). For some reason this didn't work however
in this case. The Arduino environment may interfere here.
As the performance of the existing implementation with separate ISRs already provides sufficient performance and reliability this issue wasn't debugged any further.

*/
