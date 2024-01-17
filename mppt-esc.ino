////////////////////////////////////////////////////////////////////////////////
//  mppt
//
//  Author: Felix Althaus
//  
//
//
//
//  ref GPLv3

////////////////////////////////////////////////////////////////////////////////

// Peripherial Allocation (ATmega32U4):
//
// Timer0   Arduino millis()
//          see https://www.arduino.cc/reference/en/language/functions/time/millis/
// Timer1   Servo output pulse generation
// Timer3   Servo input pulse measurement
//
//
//
// IO Allocation (ATmega32U4)
//
//
//

////////////////////////////////////////////////////////////////////////////////


#include <util/atomic.h>
#include "mppt.h"


////////////////////////////////////////////////////////////////////////////////


const long MPPT_INTERVAL_MS = ((int)(1000 / F_CTRL));
const long ADC_INTERVAL_MS  = 5;

const uint16_t SERVO_PERIOD_US = 20000;

const uint16_t AVREF = 3300;
const uint16_t VDIV = 3;


////////////////////////////////////////////////////////////////////////////////


int RXLED = 17;   // the RX LED can be accessed as a Arduino pin
                  // Note: does not apply for the TXLED, that needs TXLED1 and TXLED0 macros

int SA_AIN = A3;  // ADC input for S/A voltage


unsigned long current_millis = 0;
unsigned long mppt_previous_millis = 0;
unsigned long adc_previous_millis = 0;

int led_state = LOW;


// All variables shared with interrupt service routines (ISRs) need to be designated
// as 'volatile' as otherwise the compiler may optimize away some operations on them
//
volatile uint16_t rc0_re;
volatile uint16_t rc0_fe;
volatile bool rc0_ready;
//
volatile uint16_t rc1_re;
volatile uint16_t rc1_fe;
volatile bool rc1_ready;

uint16_t rcin0;
uint16_t rcin1;

uint16_t thr_mppt;



////////////////////////////////////////////////////////////////////////////////


void setup()
{

  // GPIO setup
  pinMode(RXLED, OUTPUT); // Set RX LED as an output
                          // TX LED is set as an output behind the scenes
  digitalWrite(RXLED, LOW);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);


  // Serial setup
  Serial.begin(57600);    // USB serial
  Serial1.begin(57600);   // UART


  // External interrupts (INTx) are used for servo input measurement
  //
  EIFR  = (1<<INTF1) | (1<<INTF0);  // clear interrupt flags as per datasheet recomendation
  EICRA = (1<<ISC10) | (1<<ISC00);  // any change triggers an interrupt
  EIMSK = (1<<INT0) | (1<<INT1);    // enable INT0 and INT1 interrupts
  rc0_ready = false;
  rc1_ready = false;
  rcin0 = RCMIN;
  rcin1 = RCMIN;


  // Timer3 is used for servo input measurement (together with external interrupts)
  //
  TCCR3A = 0x00;       // free-running timer
  TCCR3B = (1<<CS30);  // 8 MHz / 1 = 8 MHZ input clock
  TCCR3C = 0x00;


  // Timer1 for servo signal output generation
  // 16bit timer -> max period = 65536 ticks
  //                           = 65.536 ms @ 1 MHz input clock
  //                           =  8.182 ms @ 8 MHZ input clock
  //
  // WGM (Waveform Generation Mode) #14: "Fast PWM" with range 0..ICR1
  //
  TCCR1A = (1<<COM1B1) | (1<<WGM11);                // set on compare for OC1B, WGM #14
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);     // WGM #14, 8 MHz / 8 = 1 MHz input clock
  ICR1 = SERVO_PERIOD_US;                           // PWM period, i.e. servo signal period
  //
  thr_mppt = THRMIN;
  OCR1B = OCR1B = constrain(thr_mppt, THRMIN, THRMAX);


  // analog setup
  analogReference(DEFAULT);   // use 3.3V reference voltage for ADC
  // TODO do one loop of adc acquisitions


  // MPPT application setup
  mppt_init();

}


////////////////////////////////////////////////////////////////////////////////


void loop()
{

  uint16_t v_sa_adc;
  uint16_t v_sa;


  // Process servo input signal #0 (if available)
  if(rc0_ready)
  {
    // interrupts need to be switched off temporarily while accessing variables shared with ISRs
    // ATMOIC_BLOCK() takes care of this, and then re-enables interrupts ("FORCEON")
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
      rcin0 = (rc0_fe - rc0_re);            // only do the minimum while interrupts are disabled
      rc0_ready = false;
    }
    rcin0 = (rcin0 + 4) >> 3;               // complete conversion to [us] (incl. rounding)
    rcin0 = constrain(rcin0, RCMIN, RCMAX); // constrain to valid signal range

    Serial.print("0\t");
    Serial.println(rcin0);    
  }


  // Process servo input signal #1 (if available)
  if(rc1_ready)
  {
    // interrupts need to be switched off temporarily while accessing variables shared with ISRs
    // ATMOIC_BLOCK() takes care of this, and then re-enables interrupts ("FORCEON")
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {    
      rcin1 = (rc1_fe - rc1_re);            // only do the minimum while interrupts are disabled
      rc1_ready = false;
    }
    rcin1 = (rcin1 + 4) >> 3;               // complete conversion to [us] (incl. rounding)
    rcin1 = constrain(rcin1, RCMIN, RCMAX); // constrain to valid signal range

    Serial.print("1\t");
    Serial.println(rcin1);    
  }

  
  current_millis = millis();


  if (current_millis - adc_previous_millis >= ADC_INTERVAL_MS)
  {
    adc_previous_millis = current_millis;

    // handle ADC measurement
    // constat measurement in the background, rolling average over last few measurements
    //
    // TODO
    v_sa_adc = analogRead(SA_AIN);

    // TODO: add filtering

  }



  if(current_millis - mppt_previous_millis >= MPPT_INTERVAL_MS)
  {
    mppt_previous_millis = current_millis;

    // TODO: add timeouts


    // TODO: add init period where MPPT is not active



    v_sa = (uint16_t)(((uint32_t)v_sa_adc*(uint32_t)(AVREF)*(uint32_t)(VDIV) + (512UL)) >> 10);
    Serial1.println(v_sa_adc, v_sa);

    // do control stuff here
    thr_mppt = mppt_update(rcin0, rcin1, v_sa);
    
    OCR1B = constrain(thr_mppt, THRMIN, THRMAX);


    // blink RXLED
    if(led_state == LOW)
      led_state = HIGH;
    else
      led_state = LOW;
    digitalWrite(RXLED, led_state);
    
    
  }
  
}


////////////////////////////////////////////////////////////////////////////////


/* from the ATMega32U4 datasheet:

  [...] All interrupts have a separate Interrupt Vector in the Interrupt Vector table. 
  The interrupts have priority in accordance with their Interrupt Vector position. 
  The lower the Interrupt Vector address, the higher the priority.


This is what we see between INT0 and INT1, when RX RC pulses start at the same time
(which is quite common these days): INT1 always gets served later, and thus the measured 
pulse length is shorter (i.e. measurement is biased).

To void this, we collect all interrupts in a single ISR, where we save the timer first
then check the interrupt flags one after each other

may be done with ISR_ALIAS() or ISR_ALIASOF()

*/


////////////////////////////////////////////////////////////////////////////////


// INT0 @ PD0
ISR(INT0_vect)
{
  uint16_t tmp;

  tmp = TCNT3;                  // save timer value as soon as possible in the ISR  
  
  if(PIND & (1<<PIN0))        // we detected a rising edge of the signal (start of impulse)
  {
    rc0_re = tmp;
    rc0_ready = false;        // invalidate output
  }
  else                        // we detected a falling edge of the signal (end of impulse)
  {
    rc0_fe = tmp;
    rc0_ready = true;         // notify main loop the new data is ready
  }
 

}


////////////////////////////////////////////////////////////////////////////////


// INT1 @ PD1
ISR(INT1_vect)
{
  uint16_t tmp;

  tmp = TCNT3;                // save timer value as soon as possible in the ISR  

  if(PIND & (1<<PIN1))        // we detected a rising edge of the signal (start of impulse)
  {
    rc1_re = tmp;
    rc1_ready = false;        // invalidate output
  }
  else                        // we detected a falling edge of the signal (end of impulse)
  {
    rc1_fe = tmp;
    rc1_ready = true;         // notify main loop the new data is ready
  }
  
}


////////////////////////////////////////////////////////////////////////////////
