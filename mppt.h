////////////////////////////////////////////////////////////////////////////////
// TODO: add guards



////////////////////////////////////////////////////////////////////////////////


#include "Arduino.h"


////////////////////////////////////////////////////////////////////////////////


const uint16_t RCMIN = 1000;      // [us] minimum valid input pulse length
const uint16_t RCMAX = 2000;      // [us] maximum valid input pulse length
const uint16_t THRMIN = 1000;     // [us] minimum valid throttle pulse length
const uint16_t THRMAX = 2000;     // [us] maximum valid throttle pulse length
const uint16_t INIT_WAIT = 5000;  // [ms] time to wait for ESC initialization
const uint16_t V_MPPT = 7000;     // [mV] target MPPT S/A voltage
const uint16_t THR_INC = 16;      // [us] incremntal change t throttle signal
const uint16_t F_CTRL = 10;       // [Hz] control frequency

// Note: F_CTRL and THR_INC are chosen to go from off to full throttle within 1s


////////////////////////////////////////////////////////////////////////////////

void mppt_init(void);

uint16_t mppt_update(uint16_t in0, uint16_t in1, uint16_t v_sa);


////////////////////////////////////////////////////////////////////////////////
