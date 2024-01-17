////////////////////////////////////////////////////////////////////////////////


#include "mppt.h"


////////////////////////////////////////////////////////////////////////////////


// global vaiables (if any)


////////////////////////////////////////////////////////////////////////////////


void mppt_init(void)
{

  ;   // initalization code (if any)

}


////////////////////////////////////////////////////////////////////////////////


uint16_t mppt_update(uint16_t in0, uint16_t in1, uint16_t v_sa)
{ 
  uint16_t thr;

  /*
  if (in0 < 1300)             // check if RX signal is off, with margin
  {

    thr = 1100;

  }
  else
  {

	  if (v_sa < V_MPPT)
    {
		  if(thr < 1900)
      {
        thr = thr + THR_INC;  // increase throttle until max.
      }
    }
	  else
    {
		  if(thr > 1500)
      {
        thr = thr - THR_INC;  // decrease throttle down to min.
      }
    }

  }
  */

  thr = constrain(in0, THRMIN, THRMAX);
	
  return(thr);
}


////////////////////////////////////////////////////////////////////////////////
