// Timer Service Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Timer 4

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "timer.h"

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initTimer()
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4;
    _delay_cycles(3);
	
    // Configure Timer 4 for 1 sec tick
    TIMER4_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER4_TAILR_R = 977;                           // set load value (50 kHz? rate)
    TIMER4_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-on timer
    TIMER4_IMR_R |= TIMER_IMR_TATOIM;                // turn-on interrupt
    NVIC_EN2_R |= 1 << (INT_TIMER4A-80);             // turn-on interrupt 86 (TIMER4A)
}

// Placeholder random number function
uint32_t random32()
{
    return TIMER4_TAV_R;
}

