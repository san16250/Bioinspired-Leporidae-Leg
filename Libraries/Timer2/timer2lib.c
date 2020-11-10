/** @file timer2lib.c
 *
 * @brief Control the timer 2.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */
 
#include <stdint.h> 
#include <stdbool.h>

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "Libraries/Timer2/timer2lib.h"


static bool timer2_status = false;
 
/*!  
* @brief Configure the timer 2 module.
*   
* @param[in] frequency The times in a second that the interrupt ocurrs.  
*  
* @return Void. 
*/
void
timer2configure (uint32_t frequency)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/frequency);
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER2_BASE, TIMER_A);
}

/*!  
* @brief Function that returns if the timer has accomplished its count.
*   
* @param[in] Void.
*  
* @return The timer status.
*/
bool
is_timer2_done (void)
{
    return timer2_status;
}

/*!  
* @brief Function to reset the timer flag.
*   
* @param[in] Void.
*  
* @return Void.
*/
void
reset_timer2 (void)
{
    timer2_status = false;
}

/*!  
* @brief Interrupt handler for the timer 2 module. 
*   
* @param[in] g_ui32Flags Flag of the isr.
*  
* @return Void. 
*/
void
timer2_isr (uint32_t g_ui32Flags)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    HWREGBITW(&g_ui32Flags, 0) ^= 1;
    timer2_status = true;
    IntMasterEnable();
}

/*** end of file ***/
