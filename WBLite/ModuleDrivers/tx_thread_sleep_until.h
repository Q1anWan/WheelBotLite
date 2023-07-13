/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_sleep_until                              PORTABLE C      */
/*                                                           1.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    qianwan                                                             */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles application thread sleep requests.  If the    */
/*    sleep request was called from a non-thread, an error is returned.   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    timer_tick                         timer ticks where sleep begin    */
/*    inc_tick                           Number of timer ticks to sleep   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                Return completion status      */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _tx_thread_system_suspend         Actual thread suspension          */
/*    _tx_thread_system_ni_suspend      Non-interruptable suspend thread  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  19-04-2023     qianwan                  Initial Version 1.0           */
/**************************************************************************/

#ifndef TX_THREAD_SLEEP_UNTIL
#define TX_THREAD_SLEEP_UNTIL

#ifdef __cplusplus
extern "C" {
#endif
	
#include "tx_api.h"
	
#define tx_thread_sleep_until _tx_thread_sleep_until
ULONG  _tx_thread_sleep_until(ULONG *timer_tick, ULONG inc_tick);
	
#ifdef __cplusplus
}
#endif
#endif