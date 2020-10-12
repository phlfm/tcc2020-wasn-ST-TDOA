#ifndef _DEBUG_CONFIG_H
#define _DEBUG_CONFIG_H

/* define DEBUG_SWV to use serial wire viewer capabilities */
#define DEBUG_SWV

/* CH_PLOT allows user to inspect sample buffer for a given channel using SWV
 * to enable sample buffer inspection CH_PLOT must be defined
 * CH_PLOT also works as channel selection (0, 1 or 2)
 * CH_PLOT_STEP represents how many samples to skip between iterations
 * CH_WAIT_TIMER is how many cycles to wait before re-enabling sampling timer
 * CH_WAIT_UPDATE is how many cycles to wait before updating ch_plot. If the update is too fast, SWV can't keep up.
 *
 * user can monitor ch_plot for the sample buffer value and iCH for the index of the current value
 *
 **/
#define CH_PLOT 0 // DEBUG_SWV must be enabled to use CH_PLOT
#ifdef CH_PLOT
	#define CH_PLOT_STEP 20
	#define CH_WAIT_TIMER 1000
	#define CH_WAIT_UPDATE 10000
#endif //ifdef CH_PLOT


#endif //_DEBUG_CONFIG_H
