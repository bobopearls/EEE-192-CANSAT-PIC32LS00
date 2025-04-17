#include <xc.h>
#include <stdbool.h>
#include <string.h>

static void raise_perf_level(void) {
    /* CODE TAKEN FROM EEE 158, ALBERT DE VILLA */
    
	uint32_t tmp_reg = 0;
	
	/*
	 * The chip starts in PL0, which emphasizes energy efficiency over
	 * performance. However, we need the latter for the clock frequency
	 * we will be using (~24 MHz); hence, switch to PL2 before continuing.
	 */
	PM_REGS->PM_INTFLAG = 0x01;
	PM_REGS->PM_PLCFG = 0x02;
	while ((PM_REGS->PM_INTFLAG & 0x01) == 0)
		asm("nop");
	PM_REGS->PM_INTFLAG = 0x01;
	
	/*
	 * Power up the 48MHz DFPLL.
	 * 
	 * On the Curiosity Nano Board, VDDPLL has a 1.1uF capacitance
	 * connected in parallel. Assuming a ~20% error, we have
	 * STARTUP >= (1.32uF)/(1uF) = 1.32; as this is not an integer, choose
	 * the next HIGHER value.
	 */
	NVMCTRL_SEC_REGS->NVMCTRL_CTRLB = (2 << 1) ;
	SUPC_REGS->SUPC_VREGPLL = 0x00000302;
	while ((SUPC_REGS->SUPC_STATUS & (1 << 18)) == 0)
		asm("nop");
	
	/*
	 * Configure the 48MHz DFPLL.
	 * 
	 * Start with disabling ONDEMAND...
	 */
	OSCCTRL_REGS->OSCCTRL_DFLLCTRL = 0x0000;
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	/*
	 * ... then writing the calibration values (which MUST be done as a
	 * single write, hence the use of a temporary variable)...
	 */
	tmp_reg  = *((uint32_t*)0x00806020);
	tmp_reg &= ((uint32_t)(0b111111) << 25);
	tmp_reg >>= 15;
	tmp_reg |= ((512 << 0) & 0x000003ff);
	OSCCTRL_REGS->OSCCTRL_DFLLVAL = tmp_reg;
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	// ... then enabling ...
	OSCCTRL_REGS->OSCCTRL_DFLLCTRL |= 0x0002;
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	/*
	 * Configure GCLK_GEN2 as described (4MHz); this one will become the main
	 * clock for slow/medium-speed peripherals, as GCLK_GEN0 will be
	 * stepped up for 24 MHz operation.
     * 
     *  GENEN = 0x01 (ENABLE)
     *  SRC   = 0x05 (OSC16M)
	 */
	GCLK_REGS->GCLK_GENCTRL[2] = 0x00000105;
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 4)) != 0)
		asm("nop");
	
	// Switch over GCLK_GEN0 to DFLL48M, with DIV=2 to get 24 MHz.
    // TODO: switch DIV bit to DIV=1 for 48MHz, if needed
	GCLK_REGS->GCLK_GENCTRL[0] = 0x00020107;
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 2)) != 0)
		asm("nop");
	
	// Done. We're now at 24 MHz (for GCLK_GEN2) and 24/48MHz GLCK_GEN0.
	return;
}

void platform_init(void) {
    
    // Raise the power level
	raise_perf_level();
    
}
