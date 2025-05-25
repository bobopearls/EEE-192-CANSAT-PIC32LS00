// MARIE NOTE: I MODIFIED THE ORIGINAL GPIO FILE TO ACCOMODATE THE ADC SENSOR FUNCTIONS
// YOU CAN SEE THE FUNCTIONS IN PLATFORM_INIT, AND THEN JUST CHECK HOW I DID IT USING CTRL + F

#include <xc.h>
#include <stdbool.h>
#include <string.h>

#include "../platform.h"

// Initializers defined in other platform_*.c files
extern void platform_systick_init(void);
extern void platform_usart_init(void);
extern void platform_usart_tick_handler(const platform_timespec_t *tick);

/////////////////////////////////////////////////////////////////////////////

// Enable higher frequencies for higher performance
static void raise_perf_level(void)
{
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
	tmp_reg  = ((uint32_t)0x00806020);
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
	
	// ... then restoring ONDEMAND.
//	OSCCTRL_REGS->OSCCTRL_DFLLCTRL |= 0x0080;
//	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
//		asm("nop");
	
	/*
	 * Configure GCLK_GEN2 as described; this one will become the main
	 * clock for slow/medium-speed peripherals, as GCLK_GEN0 will be
	 * stepped up for 24 MHz operation.
	 */
	GCLK_REGS->GCLK_GENCTRL[2] = 0x00000105;
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 4)) != 0)
		asm("nop");
	
	// Switch over GCLK_GEN0 to DFLL48M, with DIV=2 to get 24 MHz.
	GCLK_REGS->GCLK_GENCTRL[0] = 0x00020107;
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 2)) != 0)
		asm("nop");
	
	// Done. We're now at 24 MHz.
	return;
}

/*
 * Configure the EIC peripheral
 * 
 * NOTE: EIC initialization is split into "early" and "late" halves. This is
 *       because most settings within the peripheral cannot be modified while
 *       EIC is enabled.
 */
static void EIC_init_early(void)
{
	/*
	 * Enable the APB clock for this peripheral
	 * 
	 * NOTE: The chip resets with it enabled; hence, commented-out.
	 * 
	 * WARNING: Incorrect MCLK settings can cause system lockup that can
	 *          only be rectified via a hardware reset/power-cycle.
	 */
	// MCLK_REGS->MCLK_APBAMASK |= (1 << 10);
	
	/*
	 * In order for debouncing to work, GCLK_EIC needs to be configured.
	 * We can pluck this off GCLK_GEN2, configured for 4 MHz; then, for
	 * mechanical inputs we slow it down to around 15.625 kHz. This
	 * prescaling is OK for such inputs since debouncing is only employed
	 * on inputs connected to mechanical switches, not on those coming from
	 * other (electronic) circuits.
	 * 
	 * GCLK_EIC is at index 4; and Generator 2 is used.
	 */
	GCLK_REGS->GCLK_PCHCTRL[4] = 0x00000042;
	while ((GCLK_REGS->GCLK_PCHCTRL[4] & 0x00000042) == 0)
		asm("nop");
	
	// Reset, and wait for said operation to complete.
	EIC_SEC_REGS->EIC_CTRLA = 0x01;
	while ((EIC_SEC_REGS->EIC_SYNCBUSY & 0x01) != 0)
		asm("nop");
	
	/*
	 * Just set the debounce prescaler for now, and leave the EIC disabled.
	 * This is because most settings are not editable while the peripheral
	 * is enabled.
	 */
	EIC_SEC_REGS->EIC_DPRESCALER = (0b0 << 16) | (0b0000 << 4) |
		                       (0b1111 << 0);
	return;
}
static void EIC_init_late(void)
{
	/*
	 * Enable the peripheral.
	 * 
	 * Once the peripheral is enabled, further configuration is almost
	 * impossible.
	 */
	EIC_SEC_REGS->EIC_CTRLA |= 0x02;
	while ((EIC_SEC_REGS->EIC_SYNCBUSY & 0x02) != 0)
		asm("nop");
	return;
}

// Configure the EVSYS peripheral
static void EVSYS_init(void)
{
	/*
	 * Enable the APB clock for this peripheral
	 * 
	 * NOTE: The chip resets with it enabled; hence, commented-out.
	 * 
	 * WARNING: Incorrect MCLK settings can cause system lockup that can
	 *          only be rectified via a hardware reset/power-cycle.
	 */
	// MCLK_REGS->MCLK_APBAMASK |= (1 << 0);
	
	/*
	 * EVSYS is always enabled, but may be in an inconsistent state. As
	 * such, trigger a reset.
	 */
	EVSYS_SEC_REGS->EVSYS_CTRLA = 0x01;
	asm("nop");
	asm("nop");
	asm("nop");
	return;
}

//////////////////////////////////////////////////////////////////////////////

// Configure any peripheral/s used to effect blinking
static void blink_init(void) {
    
    // Initialize PA15 as Output
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 15);  
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[15] = 0x1; 
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[7] = 0x90;
    
    // Main Clock enabled by default
    GCLK_REGS -> GCLK_PCHCTRL[25] = 0x00000040;
    while ((GCLK_REGS -> GCLK_PCHCTRL [25] & 0x00000040) == 0) ; // Wait for synchronization

    /* Reset TCC */
    TCC0_REGS->TCC_CTRLA = 0x01; // Set SWRST bit to 1 to reset
    while(TCC0_REGS->TCC_SYNCBUSY & ~(1<<0)); // Wait for synchronization
    
    /* Clock Prescaler and Mode */    
    TCC0_REGS->TCC_CTRLA = (1 << 12) | (7 << 8); // Precsync = PRESC | Prescaler = 1024
    
    TCC0_REGS->TCC_WEXCTRL = TCC_WEXCTRL_OTMX(0UL); // Default configuration
    TCC0_REGS->TCC_WAVE = (2 << 0) | (0 << 4) ;  // 0x2 NPWM Normal PWM PER TOP/Zero or Single slope PWM
                                                // RAMP 1 operation (Polarity 1) - bit 16, set at CCx, clear at TOP 
    
    /* Configure duty cycle values 
    PER = (GCLK/frequency*prescaler) - 1 
    PER = (24E6/(2*1024)) - 1 = 11717.75
    */    
    
    TCC0_REGS->TCC_PER = 11717.75; // Set the period value
    TCC0_REGS->TCC_CC[1] = 0; // Set the capture / compare register 0 or counter value       
    
    /* TCC enable */
    TCC0_REGS->TCC_CTRLA |= (1 << 1); // Enables TCC
    while(TCC0_REGS->TCC_SYNCBUSY & ~(1<<1)); // Wait for synchronization
    
	return;
}

int read_count(){
    // Allow read access of COUNT register
    TCC0_REGS->TCC_CTRLBSET |= (1 << 7);
    return TCC0_REGS->TCC_COUNT; // Return back the counter value
}

// Adjust the cycle for the LED
void platform_blink_modify(uint8_t val) {
    
    while(read_count() != 0); // Wait for synchronization
    
    if (val == 0) {
        TCC0_REGS->TCC_PER = 11717.75; // Set the period value to 1s
        TCC0_REGS->TCC_CC[1] = 0; // Set the LED to be always off
    }
    
	else if (val == 1) {
        TCC0_REGS->TCC_PER = 11717.75; // Set the period value to 1s
        TCC0_REGS->TCC_CC[1] = 11717.75 * 0.1; // Set the on time to be 100ms
    }
    else if (val == 2) {
        TCC0_REGS->TCC_PER = 11717.75 * 0.5; // Set the period value to 500ms
        TCC0_REGS->TCC_CC[1] = 11717.75 * 0.1; // Set the on time to be 100ms
    }
    else if (val == 3) {
        TCC0_REGS->TCC_PER = 11717.75 * 0.3; // Set the period value to 300ms
        TCC0_REGS->TCC_CC[1] = 11717.75 * 0.1; // Set the on time to be 100ms
    }
    else if (val == 4) {
        TCC0_REGS->TCC_PER = 11717.75; // Set the period value to 1s
        TCC0_REGS->TCC_CC[1] = 11717.75; // Set the LED to be always on
    }
    
	return;
}

//////////////////////////////////////////////////////////////////////////////

/*
 * Per the datasheet for the PIC32CM5164LS00048, PA23 belongs to EXTINT[2],
 * which in turn is Peripheral Function A. The corresponding Interrupt ReQuest
 * (IRQ) handler is thus named EIC_EXTINT_2_Handler.
 */
static volatile uint16_t pb_press_mask = 0;
void __attribute__((used, interrupt())) EIC_EXTINT_2_Handler(void)
{
	pb_press_mask &= ~PLATFORM_PB_ONBOARD_MASK;
	if ((EIC_SEC_REGS->EIC_PINSTATE & (1 << 2)) == 0)
		pb_press_mask |= PLATFORM_PB_ONBOARD_PRESS;
	else
		pb_press_mask |= PLATFORM_PB_ONBOARD_RELEASE;
	
	// Clear the interrupt before returning.
	EIC_SEC_REGS->EIC_INTFLAG |= (1 << 2);
	return;
}

static void PB_init(void)
{
	/*
	 * Configure PA23.
	 * 
	 * NOTE: PORT I/O configuration is never separable from the in-circuit
	 *       wiring. Refer to the top of this source file for each PORT
	 *       pin assignments.
	 */
	PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = 0x00800000;
	PORT_SEC_REGS->GROUP[0].PORT_PINCFG[23] = 0x03;
	PORT_SEC_REGS->GROUP[0].PORT_PMUX[(23 >> 1)] &= ~(0xF0);
	
	/*
	 * Debounce EIC_EXT2, where PA23 is.
	 * 
	 * Configure the line for edge-detection only.
	 * 
	 * NOTE: EIC has been reset and pre-configured by the time this
	 *       function is called.
	 */
	EIC_SEC_REGS->EIC_DEBOUNCEN |= (1 << 2);
	EIC_SEC_REGS->EIC_CONFIG0   &= ~((uint32_t)(0xF) << 8);
	EIC_SEC_REGS->EIC_CONFIG0   |=  ((uint32_t)(0xB) << 8);
	
	/*
	 * NOTE: Even though interrupts are enabled here, global interrupts
	 *       still need to be enabled via NVIC.
	 */
	EIC_SEC_REGS->EIC_INTENSET = 0x00000004;
	return;
}

// Get the mask of currently-pressed buttons
uint16_t platform_pb_get_event(void)
{
	uint16_t cache = pb_press_mask;
	
	pb_press_mask = 0;
	return cache;
}

//////////////////////////////////////////////////////////////////////////////

/*
 * Configure the NVIC
 * 
 * This must be called last, because interrupts are enabled as soon as
 * execution returns from this function.
 */
static void NVIC_init(void)
{
	/*
	 * Unlike AHB/APB peripherals, the NVIC is part of the Arm v8-M
	 * architecture core proper. Hence, it is always enabled.
	 */
	__DMB();
	__enable_irq();
	NVIC_SetPriority(EIC_EXTINT_2_IRQn, 3);
	NVIC_SetPriority(SysTick_IRQn, 3);
	NVIC_EnableIRQ(EIC_EXTINT_2_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
	return;
}

// INITIALIZE CLOCKS 
void CLOCK_INIT(void){
    // GCK0 : Div factor 1 | Source Select 7 | Generic Clock Generator Enable
    GCLK_REGS->GCLK_GENCTRL[0] = (1<<16) | (7 <<0) | (1 <<8) ;
    while ((GCLK_REGS->GCLK_SYNCBUSY & (1<<2)) != 0) ;
    // ADC Bus Clock : Generic Clock Generator Value | Channel Enable
    GCLK_REGS->GCLK_PCHCTRL[28] = (0<<0) | (1<<6) ;
    while ((GCLK_REGS->GCLK_PCHCTRL[28] & (1<<6) ) != (1 <<6) ) ;
}

void ADC_INIT(void){
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 6); // added port PA06 change
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[6] = 0x1U;
    //PORT_SEC_REGS->GROUP[0].PORT_PMUX[3] = (PORT_SEC_REGS->GROUP[1].PORT_PMUX[1] & ~0x0F) | 0x1;
    
    /* Reset ADC */
    ADC_REGS->ADC_CTRLA = (1<<0) ;
    while ((ADC_REGS->ADC_SYNCBUSY & (1<<0) ) == (1<<0)) ;
    /* Prescaler */
    ADC_REGS->ADC_CTRLB = (2<<0) ;
    /* Sampling length */
    ADC_REGS->ADC_SAMPCTRL = (3<<0) ;
    /* Reference */
    ADC_REGS->ADC_REFCTRL = (5<<0) ;
    /* Input pin */
    ADC_REGS->ADC_INPUTCTRL = (0x04<<0) ;
    /* Resolution & Operation Mode */
    ADC_REGS->ADC_CTRLC = (uint16_t) ((0<<4) | (0 <<8) ) ; //resolution was changed
    /* Clear all interrupt flags */
    ADC_REGS->ADC_INTFLAG = (uint8_t) 0x07 ;
    while (0U != ADC_REGS->ADC_SYNCBUSY) ; 
}

/* Enable ADC module */
void ADC_ENABLE ( void ){
    ADC_REGS -> ADC_CTRLA |= (1 <<1) ;
    while (0U != ADC_REGS -> ADC_SYNCBUSY ) ;
}

/////////////////////////////////////////////////////////////////////////////

// Initialize the platform
void platform_init(void)
{
	// Raise the power level
	raise_perf_level();
	
	// Early initialization
	EVSYS_init();
	EIC_init_early();
    
    platform_usart_init();

	
	// **Clock & ADC Setup After UART**
    CLOCK_INIT();

    ADC_INIT();
    ADC_ENABLE();

    // Late initialization
    EIC_init_late();
    platform_systick_init();
    NVIC_init();

	return;
}

// Do a single event loop
void platform_do_loop_one(void)
{
	platform_timespec_t tick;
	
	/*
	 * Some routines must be serviced as quickly as is practicable. Do so
	 * now.
	 */
	platform_tick_hrcount(&tick);
	platform_usart_tick_handler(&tick);
}
