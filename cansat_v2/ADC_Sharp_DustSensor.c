////////////////////////////////////////////////////////////////////////////////
/*
/* this code is meant for the ADC Sensor: SHARP GP2Y1010AU0F
/* the sensor's main purpose is to detect things like:
/* house dust and cigarette smoke (which contains particles we want to measure)
/* 
/* Code written by: QUITORIANO, Maria Louise C. 
/* for EEE 192 Class (template taken from EEE 158)
*///////////////////////////////////////////////////////////////////////////////

// PORTS USED:
// PA02 as the analog input for ADC_AIN[0]
// PA04 aa the external reference voltage for REG_VREFB
// find a port that you can use for the UART display

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <stdbool.h>

void PORT_INIT(void);
void CLOCK_INIT(void);
void ADC_INIT(void);
void ADC_ENABLE(void);
void ADC_ConversionStart(void);
uint16_t ADC_ConverstionGetRes(void);
bool ADC_ConversionStatusGet(void);
void delay_ms(int delay);

// initialize the variable values
uint16_t adc_value = 0x0000; // this holds the raw 12 bit adc result from the analog input
uint16_t mask = 0x0000; // used in bitwise operations to isolate bits from the adc_value
uint16_t move = 0x0000; // stores the result of the masked bit after shifting it to bit 0
// these likely simulate the serial output over something where each bit of the adc_value will
// be outputted depending on what the adc_value bit is

// set up the groups that will be used in MAIN

// initialize the pins to be used 
void PORT_INIT(void){
    // PA02 = ANALOG INPUT FOR THE SENSOR
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[2] = 0x1U;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[1] |= 0x1U;
    
    // PA04 AS THE EXTERNAL REF VOLTAGE REF_VREFB
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[4] = 0x1U ;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[2] = 0x1U ;
    
    // PA15 AS THE OUTPUT PIN GPIO_PA15
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1<<15) ;
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[15] = 0x0U;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[7] = 0x0U;
}

// INITIALIZE CLOCKS 
void CLOCK_INIT(void){
    // GCK0 : Div factor 1 | Source Select 7 | Generic Clock Generator Enable
    GCLK_REGS->GCLK_GENCTRL[0] = (1<<16) | (7 <<0) | (1 <<8) ;
    while ((GCLK_REGS->GCLK_SYNCBUSY & (1<<2)) == 0) ;
    // ADC Bus Clock : Generic Clock Generator Value | Channel Enable
    GCLK_REGS->GCLK_PCHCTRL[28] = (0<<0) | (1<<6) ;
    while ((GCLK_REGS->GCLK_PCHCTRL[28] & (1<<6) ) != (1 <<6) ) ;
}

void ADC_INIT(void){
    /* Reset ADC */
    ADC_REGS->ADC_CTRLA = (1<<0) ;
    while ((ADC_REGS->ADC_SYNCBUSY & (1<<0) ) == (1<<0)) ;
    /* Prescaler */
    ADC_REGS->ADC_CTRLB = (2<<0) ;
    /* Sampling length */
    ADC_REGS->ADC_SAMPCTRL = (3<<0) ;
    /* Reference */
    ADC_REGS->ADC_REFCTRL = (4<<0) ;
    /* Input pin */
    ADC_REGS->ADC_INPUTCTRL = (0<<0) ;
    /* Resolution & Operation Mode */
    ADC_REGS->ADC_CTRLC = (uint16_t) ((0<<4) | (0 <<10) ) ; //resolution was changed
    /* Clear all interrupt flags */
    ADC_REGS->ADC_INTFLAG = (uint8_t) 0x07 ;
    while (0U != ADC_REGS->ADC_SYNCBUSY) ;
}

/* Enable ADC module */
void ADC_ENABLE ( void ){
    ADC_REGS -> ADC_CTRLA |= (1 <<1) ;
    while (0U != ADC_REGS -> ADC_SYNCBUSY ) ;
}

/* Start the ADC conversion by SW */
void ADC_ConversionStart ( void ){
    ADC_REGS->ADC_SWTRIG |= (1 <<1) ;
    while ((ADC_REGS -> ADC_SYNCBUSY & (1 <<10) ) == (1 <<10)) ;
}
/* Read the conversion result */
uint16_t ADC_ConverstionGetRes ( void ){
    return ( uint16_t ) ADC_REGS -> ADC_RESULT ;
}
/* Check whether result is ready */
bool ADC_ConversionStatusGet ( void ){
    bool status ;
    status = ((( ADC_REGS -> ADC_INTFLAG & (1 <<0)) >> 0) != 0U ) ;
    if (status == true ){
        ADC_REGS -> ADC_INTFLAG = (1 <<0) ;
    }
    return status ;
}

// NEW BLOCK ADDED FROM THE GIVEN ADC TEMPLATE TO CALCULATE THE CONVERSION
float calculate_dust_density(uint16_t adc_value){
    float voltage = (adc_value / 4095.0f) * 3.3f; // convert to voltage (this assumes 3.3V)
    float delta_v = voltage - 0.9f; // subtract the clean air voltage from the total air
    if(delta_v < 0.0f) delta_v = 0.0f; // do not allow negative values
    return (delta_v / 5.0f); // return values and note that ever 0.5V increase means that we have +0.1 mg/m^3
}

// Blocking delay (very rough approximation)
void delay_ms(int delay) {
    for (; delay > 0; delay--) {
        for (int i = 0; i < 2657; i++);
    }
}

int main (void){
    // initialize
    PORT_INIT(); // set up the GPIOs (PA02 ADC INPUT, and PA15 as the temporary output)
    CLOCK_INIT(); // set up the system and the ADC clocks
    ADC_INIT(); // configure the ADC registers
    
    // then turn on the adc
    ADC_ENABLE();
    
    float dust_density = 0.0f;
    
    while(true){
        ADC_ConversionStart(); // begin the ADC sampling for the sensor
        while( !ADC_ConversionStatusGet() ); // while the ADC result is not ready
        adc_value = ADC_ConverstionGetRes(); // read the converted DIGITAL result
        
        dust_density = calculate_dust_density(adc_value); // convert ADC to dust density (mg/m^3)
        
        // so the PA15 should blink faster if the dust detected is above 0.2 mg/m^3
        if(dust_density > 0.5f){
            PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 15); // TURN ON THE LED
            delay_ms(1000); // delay of 100ms
            PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 15); // TURN OFF THE LED
            delay_ms(1000); // delay of 100 ms again
        } else if (dust_density > 0.2f) {
            PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 15); // TURN ON THE LED
            delay_ms(500); // delay of 500 ms
            PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 15);
            delay_ms(500); // delay of 500 ms
        } else{ // clean air
            PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 15); // TURN ON THE LED           
        }
    }
    
    return EXIT_FAILURE;
}
