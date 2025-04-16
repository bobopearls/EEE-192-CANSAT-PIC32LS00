// this is the c file for the i2c master with the sercom2 initializing and write and reads
// this is so that we can use the stuff in the header file such as the ff:
// stop, repeated start for the stop condition
// then the sercom2 clk init, init, set baud rates and then the pins we will use, etc.

#include <xc.h>
#include "i2c_BME280_sensor_master.h"

void SERCOM2_I2C_Clock_Init(){
    // the APB C periperal clocks are already enabled by default so we dont need to enable it manually
    GCLK_REGS->GCLK_PCHCTRL[19] = 0x00000042; // GCLK peripheral channel, 19 = sercom2 core
                                              // the i2c clk is set to gen2, 4MHz
    while((GCLK_REGS->GCLK_PCHCTRL[19] & 0x00000040)==0) 
        asm("nop");
}

void SERCOM2_I2C_Init(void){
    SERCOM2_I2C_Clock_Init();
    //ctrla uses enable and swrst nits , I2CM means that it is operating as master (s means slave))
    SERCOM2_REGS->I2CM.SERCOM_CTRLA  = SERCOM_I2CM_CTRLA_SWRST_Msk; //reset the module
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U){
        // do nothing
    }
    
    // smart mode enabled to automatically semd am ack instead of doing it manually 
    // this is done after receiving a byte 
    SERCOM2_REGS->I2CM.SERCOM_CTRLB |= SERCOM_I2CM_CTRLB_SMEN_Msk;
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U){ // wait for sync again
        // do nothing
    }
    
    //setting the baud rate for the master (bme280 can handle the standard and fast mode, 100kHz and 400kHz)
    SERCOM2_REGS->I2CM.SERCOM_BAUD = 0x00;
    
    // set up the i2c interrupts such as the data transfer completion and the error handling 
    SERCOM2_REGS->I2CM.SERCOM_INTENSET = SERCOM_I2CM_INTENSET_MB_Msk | 
                                         SERCOM_I2CM_INTENSET_SB_Msk | 
                                         SERCOM_I2CM_INTENSET_ERROR_Msk; // error handling
    // write the ISR (interrupt service routine) that will handle these events in another file
    
    // set up the pins used: PA12 AS THE SDA AND PA13 AS THE SCL
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[6] = 0x22;
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[13] = 0x03;
	PORT_SEC_REGS->GROUP[0].PORT_PINCFG[12] = 0x03;
    
    /* Set Operation Mode (Master), SDA Hold time, run in stand by and i2c master enable */
    SERCOM2_REGS->I2CM.SERCOM_CTRLA = SERCOM_I2CM_CTRLA_INACTOUT(0x2) |  // bus idle timeout
                                      SERCOM_I2CM_CTRLA_MODE_I2C_MASTER | 
                                      SERCOM_I2CM_CTRLA_SDAHOLD_75NS | 
                                      SERCOM_I2CM_CTRLA_SPEED_SM | // standard fast
                                      SERCOM_I2CM_CTRLA_SCLSM(0UL) | 
                                      SERCOM_I2CM_CTRLA_ENABLE_Msk;

    /* Wait for synchronization */
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
    
     /* Initial Bus State: IDLE */
    SERCOM2_REGS->I2CM.SERCOM_STATUS = (uint16_t)SERCOM_I2CM_STATUS_BUSSTATE(0x01UL);

    /* Wait for synchronization */
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
    
}

// original i2c code just had the abort empty
void SERCOM2_I2C_ABORT(){
    // disable the i2c module if active
    SERCOM2_REGS->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_SWRST_Msk;  // Trigger a software reset
    while ((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U) {
        // Wait for the reset to complete
        asm("nop");
    }
    SERCOM2_REGS->I2CM.SERCOM_CTRLA &= ~SERCOM_I2CM_CTRLA_ENABLE_Msk; // turn off enable
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0); 
    
    // enable the i2c module again
    SERCOM2_REGS->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_ENABLE_Msk; // turn on enable
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0); 
    
    // clear MB, SB, and ERROR interrupts
    SERCOM2_REGS->I2CM.SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_MB_Msk | 
                                        SERCOM_I2CM_INTFLAG_SB_Msk |
                                        SERCOM_I2CM_INTFLAG_ERROR_Msk; // intflag will clear before we intenset in the i2c init
    
    // reinitialize to idle
    SERCOM2_REGS->I2CM.SERCOM_STATUS = (uint16_t)SERCOM_I2CM_STATUS_BUSSTATE(0x01UL);
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0); 
    
    // initialize again
    SERCOM2_I2C_Init();
    
}

uint16_t SERCOM2_I2C_Write_Polled_Init(uint8_t addr, uint8_t *buf, uint16_t len, StopCondition S_condt) {
    uint8_t flags = 0;
    uint16_t pos = 0;
    uint32_t temp = 0;

    // this is the set up to send the data to an i2c slave device (aka any of the sensors)
    // polling is used to send data w/ no interrputs
    
    /******** Step 1: Wait until the bus is idle ********/
    while((SERCOM2_REGS->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) != SERCOM_I2CM_STATUS_BUSSTATE_IDLE);
    
    /******** Step 2: Send Start + Address + Direction bit ********/
    SERCOM2_REGS->I2CM.SERCOM_ADDR = (addr << 1) & 0xFE; // ensures write operation
    
    temp = 0;
    while((SERCOM2_REGS->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_MB_Msk) == 0); // wait for MB flag to be HIGH
    // 3: wait for and process the interrupt flags and if error, abort and exit
    while((flags = (SERCOM2_REGS->I2CM.SERCOM_INTFLAG &
                   (SERCOM_I2CM_INTFLAG_MB_Msk |
                    SERCOM_I2CM_INTFLAG_ERROR_Msk))) == 0){
        if(++temp > 10000){
            SERCOM2_I2C_ABORT();
            return 0;
        }
    }
    // check for an error right after addr
    if (flags & SERCOM_I2CM_INTFLAG_ERROR_Msk || 
                SERCOM2_REGS->I2CM.SERCOM_STATUS & 
               (SERCOM_I2CM_STATUS_RXNACK_Msk | SERCOM_I2CM_STATUS_ARBLOST_Msk)){
        SERCOM2_I2C_ABORT();
        return 0;
    }
    /******** Step 3: Send data bytes one by one using for loop ********/
    // 3: send data bytes (loop over len) aka write data and waitfor the acks
    for (pos = 0; pos < len; pos++){
        //poll until we can write the data (until MB flag)
        temp = 0;
        while((SERCOM2_REGS->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_MB_Msk) == 0){
            if (++temp > 10000) {
                SERCOM2_I2C_ABORT();
                return pos;
            }
        }
        
        // write the data byte to the data register 
        SERCOM2_REGS->I2CM.SERCOM_DATA = buf[pos];
        
        // then wait for the operation to complete
        while(SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk){ // (1 << 2)
            if (++temp > 10000) {
                SERCOM2_I2C_ABORT();
                return pos;
            }
        }
        
        // then check for ACK or NCK errors
        if((SERCOM2_REGS->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_RXNACK_Msk) || 
          (SERCOM2_REGS->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_ARBLOST_Msk)){
            SERCOM2_I2C_ABORT();
            return pos; // if error occurs, then abort ad return pos
        }
    }
    
    /******** Step 4: Send stop or repeated start ********/
    temp = 0;
    while((SERCOM2_REGS->I2CM.SERCOM_INTFLAG &(SERCOM_I2CM_INTFLAG_MB_Msk | SERCOM_I2CM_INTFLAG_SB_Msk)) == 0){
        // wait
        // the while statement waits for MB or SB to safely be issued
        if (++temp > 10000) {
            SERCOM2_I2C_ABORT();
            return pos;
        }
    }
    // then we issue the StopConndition cases and what they should do
    switch(S_condt){
        case ACK_THEN_STOP:
            SERCOM2_REGS->I2CM.SERCOM_CTRLB |= (0x3 << SERCOM_I2CM_CTRLB_CMD_Pos); // << 16
            break;
        
        case ACK_THEN_REPEATED_START:
            SERCOM2_REGS->I2CM.SERCOM_CTRLB |= (0x1 << SERCOM_I2CM_CTRLB_CMD_Pos); // ack then issue the repeated start condition
            break;                                                                 // this begins another transmission without a stop
  
    }
    // wait for syncbusy to complete command
    temp = 0;
    while(SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk){
        if (++temp > 10000) {
            SERCOM2_I2C_ABORT();
            return pos;
        }
    }
    // then wait till idle
    temp = 0;
    while((SERCOM2_REGS->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) != SERCOM_I2CM_STATUS_BUSSTATE_IDLE){
        if (++temp > 10000) {
            SERCOM2_I2C_ABORT();
            return pos;
        }
    }
    

    // Wait for IDLE only if STOP was issued
    if (S_condt == ACK_THEN_STOP) {
        temp = 0;
        while((SERCOM2_REGS->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) != SERCOM_I2CM_STATUS_BUSSTATE_IDLE){ // BUSSTATE != IDLE
            if (++temp > 10000) {
                SERCOM2_I2C_ABORT();
                return pos;
            }
        }
    }
    return pos;
}
