#include <xc.h>
#include "../platform.h"

/*
    Initialization function based off code generated through Microchip Code Configurator. (c) Microchip Inc.
    Read/Write functions written by Allen.
*/

static void SERCOM2_I2C_Clock_Init() {
    // Set clock to GEN2 (4MHz)
    GCLK_REGS->GCLK_PCHCTRL[19] = 0x00000042;
    while ((GCLK_REGS->GCLK_PCHCTRL[19] & 0x00000040) == 0)
        asm("nop");
}

void SERCOM2_I2C_Initialize(void) {
    SERCOM2_I2C_Clock_Init();

    SERCOM2_REGS->I2CM.SERCOM_CTRLA = SERCOM_I2CM_CTRLA_SWRST_Msk;
    while ((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U);

    SERCOM2_REGS->I2CM.SERCOM_CTRLB = SERCOM_I2CM_CTRLB_SMEN_Msk;
    while ((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U);

    SERCOM2_REGS->I2CM.SERCOM_BAUD = 19;

    SERCOM2_REGS->I2CM.SERCOM_INTENSET |= SERCOM_I2CM_INTENSET_MB(1) | SERCOM_I2CM_INTENSET_SB(1) | SERCOM_I2CM_INTENSET_ERROR(1);

    // Configure PA12 (SDA) and PA13 (SCL)
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[6] = 0x22;
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[13] = 0x03;
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[12] = 0x03;

    SERCOM2_REGS->I2CM.SERCOM_CTRLA = SERCOM_I2CM_CTRLA_INACTOUT(0x2) |
        SERCOM_I2CM_CTRLA_MODE_I2C_MASTER |
        SERCOM_I2CM_CTRLA_SDAHOLD_75NS |
        SERCOM_I2CM_CTRLA_SPEED_SM |
        SERCOM_I2CM_CTRLA_SCLSM(0UL) |
        SERCOM_I2CM_CTRLA_ENABLE_Msk;

    while ((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U);

    SERCOM2_REGS->I2CM.SERCOM_STATUS = (uint16_t)SERCOM_I2CM_STATUS_BUSSTATE(0x01UL);
    while ((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0U);
}

uint16_t SERCOM2_I2C_Write_Polled(uint8_t addr, uint8_t *buf, uint16_t len) {
    uint16_t pos = 0;
    volatile uint32_t timeout;

    // Wait until bus is IDLE
    timeout = 1000000;
    while ((((SERCOM2_REGS->I2CM.SERCOM_STATUS >> 4) & 0x3) != 0x1) && --timeout);
    if (!timeout) {
        SERCOM2_I2C_Abort();
        return 0;
    }

    // Clear interrupt flags
    SERCOM2_REGS->I2CM.SERCOM_INTFLAG = 0xFF;

    // Send address with write bit (0)
    SERCOM2_REGS->I2CM.SERCOM_ADDR = addr << 1;
    while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY);

    // Check for errors (NACK or Bus Error)
    if (SERCOM2_REGS->I2CM.SERCOM_STATUS & ((1 << 2) | (1 << 1))) {
        SERCOM2_I2C_Abort();
        return 0;
    }

    for (pos = 0; pos < len; pos++) {
        timeout = 1000000;
        while (!(SERCOM2_REGS->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_MB_Msk) && --timeout);
        if (!timeout) {
            SERCOM2_I2C_Abort();
            return 0;
        }

        SERCOM2_REGS->I2CM.SERCOM_DATA = buf[pos];
        while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY);

        if (SERCOM2_REGS->I2CM.SERCOM_STATUS & ((1 << 2) | (1 << 1))) {
            SERCOM2_I2C_Abort();
            return 0;
        }
    }

    // Wait for last byte to finish
    timeout = 1000000;
    while (!(SERCOM2_REGS->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_MB_Msk) && --timeout);
    if (!timeout) {
        SERCOM2_I2C_Abort();
        return 0;
    }

    // STOP condition
    SERCOM2_REGS->I2CM.SERCOM_CTRLB =
        (SERCOM2_REGS->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_CMD_Msk) |
        SERCOM_I2CM_CTRLB_CMD(3);
    while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY);

    return pos;
}

uint16_t SERCOM2_I2C_Read_Polled(uint8_t addr, uint8_t *buf, uint16_t len) {
    uint16_t pos = 0;
    volatile uint32_t timeout;

    if (len == 0) return 0;

    timeout = 1000000;
    while ((((SERCOM2_REGS->I2CM.SERCOM_STATUS >> 4) & 0x3) != 0x1) && --timeout);
    if (!timeout) return 0;

    // Clear interrupt flags
    SERCOM2_REGS->I2CM.SERCOM_INTFLAG = 0xFF;

    // Send address with read bit (1)
    SERCOM2_REGS->I2CM.SERCOM_ADDR = (addr << 1) | 1;
    while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY);

    if (SERCOM2_REGS->I2CM.SERCOM_STATUS & ((1 << 2) | (1 << 1))) {
        SERCOM2_I2C_Abort();
        return 0;
    }

    for (pos = 0; pos < len; pos++) {
        if (pos == (len - 1)) {
            // Prepare to NACK on last byte
            SERCOM2_REGS->I2CM.SERCOM_CTRLB |= SERCOM_I2CM_CTRLB_ACKACT_Msk;
        } else {
            // ACK next byte
            SERCOM2_REGS->I2CM.SERCOM_CTRLB &= ~SERCOM_I2CM_CTRLB_ACKACT_Msk;
        }
        while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY);

        timeout = 1000000;
        while (!(SERCOM2_REGS->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_SB_Msk) && --timeout);
        if (!timeout) {
            SERCOM2_I2C_Abort();
            return 0;
        }

        buf[pos] = SERCOM2_REGS->I2CM.SERCOM_DATA;
        while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY);
    }

    // Send STOP condition
    SERCOM2_REGS->I2CM.SERCOM_CTRLB =
        (SERCOM2_REGS->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_CMD_Msk) |
        SERCOM_I2CM_CTRLB_CMD(3);
    while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY);

    return pos;
}

void SERCOM2_I2C_Abort() {
    SERCOM2_REGS->I2CM.SERCOM_CTRLA &= ~SERCOM_I2CM_CTRLA_ENABLE_Msk;
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0);

    SERCOM2_REGS->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_ENABLE_Msk;
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0);

    // Reinitialize bus to IDLE
    SERCOM2_REGS->I2CM.SERCOM_STATUS = (uint16_t)SERCOM_I2CM_STATUS_BUSSTATE(0x01UL);
    while((SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY) != 0);

    // Send STOP condition manually
    SERCOM2_REGS->I2CM.SERCOM_CTRLB =
        (SERCOM2_REGS->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_CMD_Msk) |
        SERCOM_I2CM_CTRLB_CMD(3);
    while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY);
}
