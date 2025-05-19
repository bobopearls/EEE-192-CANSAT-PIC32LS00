#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "platform.h"


#define BME280_ADDR              0x76  //default I2C address, low address, so when it is high it will be 0x77
#define BME280_REG_ID            0xD0  //stores the chip ID of the BME280. if we read from this register, the program will know that this is the BME280
#define BME280_CHIP_ID           0x60  //unique identifier for verification and error detection
                                       //will use this to compare later to verify if the correct id is placed
#define BME280_REG_RESET         0xE0  //allows for BME280 "soft reset" but only for the sensor. 
                                       //if we write 0xB6 to this register, it does the soft reset (accd. to Bosch mauf.)
#define BME280_REG_CTRL_HUMIDITY 0xF2  //regsiter controls the humidty measurement (also oversampling)
                                       //Bits [2:0] control oversampling:
                                       //000: Humidity measurement skipped
                                       //001: 1? oversampling (default)
                                       //010: 2? oversampling
                                       //011: 4? oversampling
                                       //100: 8? oversampling
                                       //101: 16? oversampling NOTE: Higher oversampling is more accurate, but uses more power (WE WILL USE THIS, CHANGE IT LATUER)
#define BME280_REG_CTRL_MEAS     0xF4  //temperature and pressure measurement (oversampling and power mode)
                                       //use 0x27 (00100111 binary), which means:
                                       //Temperature: 1? oversampling
                                       //Pressure: 1? oversampling
                                       //Mode: Normal mode
#define BME280_REG_CONFIG        0xF5  //configures stand-by time, filter settings, and enabling or disabling the SPI (we don't need because I2C!)
                                       //Bits [7:5]: Standby time in normal mode:
                                       //000: 0.5ms
                                       //001: 62.5ms
                                       //010: 125ms and so on
                                       //Bits [4:2]: Filter coefficient:
                                       //000: Filter off
                                       //001: 2? filter
                                       //010: 4? filter and so on
                                       //Bit [0]: SPI interface mode (not relevant for I2C)
#define BME280_REG_DATA          0xF7  //MSB of pressure measurement data (full pressure valve requires additional reading bytes)
                                       //Pressure uses F7 to F9 (3-bytes, 20 bits)
                                       //Temperature uses FA to FC (also 3-bytes)
                                       //Humidity uses FD and FE (2-bytes, 16 bits)
#define BME280_REG_CALIB         0x88  //base address for calibrating data that stores the raw data that the sensor outputs
                                       //basically, not yet the final values since we are just getting it from non-volatile memory
                                       //from 0x88 to 0xA1, we have the Temp. Callib, 6 bytes (T1, T2, T3) and then Pres. Callib, 18 bytes (P1 to P9)
#define BME280_REG_CALIB_H1      0xA1  //contains H1 (first humidity data, separate from others)
#define BME280_REG_CALIB_H2      0xE1  //H2, then we reach till 0xE7 because we have till H6
#define SEA_LVL_PRES         101355.0  //constant standard atmospheric pressure, Po1

// ADC definitions for dust sensor
#define ADC_VREF 3.3f
#define ADC_RESOLUTION 4095.0f  // 12-bit ADC resolution
#define CLEAN_AIR_VOLTAGE 0.9f  // Clean air baseline voltage
#define DUST_CONVERSION_FACTOR 5.0f  // Conversion factor for dust density

#define LOOP_DELAY 6000000 // ~5 seconds

static const char banner_msg[] =
"\033[0m\033[2J\033[1;1H"
"";

// --- State ---
typedef struct prog_state_type {
    uint16_t flags;
    uint8_t lc;
    platform_usart_tx_bufdesc_t tx_desc[4];
    char tx_buf[512];
    uint16_t tx_blen;
    platform_usart_rx_async_desc_t rx_desc;
    uint16_t rx_desc_blen;
    char rx_desc_buf[16];
} prog_state_t;

static void delay_loop(volatile uint32_t loops) {
    while (loops--) {
        __asm__ __volatile__("nop");
    }
}

static void usart_cdc_tx_blocking(const char *msg) {
    while (*msg) {
        while (!(SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
        SERCOM3_REGS->USART_INT.SERCOM_DATA = *msg++;
    }
    delay_loop(5000);
}

// --- BME280 I2C Helpers ---
static bool BME280_ReadReg(uint8_t reg, uint8_t *data, uint8_t len) {
    const uint8_t max_retries = 3;
    
    for (uint8_t retry = 0; retry < max_retries; retry++) {
        if (SERCOM2_I2C_Write_Polled(BME280_ADDR, &reg, 1)) {
            // Small delay between write and read operations
            delay_loop(1000);
            
            if (SERCOM2_I2C_Read_Polled(BME280_ADDR, data, len)) {
                return true;  // Success
            }
        }
        // Wait before retry
        delay_loop(5000);
    }
    
    usart_cdc_tx_blocking("I2C communication failed after retries\r\n");
    return false;
}

static bool BME280_WriteReg(uint8_t reg, uint8_t value) {
    const uint8_t max_retries = 3;
    
    for (uint8_t retry = 0; retry < max_retries; retry++) {
        uint8_t buf[2] = { reg, value };
        if (SERCOM2_I2C_Write_Polled(BME280_ADDR, buf, 2)) {
            return true;
        }
        delay_loop(5000);
    }
    
    usart_cdc_tx_blocking("I2C write failed after retries\r\n");
    return false;
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

void ADC_ENABLE(void) {
    ADC_REGS->ADC_CTRLA |= (1<<1);
    while (0U != ADC_REGS->ADC_SYNCBUSY);
}

void ADC_ConversionStart(void) {
    ADC_REGS->ADC_SWTRIG |= (1<<1);
    while ((ADC_REGS->ADC_SYNCBUSY & (1<<10)) == (1<<10));
}

uint16_t ADC_ConversionGetRes(void) {
    return (uint16_t)ADC_REGS->ADC_RESULT;
}

bool ADC_ConversionStatusGet(void) {
    bool status;
    status = (((ADC_REGS->ADC_INTFLAG & (1<<0)) >> 0) != 0U);
    if (status == true) {
        ADC_REGS->ADC_INTFLAG = (1<<0);
    }
    return status;
}

float calculate_dust_density(uint16_t adc_value) {
    float voltage = (adc_value / ADC_RESOLUTION) * ADC_VREF;
    float delta_v = voltage - CLEAN_AIR_VOLTAGE;
    if (delta_v < 0.0f) delta_v = 0.0f;
    return (delta_v / DUST_CONVERSION_FACTOR);
}

// --- Calibration Data ---
uint32_t raw_value, raw_press, raw_hum, raw_temp;
uint16_t dig_T1;
int16_t  dig_T2, dig_T3; //calibration for temperature measure which includes compensation of formula and temp corrections
uint16_t dig_P1;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; //calibration for pressure measurement
uint8_t  dig_H1, dig_H3;
int16_t  dig_H2, dig_H4, dig_H5; //humidity calibration
int8_t   dig_H6;
int32_t t_fine;

static void read_calibration_data(void) {
    uint8_t calib_data[26];
    BME280_ReadReg(BME280_REG_CALIB, calib_data, 26);

    dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dig_T3 = (calib_data[5] << 8) | calib_data[4];

    dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dig_P9 = (calib_data[23] << 8) | calib_data[22];

    dig_H1 = calib_data[25];
    BME280_ReadReg(0xE1, calib_data, 7);

    dig_H2 = (calib_data[1] << 8) | calib_data[0];
    dig_H3 = calib_data[2];
    dig_H4 = (calib_data[3] << 4) | (calib_data[4] & 0x0F);
    dig_H5 = (calib_data[5] << 4) | (calib_data[4] >> 4);
    dig_H6 = (int8_t)calib_data[6];
}

// --- Updated compensation formulas ---
static int32_t compensate_temperature(uint32_t raw_temp) {
    int32_t var1, var2, T;
    var1 = ((((int32_t)raw_temp >> 3) - ((int32_t)dig_T1 << 1)) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((int32_t)raw_temp >> 4) - ((int32_t)dig_T1)) *
             (((int32_t)raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) *
             ((int32_t)dig_T3) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8; // actual temperature in °C × 100
    return T;
}

static uint32_t compensate_pressure(uint32_t raw_value) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // avoid division by zero
    }

    p = 1048576 - raw_value;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    
    // Return pressure in Pa (divide by 256 to get Pa)
    return (uint32_t)(p >> 8); 
}

static uint32_t compensate_humidity(uint32_t raw_value) {
    int32_t v_x1;
    
    v_x1 = t_fine - 76800;
    v_x1 = (((((raw_value << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1)) +
              ((int32_t)16384)) >> 15) *
            (((((((v_x1 * ((int32_t)dig_H6)) >> 10) *
                (((v_x1 * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
              ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    
    // Return humidity in %RH (divide by 1024 to get %RH)
    return (uint32_t)(v_x1 >> 12);
}

// --- Port initialization for ADC ---
void PORT_INIT(void) {
    // PA02 = ANALOG INPUT FOR THE SENSOR
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[2] = 0x1U;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[1] |= 0x1U;
    
    // PA04 AS THE EXTERNAL REF VOLTAGE REF_VREFB
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[4] = 0x1U;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[2] = 0x1U;
    
    // PA15 AS THE OUTPUT PIN GPIO_PA15 (kept for potential future use)
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1<<15);
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[15] = 0x0U;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[7] = 0x0U;
}

// --- Clock initialization for ADC ---
void CLOCK_INIT(void) {
    // GCK0: Div factor 1 | Source Select 7 | Generic Clock Generator Enable
    GCLK_REGS->GCLK_GENCTRL[0] = (1<<16) | (7<<0) | (1<<8);
    while ((GCLK_REGS->GCLK_SYNCBUSY & (1<<2)) == 0);
    
    // ADC Bus Clock: Generic Clock Generator Value | Channel Enable
    GCLK_REGS->GCLK_PCHCTRL[28] = (0<<0) | (1<<6);
    while ((GCLK_REGS->GCLK_PCHCTRL[28] & (1<<6)) != (1<<6));
}

// --- BME Init ---
void bme280_init(void) {
    uint8_t id;
    if (BME280_ReadReg(BME280_REG_ID, &id, 1) && id == BME280_CHIP_ID) {
        // Configure with recommended settings
        BME280_WriteReg(BME280_REG_CTRL_HUMIDITY, 0x05);  // Humidity oversampling x16
        BME280_WriteReg(BME280_REG_CTRL_MEAS, 0x25);  // Temp/pressure oversampling x1, mode=forced (01)
        BME280_WriteReg(BME280_REG_CONFIG, 0xA0);  // Standby 1000ms, filter off
        read_calibration_data();
        usart_cdc_tx_blocking("BME280 initialized in forced mode\r\n");
    } else {
        usart_cdc_tx_blocking("BME280 not detected\r\n");
    }
}

// --- Read & convert with forced mode ---
void Read_All_Sensors(prog_state_t *ps) {
    // Trigger a new measurement in forced mode
    if (!BME280_WriteReg(BME280_REG_CTRL_MEAS, 0x25)) {  // Start forced measurement
        usart_cdc_tx_blocking("Failed to trigger measurement\r\n");
        return;
    }
    
    // Wait for measurement to complete (usually 10-12ms for 1x oversampling)
    uint8_t status;
    uint8_t retry_count = 0;
    do {
        delay_loop(15000);  // Delay about ~10-15ms
        
        if (!BME280_ReadReg(0xF3, &status, 1)) {
            if (++retry_count > 10) {
                usart_cdc_tx_blocking("Failed to read status register\r\n");
                return;
            }
            continue;
        }
        // Bit 3 (measuring) should be 0 when complete
    } while (status & 0x08);
    
    // Read all data registers at once (burst read)
    uint8_t raw[8];
    if (!BME280_ReadReg(BME280_REG_DATA, raw, 8)) {
        usart_cdc_tx_blocking("BME280 data read failed\r\n");
        return;
    }

    // Convert the raw data
    uint32_t adc_P = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4);
    uint32_t adc_T = ((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4);
    uint32_t adc_H = ((uint32_t)raw[6] << 8) | raw[7];

    // Use the updated compensation functions
    int32_t temp = compensate_temperature(adc_T);       // °C * 100
    uint32_t press = compensate_pressure(adc_P);        // Pa
    uint32_t hum = compensate_humidity(adc_H);          // %RH * 1024
    float altitude = 44330.0 * (1.0 - pow((press / 100.0) / 1013.25, 0.1903)); // computed using pressure
    
    //then read the adc dust sensor
    ADC_ConversionStart();
    while (!ADC_ConversionStatusGet());
    uint16_t dust_adc_value = ADC_ConversionGetRes();
    float dust_density = calculate_dust_density(dust_adc_value);
    
    // Determine dust level status and color
    const char *dust_status;
    const char *dust_color;
    
    if (dust_density > 0.5f) {
        dust_status = "HIGH DUST LEVEL - UNHEALTHY";
        dust_color = "\033[1;31m";  // Bright red
    } else if (dust_density > 0.2f) {
        dust_status = "MODERATE DUST LEVEL";
        dust_color = "\033[1;33m";  // Bright yellow
    } else {
        dust_status = "CLEAN AIR";
        dust_color = "\033[1;32m";  // Bright green
    }
    
    delay_loop(15000);


    // Format and print combined readings
    snprintf(ps->tx_buf, sizeof(ps->tx_buf),
        "\033[2J\033[H"  // Clear screen and move cursor to top-left
        "\033[1;36m=== ENVIRONMENTAL SENSOR READINGS ===\033[0m\r\n"
        "\033[1;35mTemperature:\033[0m %ld.%02ld °C\r\n"
        "\033[1;34mPressure:   \033[0m %.2f hPa\r\n"
        "\033[1;32mHumidity:   \033[0m %.2f %%\r\n"
        "\033[1;31mAltitude:   \033[0m %.2lf m\r\n"
        "\033[1;36m=====================================\033[0m\r\n"
        "%sDust Density:\033[0m %.2f mg/m³\r\n"
        "%sStatus: %s\033[0m\r\n"
        "\033[1;90mRaw ADC: %u\033[0m\r\n"
        "\033[1;36m=====================================\033[0m\r\n",
        temp/100, temp % 100,           // Temperature in °C
        press / 100.0,                  // Pressure in hPa
        hum / 1024.0,                   // Humidity in %
        altitude,                       // Altitude in meters
        dust_color,                     // Dust level color
        dust_density,                   // Dust density in mg/m³
        dust_color,                     // Status color
        dust_status,                    // Status text
        dust_adc_value                  // Raw ADC value for debugging
    );
    
    usart_cdc_tx_blocking(ps->tx_buf);
    delay_loop(30000);
}

// --- Reset I2C if needed ---
static void reset_i2c_if_needed(void) {
    // Check if I2C is in error state (implementation depends on your specific MCU)
    if (SERCOM2_REGS->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSERR_Msk) {
        // Reset the I2C module
        SERCOM2_REGS->I2CM.SERCOM_CTRLA &= ~SERCOM_I2CM_CTRLA_ENABLE_Msk;
        while (SERCOM2_REGS->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);
        
        // Re-initialize
        SERCOM2_I2C_Initialize();
        usart_cdc_tx_blocking("I2C bus reset performed\r\n");
        
        // Re-initialize BME280 after I2C reset
        bme280_init();
    }
}

// --- Setup ---
static void prog_setup(prog_state_t *ps) {
    memset(ps, 0, sizeof(*ps));
    platform_init(); // uart init
    // ADC INIT
    PORT_INIT();
    CLOCK_INIT();
    
    //I2C INIT
    SERCOM2_I2C_Initialize();
    bme280_init();
    
    //ADC DUST SENSOR
    ADC_INIT();
    ADC_ENABLE();

    usart_cdc_tx_blocking(banner_msg);
    usart_cdc_tx_blocking("Combined BME280 + Dust Sensor initialized\r\n");

    ps->rx_desc.buf = ps->rx_desc_buf;
    ps->rx_desc.max_len = sizeof(ps->rx_desc_buf);
    platform_usart_cdc_rx_async(&ps->rx_desc);
}

// --- Loop ---
static void prog_loop_one(prog_state_t *ps) {
    platform_do_loop_one();
    reset_i2c_if_needed();  // Check and reset I2C if needed
    Read_All_Sensors(ps);
    delay_loop(LOOP_DELAY);
}

int main(void) {
    prog_state_t ps; 
    prog_setup(&ps);
    delay_loop(LOOP_DELAY);

    while (1) {
        prog_loop_one(&ps);
    }

    return 1;
}
