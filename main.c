// note that the i2c_host file uses sercom2, 4MHz, smart mode is enabled and is set to response ACK
// master baud rate: 0xE8
// enabled interrupts: ERROR, SB, MB but not the NVIC interrupt lines

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "platform.h"
//#include "i2c_host.c"

// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf (data sheet)
// https://community.bosch-sensortec.com/mems-sensors-forum-jrmujtaw/post/bme280-returns-reset-values-T3qlqm036zdor2R
// from chatgpt https://learn.sparkfun.com/tutorials/qwiic-atmospheric-sensor-bme280-hookup-guide/all?utm_source=chatgpt.com
 
// defining addresses (register compatibility)
// memory mapping is shown in the bme280 datasheet table 18:
#define BME280_ADDR              0x76  //default I2C address, low address, so when it is high it will be 0x77
#define BME280_REG_ID            0xD0  //stores the chip ID of the BME280. if we read from this register, the program will know that this is the BME280
#define BME280_CHIP_ID           0x60  //unique identifier for verification and error detection
                                       //will use this to compare later to verify if the correct id is placed
#define BME280_REG_RESET         0xE0  //allows for BME280 "soft reset" but only for the sensor. 
                                       //if we write 0xB6 to this register, it does the soft reset (accd. to Bosch mauf.)
#define BME280_REG_CTRL_HUMIDITY 0xF2  //regsiter controls the humidty measurement (also oversampling)
                                       //Bits [2:0] control oversampling:
                                       //000: Humidity measurement skipped
                                       //001: 1× oversampling (default)
                                       //010: 2× oversampling
                                       //011: 4× oversampling
                                       //100: 8× oversampling
                                       //101: 16× oversampling NOTE: Higher oversampling is more accurate, but uses more power
#define BME280_REG_CTRL_MEAS     0xF4  //temperature and pressure measurement (oversampling and power mode)
                                       //use 0x27 (00100111 binary), which means:
                                       //Temperature: 1× oversampling
                                       //Pressure: 1× oversampling
                                       //Mode: Normal mode
#define BME280_REG_CONFIG        0xF5  //configures stand-by time, filter settings, and enabling or disabling the SPI (we don't need because I2C!)
                                       //Bits [7:5]: Standby time in normal mode:
                                       //000: 0.5ms
                                       //001: 62.5ms
                                       //010: 125ms and so on
                                       //Bits [4:2]: Filter coefficient:
                                       //000: Filter off
                                       //001: 2× filter
                                       //010: 4× filter and so on
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

// structure to store data:
typedef struct {
    uint16_t T1;
    int16_t  T2, T3; //calibration for temperature measure which includes compensation of formula and temp corrections
    uint16_t P1;
    int16_t  P2, P3, P4, P5, P6, P7, P8, P9; //calibration for pressure measurement
    uint8_t  H1, H3;
    int16_t  H2, H4, H5; //humidity calibration
    int8_t   H6;
} BME280_CalibData;

static BME280_CalibData cal_data; //storing the parameters of the calibration to the BME280 Sensor
static int32_t t_fine; //placeholder for the temperature value computation

//function prototypes
uint8_t BME280_INIT(void);
void BME280_READ_ALL(float *temp, float *pres, float *hum); //we want to be able to modify the temp, pressure, and humidity by updating the float values
                                                                     //these are all used as "pointer parameters". this is modified in a single function now.
                                                                     //precise floating point values since we are displaying as Celcius, Pa, etc.
float BME280_ALT(float pressure); //barometric formula: altitude = 44330(1 - (P/Po)^0.19029495) where P is the measured pressure in hPa and then Po is 1013.25 hPa

//i2c functions
static uint8_t BME_WRITE_REG(uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg, val}; //buf[0] register addr. buf[1] data to write
    return SERCOM2_I2C_Write_Polled(BME280_ADDR, buf, 2) == 2; //writing now to the BME280_ADDR, 2 BYTES
}

//i2c read operations:
static uint8_t BME_READ_REG(uint8_t reg){ //reading a single register
    uint8_t val;
    SERCOM2_I2C_Write_Polled(BME280_ADDR, &reg, 1);
    SERCOM2_I2C_Read_Polled(BME280_ADDR, &val, 1); // reading and writing 1 byte from the register
    return val;
}

static void BME_READ_MUL_REG(uint8_t reg, uint8_t *buf, uint8_t len){ //reads a block of registers
    SERCOM2_I2C_Write_Polled(BME280_ADDR, &reg, 1); //write register addr.
    SERCOM2_I2C_Read_Polled(BME280_ADDR, buf, len); //reading len number of bytes
}

// initializing the sensor
uint8_t BME280_INIT(void){
    uint8_t id = BME_READ_REG(BME280_REG_ID);
    if(id != BME280_CHIP_ID) return 0; // assigning the id
    
    // reset
    BME_WRITE_REG(BME280_REG_RESET, 0xB6); // reset value 0xB6
    
    // delay for 10ms
    uint16_t i;
    for(i = 0; i < 5000; i++) asm("nop");
    
    // read the data of temp and pressure (registers 0x88 to 0x9F)
    uint8_t buf[24]; // 2 bytes per calibration parameter
    BME_READ_MUL_REG(BME280_REG_CALIB, buf, 24);
    
    cal_data.T1 = (buf[1] << 8) | buf[0]; // MSB | LSB
    cal_data.T2 = (buf[3] << 8) | buf[2];
    cal_data.T3 = (buf[5] << 8) | buf[4];
    cal_data.P1 = (buf[7] << 8) | buf[6];
    cal_data.P2 = (buf[9] << 8) | buf[8];
    cal_data.P3 = (buf[11] << 8) | buf[10];
    cal_data.P4 = (buf[13] << 8) | buf[12];
    cal_data.P5 = (buf[15] << 8) | buf[14];
    cal_data.P6 = (buf[17] << 8) | buf[16];
    cal_data.P7 = (buf[19] << 8) | buf[18];
    cal_data.P8 = (buf[21] << 8) | buf[20];
    cal_data.P9 = (buf[23] << 8) | buf[22];
    
    //humidity calibration
    cal_data.H1 = BME_READ_REG(BME280_REG_CALIB_H1);
    
    BME_READ_MUL_REG(BME280_REG_CALIB_H2, buf, 7);
    cal_data.H2 = (buf[1] << 8) | buf[0];
    cal_data.H3 = buf[2];
    cal_data.H4 = (buf[3] << 4) | (buf[4] & 0x0F);
    cal_data.H5 = (buf[5] << 4) | (buf[4] >> 4);
    cal_data.H6 = buf[6];
    
    /*
     * BME280 Calibration Data Memory Map
     * -----------------------------------
     * Temperature & Pressure Calibration Data (Registers 0x88 - 0x9F)
     * ---------------------------------------------------------------
     * | Addr  | Data (Raw) |   Meaning  |
     * |-------|------------|------------|
     * | 0x88  | buf[0]     | T1 (LSB)   |
     * | 0x89  | buf[1]     | T1 (MSB)   |
     * | 0x8A  | buf[2]     | T2 (LSB)   |
     * | 0x8B  | buf[3]     | T2 (MSB)   |
     * | 0x8C  | buf[4]     | T3 (LSB)   |
     * | 0x8D  | buf[5]     | T3 (MSB)   |
     * | 0x8E  | buf[6]     | P1 (LSB)   |
     * | 0x8F  | buf[7]     | P1 (MSB)   |
     * | 0x90  | buf[8]     | P2 (LSB)   |
     * | 0x91  | buf[9]     | P2 (MSB)   |
     * | 0x92  | buf[10]    | P3 (LSB)   |
     * | 0x93  | buf[11]    | P3 (MSB)   |
     * | 0x94  | buf[12]    | P4 (LSB)   |
     * | 0x95  | buf[13]    | P4 (MSB)   |
     * | 0x96  | buf[14]    | P5 (LSB)   |
     * | 0x97  | buf[15]    | P5 (MSB)   |
     * | 0x98  | buf[16]    | P6 (LSB)   |
     * | 0x99  | buf[17]    | P6 (MSB)   |
     * | 0x9A  | buf[18]    | P7 (LSB)   |
     * | 0x9B  | buf[19]    | P7 (MSB)   |
     * | 0x9C  | buf[20]    | P8 (LSB)   |
     * | 0x9D  | buf[21]    | P8 (MSB)   |
     * | 0x9E  | buf[22]    | P9 (LSB)   |
     * | 0x9F  | buf[23]    | P9 (MSB)   |
     *
     * Humidity Calibration Data (Registers 0xA1, 0xE1 - 0xE7)
     * ---------------------------------------------------------
     * | Addr  |  Data (Raw)|   Meaning                          |
     * |-------|------------|------------------------------------|
     * | 0xA1  | buf[0]     | H1 (8-bit)                         |
     * |-------|------------|------------------------------------|
     * | 0xE1  | buf[0]     | H2 (LSB) (signed 16-bit)           |
     * | 0xE2  | buf[1]     | H2 (MSB)                           |
     * | 0xE3  | buf[2]     | H3 (8-bit)                         |
     * | 0xE4  | buf[3]     | H4 upper (MSB 8 bits)              |
     * | 0xE5  | buf[4]     | H4 lower (4 bits) / H5 lower (4 bits) |
     * | 0xE6  | buf[5]     | H5 upper (MSB 8 bits)              |
     * | 0xE7  | buf[6]     | H6 (signed 8-bit)                  |
     *
     * Notes:
     * - All temperature and pressure values are stored in little-endian format.
     * - H4 and H5 require special bit manipulation:
     *   - H4 = (buf[3] << 4) | (buf[4] & 0x0F);
     *   - H5 = (buf[5] << 4) | (buf[4] >> 4);
     */
    
    BME_WRITE_REG(BME280_REG_CTRL_HUMIDITY, 0x01); // humidity oversampling 1x
    BME_WRITE_REG(BME280_REG_CTRL_MEAS, 0x27); // temp, pressure both 1x, normal mode
    BME_WRITE_REG(BME280_REG_CONFIG, 0x00); //0.5ms standby, filter off
    
    return 1;
}

void BME280_READ_ALL(float *temp, float *pres, float *hum){
    // formula from the BME280 datasheet section 4.2.3 - Compensation Formulas
    uint8_t data[8];
    int32_t raw_T, raw_P, raw_H; // raw values for temp, pres, and humidity
                                 // data sheet uses adc, but to lessen confusion, used raw
    
    BME_READ_MUL_REG(BME280_REG_DATA, data, 8); // read all data at the same time
    
    raw_T = ((uint32_t) data[3] << 12) | ((uint32_t) data[4] << 4) | (data[5] >> 4);
    raw_P = ((uint32_t) data[0] << 12) | ((uint32_t) data[1] << 4) | (data[2] >> 4);
    raw_H = ((uint32_t) data[6] << 8)  | data[7];
    
    /*
    Data format in the BME280 register map:

    | Addr  | Data      | Description     | Bits Used |
    |-------|-----------|------------------|-----------|
    | 0xF7  | data[0]   | Press MSB        | [19:12]   |
    | 0xF8  | data[1]   | Press LSB        | [11:4]    |
    | 0xF9  | data[2]   | Press XLSB       | [3:0]     |
    | 0xFA  | data[3]   | Temp MSB         | [19:12]   |
    | 0xFB  | data[4]   | Temp LSB         | [11:4]    |
    | 0xFC  | data[5]   | Temp XLSB        | [3:0]     |
    | 0xFD  | data[6]   | Humidity MSB     | [15:8]    |
    | 0xFE  | data[7]   | Humidity LSB     | [7:0]     |

    Data reconstruction:
    ----------------------------------------
    Pressure (20-bit): 
       adc_P = (MSB << 12) | (LSB << 4) | (XLSB >> 4)
               (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)

    Temperature (20-bit): 
       adc_T = (MSB << 12) | (LSB << 4) | (XLSB >> 4)
               (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

    Humidity (16-bit):
       adc_H = (MSB << 8) | LSB
               (data[6] << 8) | data[7]
    */
    
    int32_t var1, var2;
    var1 = ((((raw_T >> 3) - ((int32_t)cal_data.T1 << 1))) * ((int32_t)cal_data.T2)) >> 11;
    var2 = (((((raw_T >> 4) - ((int32_t)cal_data.T1)) * ((raw_T >> 4) - ((int32_t)cal_data.T1))) >> 12) * ((int32_t)cal_data.T3)) >> 14;
    t_fine = var1 + var2;
    
    *temp = (t_fine * 5 + 128) >> 8;
    *temp/= 100.0f; // convert to degrees celcius by dividing w/ 100
    
    // compensating pressure, note the data sheet is 64 bit version. adjusted for 32 bit pic32
    var1 = (((int32_t)t_fine) >> 1) - 64000; // older datasheets use 64000 in the compensation formula, bmp280 uses 128000 and it is out of range
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)cal_data.P6);
    var2 = var2 + ((var1 * ((int32_t)cal_data.P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)cal_data.P4) << 16);
    var1 = (((cal_data.P3 * (((var1 >> 1) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)cal_data.P2) *var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)cal_data.P1)) >> 15);
    
    if(var1 == 0){
        *pres = 0;
    } else{
        uint32_t p = (((uint32_t)(((int32_t)1048576) - raw_P) - (var2 >> 12))) * 3125;
        if(p < 0x80000000){
            p = (p << 1) / ((uint32_t)var1) * 2;
        } else{
            p = (p / (uint32_t)var1) * 2;
        }
        
        var1 = (((int32_t)cal_data.P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(p >> 2)) * ((int32_t)cal_data.P8)) >> 13;
        p = (uint32_t)((int32_t)p + ((var1 + var2 + cal_data.P7) >> 4));
        
        *pres = p /100.0f; // converted to hPa
    }
    
    // compensating humidity
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((raw_H << 14) - (((int32_t)cal_data.H4) << 20) - (((int32_t)cal_data.H5) * v_x1_u32r)) + 
              ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)cal_data.H6)) >> 10) * 
              (((v_x1_u32r * ((int32_t)cal_data.H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
              ((int32_t)cal_data.H2) + 8192) >> 14)); //oh my god
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)cal_data.H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    *hum = (v_x1_u32r >> 12) / 1024.0f;
}

float BME280_ALT(float pressure) {
    // Simplified altitude formula: altitude = 44330 * (1 - (P/P0)^(1/5.255))
    // where P0 is sea level pressure (101325 Pa)
    const float P0 = 101325.0f;
    return 44330.0f * (1.0f - pow(pressure / 10.0f / P0, 0.19029495f));
}

void UART_PRINTF(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    
    platform_usart_tx_bufdesc_t desc;
    desc.buf = buffer; 
    desc.len = strlen(buffer);
    
    platform_usart_cdc_tx_async(&desc, 1);
    
    while(platform_usart_cdc_tx_busy()){
        // do nothing
    }
}

void data_vis_send (float temperature, float pressure, float humidity, float altitude){
    UART_PRINTF("DV_data,%.2f,%.2f,%.2f,%.2f\r\n", temperature, pressure, humidity, altitude);
}

int main(void){
    platform_usart_init();
    if(!BME280_INIT()){ //initialize sensor
        UART_PRINTF("Failed to initialize BME280 sensor \r\n");
        while(1);
    }
    UART_PRINTF("BME280 initialized successfully \r\n");
    
    float temperature, pressure, humidity, altitude;
    while(1){
        BME280_READ_ALL(&temperature, &pressure, &humidity);
        altitude = BME280_ALT(pressure);
        
        UART_PRINTF("Temperature: %.2f C\r\n", temperature);
        UART_PRINTF("Pressure: %.2f hPa\r\n", pressure);
        UART_PRINTF("Humidity: %.2f %%\r\n", humidity);
        UART_PRINTF("Altitude: %.2f m\r\n", altitude);
        UART_PRINTF("------------------------\r\n");
        
        data_vis_send(temperature, pressure, humidity, altitude);
        
        // Delay between readings (simple delay loop)
        for(uint32_t i = 0; i < 1000000; i++) {
            asm("nop");
        }
    }
    return 0;
}