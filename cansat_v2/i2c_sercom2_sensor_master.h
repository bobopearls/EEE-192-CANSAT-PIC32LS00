#include <xc.h>
#include <stdint.h>

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
                                       //101: 16? oversampling NOTE: Higher oversampling is more accurate, but uses more power
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

// Function prototypes (to be implemented in i2c_BME280_sensor.c)
uint8_t BME280_ReadChipID(void);
void    BME280_SoftReset(void);
void    BME280_Init(void);
void    BME280_ReadCalibData(BME280_CalibData* calib);
void    BME280_Configure(void);
void    BME280_ReadRawData(uint32_t* rawTemp, uint32_t* rawPress, uint32_t* rawHumid);
float   BME280_CompensateTemperature(int32_t rawTemp, BME280_CalibData* calib, int32_t* t_fine);
float   BME280_CompensatePressure(int32_t rawPress, BME280_CalibData* calib, int32_t t_fine);
float   BME280_CompensateHumidity(int32_t rawHumid, BME280_CalibData* calib, int32_t t_fine);
