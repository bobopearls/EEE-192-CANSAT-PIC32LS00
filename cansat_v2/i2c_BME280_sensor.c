//#include "i2c_sercom2_sensor_master.h"
//#include "i2c_sercom2_sensor_master.c"
//#include "i2c_BME280_sensor.h"
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "i2c_BME280_sensor.h"
#include "i2c_sercom2_sensor_master.h"

static BME280_CalibData calib_data; // sensor calibration data from the h file
// data here will be used to calculate the actual temp, pressure, and humidity

/**
 * @brief Reads the chip ID from the BME280 sensor to verify that the device is connected and correct.
 * @param chip_id Pointer to store the chip ID
 * @return true if the read operation is successful, false if any issue occurs
 */
uint8_t BME280_ReadChipID(void){
    uint8_t chip_ID; // read the chip id from the sensor which should return 0x60 for the sensor
    SERCOM2_I2C_ReadReg(BME280_ADDR, BME280_REG_ID, &chip_ID, 1);
    return chip_ID;
}
// SOFT RESET
void BME280_SoftReset(void){
    uint8_t resetValue = 0xB6; // writing this value to the reset register
    SERCOM2_I2C_Write_Polled_Init(BME280_ADDR, BME280_REG_RESET, &resetValue, 1); // 1 is the number of bytes we are writing to the register
    // address of the sensor, then the register addr we want to write to, then the data byte we want to register
    __delay_ms(100); // wait for the reset to complete
}

// INITIALIZE
#define MAX_RETRIES 6
static bool bme280_ok = false; // need this for checking if the bme280 successfully read the chip id properly
void BME280_Init(void){
    uint8_t chip_ID;
    int retries = 0;
    do{
        chip_ID = BME280_ReadChipID();
        if(chip_ID == BME280_CHIP_ID){
            bme280_ok = true; // means the sensor is working since we read the correct chip ID
            break; // if the chip ID that was read is the same with the one in the register, then we can proceed
        }
        
        BME280_SoftReset(); // soft reset if its stuck
                
        __delay_ms(100); // delay for it to process
        retries++; // increment until we get the correct ID or else, it will stop until we reach max retries
    } while(retries < MAX_RETRIES);
    
    // final status check
    if(chip_ID != BME280_CHIP_ID){
        bme280_ok = false; // really doesnt work even after we reached the max retries
        // worst case scenario, it will re-initialize again IN THE MAIN LOOP
    }
}

void BME280_Configure(void){
    uint8_t configData[2];
    
    // 1x oversampling for humidity (check the github notes or h file if you want to change the sampling)
    configData[0] = BME280_REG_CTRL_HUMIDITY; 
    configData[1] = 0x01; // this is 1x oversampling
    SERCOM2_I2C_Write_Polled(BME280_ADDR, configData, 2);
    
    configData[0] = BME280_REG_CTRL_MEAS;
    configData[1] = 0x27; // check the h file notes. this is for 1x oversampling still
    SERCOM2_I2C_Write_Polled(BME280_ADDR, configData, 2);
    
    configData[0] = BME280_REG_CONFIG;
    configData[1] = 0x00;
    SERCOM2_I2C_Write_Polled(BME280_ADDR, configData, 2);
}

// THE FF CALCULATIONS ARE NOW BASED ON THE BME280 SENSOR'S DATA SHEET
void BME280_ReadRawData(uint32_t* rawTemp, uint32_t* rawPress, uint32_t* rawHumid){
    uint8_t data[8];
    
    // read raw temp pressure and humidity (3 vytes each)
    SERCOM2_I2C_ReadReg(BME280_ADDR, BME280_REG_CALIB, data, 8); 
    
    // combine to form 32 bit raw values
    *rawTemp = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)(data[5] >> 4)); // temp msb to temp lsb to temp xlsb
    *rawPress = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)(data[2] >> 4));
    *rawHumid = ((uint32_t)data[6] << 8) | (uint32_t)data[7];
}

void BME280_ReadCalibData(BME280_CalibData* calib){
    // read and store the calibrations for the data
    uint8_t calibData[24];
    
    // read data from the sensor
    SERCOM2_I2C_ReadReg(BME280_ADDR, BME280_REG_CALIB, calibData, 24);
    
    // temp calib NOTE THAT THE BME280 SENSOR USES LITTLE ENDIAN (LSB THEN MSB)
    // chapter 4 says that the data readout is from the largest reg number and then the smallest
    // the largest number register has 0 (the lsb), so it is correc that it is little endian
    calib->T1 = (uint16_t)(calibData[0] | (calibData[1] << 8));
    calib->T2 = (int16_t)(calibData[2] | (calibData[3] << 8));
    calib->T3 = (int16_t)(calibData[4] | (calibData[5] << 8));
    
     // Pressure calibration
    calib->P1 = (uint16_t)(calibData[6] | (calibData[7] << 8));
    calib->P2 = (int16_t)(calibData[8] | (calibData[9] << 8));
    calib->P3 = (int16_t)(calibData[10] | (calibData[11] << 8));
    calib->P4 = (int16_t)(calibData[12] | (calibData[13] << 8));
    calib->P5 = (int16_t)(calibData[14] | (calibData[15] << 8));
    calib->P6 = (int16_t)(calibData[16] | (calibData[17] << 8));
    calib->P7 = (int16_t)(calibData[18] | (calibData[19] << 8));
    calib->P8 = (int16_t)(calibData[20] | (calibData[21] << 8));
    calib->P9 = (int16_t)(calibData[22] | (calibData[23] << 8));
    
    // Humidity calibration
    calib->H1 = calibData[24];
    calib->H2 = (int16_t)(calibData[25] | (calibData[26] << 8));
    calib->H3 = calibData[27];
    calib->H4 = (int16_t)(calibData[28] | (calibData[29] << 4));
    calib->H5 = (int16_t)(calibData[30] | (calibData[31] << 4));
    calib->H6 = calibData[32];
}

float BME280_CompensateTemperature(int32_t rawTemp, BME280_CalibData* calib, int32_t* t_fine) {
    int32_t var1, var2;
    var1 = (((rawTemp >> 3) - ((int32_t)calib->T1 << 1)) * ((int32_t)calib->T2)) >> 11;
    var2 = (((((rawTemp >> 4) - ((int32_t)calib->T1)) * ((rawTemp >> 4) - ((int32_t)calib->T1))) >> 12) * ((int32_t)calib->T3)) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;  // Convert to temperature in Celsius
}

// Function to compensate pressure based on raw data, temperature compensation, and calibration
float BME280_CompensatePressure(int32_t rawPress, BME280_CalibData* calib, int32_t t_fine) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->P6;
    var2 = var2 + ((var1 * (int64_t)calib->P5) << 17);
    var2 = var2 + (((int64_t)calib->P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->P3) >> 8) + ((var1 * (int64_t)calib->P2) << 12);
    var1 = ((int64_t)1 << 47) + var1;
    p = ((int64_t)var1 * (int64_t)calib->P1) >> 33;
    return (float)p / 256.0f; // Pressure in Pascals
}

// Function to compensate humidity based on raw data, temperature compensation, and calibration
float BME280_CompensateHumidity(int32_t rawHumid, BME280_CalibData* calib, int32_t t_fine) {
    int32_t var1;
    var1 = t_fine - 76800;
    var1 = (((((rawHumid << 14) - ((int32_t)calib->H4 << 20) - ((int32_t)calib->H5 * var1)) + 16384) >> 15) * (((((((var1 * (int32_t)calib->H6) >> 10) * (((var1 * (int32_t)calib->H3) >> 11) + 32768)) >> 10) + 2097152) * (int32_t)calib->H2 + 8192) >> 14));
    return var1 / 1024.0f;  // Humidity in percentage
}
