// i2c master header file 
// purpose of this portion of the code:
// this is supposed to serve as a master file to communicate with the sensor
// so the master is the pic32, and the slave is the BME280 sensor

#include <xc.h>
typedef enum{
    ACK_THEN_STOP, //ack the stuff then stop
    ACK_THEN_REPEATED_START//repeated start
}StopCondition;//stop condition here

uint16_t SERCOM2_I2C_Write_Polled(uint8_t addr, uint8_t *buf, uint16_t len);
uint16_t SERCOM2_I2C_Read_Polled(uint8_t addr, uint8_t *buf, uint16_t len);
void SERCOM2_I2C_Init(void); // write the c file for this i2c master
