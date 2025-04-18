#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
//#include "i2c_BME280_sensor.h"
#include "i2c_BME280_sensor.c"
//#include "i2c_sercom2_sensor_master.h"
#include "platform.h"

void __delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __asm__ __volatile__("nop");
    }
}


#define RETRY_DELAY_MS 5000 // equivalent to 5 seconds
static uint32_t lastRetry = 0;

int main(void) {
    
    platform_init(); // from the platform.h file
    SERCOM2_I2C_Init(); // from the i2c_sercom2_sensor_master.c file
    BME280_Init(); // from the i2c_BME280_sensor.c file
    
    while (1) {
        // this is the reset we will do in the worst case scenario for the re-initialization in the main loop
        if (!bme280_ok) {
            __delay_ms(RETRY_DELAY_MS);
            BME280_Init();
            continue; // Skip further processing until sensor is working
        }

    
        // sensor is supposed to be working at this pointy so start to read and display the data
        int32_t rawTemp, rawPress, rawHumid;
        BME280_ReadRawData(&rawTemp, &rawPress, &rawHumid);

        int32_t t_fine;
        float temperature = BME280_CompensateTemperature(rawTemp, &calib_data, &t_fine);
        float pressure    = BME280_CompensatePressure(rawPress, &calib_data, t_fine);
        float humidity    = BME280_CompensateHumidity(rawHumid, &calib_data, t_fine);

        // Send or log data, like via UART or SD card
        // printf or UART_Write formatted strings here

        __delay_ms(1000);  // Wait before next read
    }

    return 0;
}
