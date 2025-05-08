
// Common include for the XC32 compiler
#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>


#include "platform.h"
//#include "compensation_formula.h" no need to since its already in the compensation_formula.c file

int start=0;

static const char banner_msg[] =
"\033[0m\033[2J\033[1;1H"
"Temperature:[NO DATA] \r\n"
"Altitude: [NO DATA]\r\n"
"Humidity: [NO DATA]";

static const char ESC_SEQ_KEYP_LINE[] = "\033[1;1H\033[0K "; //11th Row, 39th Column
static const char ESC_SEQ_PRESSURE[] = "\033[2;1H\033[0K ";
static const char ESC_SEQ_HUMIDITY[] = "\033[3;1H\033[0K";
static const char ESC_SEQ_IDLE_INF[]  = "\033[4;1H";

typedef struct prog_state_type
{
    // Flags for this program
#define PROG_FLAG_BANNER_PENDING    0x0001  // Waiting to transmit the banner
#define PROG_FLAG_UPDATE_PENDING    0x0002  // Waiting to transmit updates
#define PROG_FLAG_GEN_COMPLETE      0x8000  // Message generation has been done, but transmission has not occurred
    uint16_t flags;
    uint8_t lc;
    
    // Transmit stuff
    platform_usart_tx_bufdesc_t tx_desc[10];
    char tx_buf[64];
    uint16_t tx_blen;
    
    // Receiver stuff
    platform_usart_rx_async_desc_t rx_desc;
    uint16_t rx_desc_blen;
    char rx_desc_buf[16];
} prog_state_t;

// Struct to hold all BME280 calibration data
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data_t;

static void prog_setup(prog_state_t *ps)
{
    memset(ps, 0, sizeof(*ps));
    platform_init();
    ps->rx_desc.buf     = ps->rx_desc_buf;
    ps->rx_desc.max_len = sizeof(ps->rx_desc_buf);
    ps->lc = 0;
    platform_usart_cdc_rx_async(&ps->rx_desc);
    return;
} 

void wait_for_measurement(void) {
    uint8_t status = 0;
    uint8_t retries = 0;
    do {
        SERCOM2_I2C_Write_Polled(0x77, 0xF3, 1);         // Select status register
        SERCOM2_I2C_Read_Polled(0x77, &status, 1);       // Read status
        retries++;
        if (retries > 100) {  // Timeout after 100 attempts
            break;
        }
    } while (status & 0x08);  // Wait while measuring (bit 3 = 1)
}


void forced_mode(void){
    uint8_t ctrl_meas = (1 << 5) | (1 << 2) | 0x01;  // osrs_t=1, osrs_p=1, mode=1 (forced)
    uint8_t config_data[] = {0xF4, ctrl_meas};
    SERCOM2_I2C_Write_Polled(0x77, config_data, 2);
   
    
   
}

void delay_loop_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 4000; j++) {
            // Do nothing, just waste cycles (adjust 4000 based on clock speed)
        }
    }
}


static void prog_loop_one(prog_state_t *ps)
{   
    uint32_t temperature_raw = 0;
    uint32_t pressure_raw = 0;
    uint32_t humidity_raw = 0;
    
    int32_t temperature = 0;
    int32_t pressure = 0;
    uint32_t humidity = 0;
    
    // Calibration data
    uint16_t dig_T1 = 0;
    int16_t  dig_T2 = 0;
    int16_t  dig_T3 = 0;
    uint16_t dig_P1 = 0;
    int16_t  dig_P2 = 0;
    int16_t  dig_P3= 0;
    int16_t  dig_P4 = 0;
    int16_t  dig_P5 = 0;
    int16_t  dig_P6 = 0;
    int16_t  dig_P7 = 0;
    int16_t  dig_P8 = 0;
    int16_t  dig_P9 = 0;
    uint8_t  dig_H1 = 0;
    int16_t  dig_H2 = 0;
    uint8_t  dig_H3 = 0;
    int16_t  dig_H4 = 0;
    int16_t  dig_H5 = 0;
    int8_t   dig_H6 = 0;
    
    
    uint8_t calib_data[24]; // For reading all calibration data at once

   
    // Read all calibration data (0x88-0x9F)
    // Note that the Chip ID addr is in 0x76 (low. high is 0x77)
    SERCOM2_I2C_Write_Polled(0x76, 0x88, 1);
    SERCOM2_I2C_Read_Polled(0x76, calib_data, 24);
    
      // Parse temperature calibration data
    dig_T1 = (uint16_t)(calib_data[0]| (calib_data[1] << 8) );
    dig_T2 = (int16_t)(calib_data[2] | (calib_data[3] << 8) );
    dig_T3 = (int16_t)(calib_data[4] | (calib_data[5] << 8) );  
    
      /* Parse pressure calibration data
    dig_P1 = (uint16_t)(calib_data[6] | (calib_data[7] << 8) );
    dig_P2 = (int16_t)(calib_data[8]  | (calib_data[9] << 8));
    dig_P3 = (int16_t)(calib_data[10] | (calib_data[11] << 8));
    dig_P4 = (int16_t)(calib_data[12] | (calib_data[13] << 8));
    dig_P5 = (int16_t)(calib_data[14] | (calib_data[15] << 8));
    dig_P6 = (int16_t)(calib_data[16] | (calib_data[17] << 8));
    dig_P7 = (int16_t)(calib_data[18] | (calib_data[19] << 8));
    dig_P8 = (int16_t)(calib_data[20] | (calib_data[21] << 8));
    dig_P9 = (int16_t)(calib_data[22] | (calib_data[23] << 8));
    
    // Humidity calibration
    dig_H1 = calib_data[24];
    dig_H2 = (int16_t)(calib_data[25] | (calib_data[26] << 8));
    dig_H3 = calib_data[27];
    dig_H4 = (int16_t)(calib_data[28] | (calib_data[29] << 4));
    dig_H5 = (int16_t)(calib_data[30] | (calib_data[31] << 4));
    dig_H6 = calib_data[32];
    */
    
  
    uint8_t raw_data[8];
    bool read_success = true;

    // Clear buffers
    //memset(temp_data, 0, sizeof(temp_data));
    //memset(press_data, 0, sizeof(press_data));
    //memset(hum_data, 0, sizeof(hum_data));
    
    uint8_t hum_ctrl[2] = {0xF2, 0x01}; 
    SERCOM2_I2C_Write_Polled(0x77, hum_ctrl, 2);
    
    forced_mode();
    
     wait_for_measurement();
    // Read Temperature (20-bit)
    SERCOM2_I2C_Write_Polled(0x76, 0xF7, 1);
    SERCOM2_I2C_Read_Polled(0x76, raw_data, 8);
    
    pressure_raw = ((uint32_t)raw_data[0] << 12) | ((uint32_t)raw_data[1] << 4) | ((uint32_t)raw_data[2] >> 4);
    temperature_raw  = ((uint32_t)raw_data[3] << 12) | ((uint32_t)raw_data[4] << 4) | ((uint32_t)raw_data[5] >> 4);
    humidity_raw   = ((uint32_t)raw_data[6] << 8)  | (uint32_t)raw_data[7];
     
    temperature = compensate_temperature(temperature_raw, dig_T1, dig_T2, dig_T3);
    //pressure = compensate_pressure(pressure_raw, dig_P1, dig_P2, dig_P3, dig_P4,
            //dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
    //humidity = compensate_humidity(humidity_raw, dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6);
    // Prepare output buffers only if read succeeded
    char temp_buffer[16] = "NO DATA";
    char hum_buffer[16] = "NO DATA";
    char press_buffer[16] = "NO DATA";
    
   
    
    snprintf(temp_buffer, sizeof(temp_buffer), "%lu", temperature_raw);
   
    snprintf(press_buffer, sizeof(press_buffer), "%lu", pressure_raw);
        
    char debug[64];
snprintf(debug, sizeof(debug), "%d %d %d | %02X %02X %02X | %02X %02X",
         temperature/1000 , temperature % 100, temperature,
         raw_data[5], raw_data[5], raw_data[5],
         raw_data[6], raw_data[7]);

uint8_t chip_id;

SERCOM2_I2C_Write_Polled(0x76, 0xD0, 1);  // Send chip ID register address
SERCOM2_I2C_Read_Polled(0x76, &chip_id, 1);  // Read chip ID
printf("Chip ID: %02X\n", chip_id);  // Should print 0x60 for BME280

 snprintf(hum_buffer, sizeof(hum_buffer), "%02X\n", chip_id);
// Send debug over UART


    // Do one iteration of the platform event loop first.
    platform_do_loop_one();
    
    if(start==0){
        ps->flags |= PROG_FLAG_BANNER_PENDING;
        start=1;
    }

    // Something from the UART?
    if (ps->rx_desc.compl_type == PLATFORM_USART_RX_COMPL_DATA) {
        ps->flags |= PROG_FLAG_UPDATE_PENDING;
        ps->rx_desc_blen = ps->rx_desc.compl_info.data_len;
    }

    ////////////////////////////////////////////////////////////////////

    // Process any pending flags (BANNER)
    do {
        if ((ps->flags & PROG_FLAG_BANNER_PENDING) == 0)
            break;
        if (platform_usart_cdc_tx_busy())
            break;
        
        if ((ps->flags & PROG_FLAG_GEN_COMPLETE) == 0) {
            
            ps->tx_desc[0].buf = ESC_SEQ_KEYP_LINE;
            ps->tx_desc[0].len = sizeof(ESC_SEQ_KEYP_LINE)-1;
            
            ps->tx_desc[1].buf = "Temperature: ";
            ps->tx_desc[1].len = 13;
            
            
            ps->tx_desc[2].buf = debug;
            ps->tx_desc[2].len = strlen(debug); 

            
            ps->tx_desc[3].buf = ESC_SEQ_PRESSURE;
            ps->tx_desc[3].len = sizeof(ESC_SEQ_PRESSURE)-1;
            
            ps->tx_desc[4].buf = "Pressure: ";
            ps->tx_desc[4].len = 9;
            
            ps->tx_desc[5].buf = press_buffer;
            ps->tx_desc[5].len = strlen(press_buffer);
            
            ps->tx_desc[6].buf = ESC_SEQ_HUMIDITY;
            ps->tx_desc[6].len = sizeof(ESC_SEQ_HUMIDITY)-1;
            
            ps->tx_desc[7].buf = "Humidity: ";
            ps->tx_desc[7].len = 9;
            
            ps->tx_desc[8].buf = hum_buffer;
            ps->tx_desc[8].len = strlen(hum_buffer);
            
            ps->tx_desc[9].buf = ESC_SEQ_IDLE_INF;
            ps->tx_desc[9].len = sizeof(ESC_SEQ_IDLE_INF)-1;
        
            ps->flags |= PROG_FLAG_GEN_COMPLETE;
            
            
        }
        
       
        
        
        ps->flags |= PROG_FLAG_GEN_COMPLETE;
        ps->rx_desc_blen = 0;
        
        if (platform_usart_cdc_tx_async(&ps->tx_desc[0], 10)) {
            ps->rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE;
            platform_usart_cdc_rx_async(&ps->rx_desc);
            ps->flags &= ~(PROG_FLAG_UPDATE_PENDING | PROG_FLAG_GEN_COMPLETE);
        }
       
    } while (0);

    // Process any pending flags (UPDATE)
    do {
        if ((ps->flags & PROG_FLAG_UPDATE_PENDING) == 0)
            break;
        if (platform_usart_cdc_tx_busy())
            break;
        
        static int count = 0;
       

        
        count = 0;
        
    } while (0);
}

// main() -- the heart of the program
int main(void)
{
    prog_state_t ps;
    prog_setup(&ps);
    
    SERCOM2_I2C_Initialize();
    
    while(1) {
        prog_loop_one(&ps);
        
  
    }
    return 0;
}
