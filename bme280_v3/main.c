
// Common include for the XC32 compiler
#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>


#include "platform.h"
#include "compensation_formula.h"

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
        SERCOM2_I2C_Write_Polled(0x76, 0xF3, 1);         // Select status register
        SERCOM2_I2C_Read_Polled(0x76, &status, 1);       // Read status
        retries++;
        if (retries > 100) {  // Timeout after 100 attempts
            break;
        }
    } while (status & 0x08);  // Wait while measuring (bit 3 = 1)
}


void forced_mode(void){
    uint8_t ctrl_meas = (1 << 5) | (1 << 2) | 0x01;  // osrs_t=1, osrs_p=1, mode=1 (forced)
    uint8_t config_data[] = {0xF4, ctrl_meas};
    SERCOM2_I2C_Write_Polled(0x76, config_data, 2);
   
    delay_loop_ms(2);
   
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
    uint32_t temperature_raw;
    uint32_t pressure_raw;
    uint32_t humidity_raw;
    
    int32_t temperature;
    int32_t pressure;
    
    
    // Calibration data
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    
    uint8_t calib_data[24]; // For reading all calibration data at once

   
    // Read all calibration data (0x88-0x9F)
    uint8_t reg = 0x88;
    SERCOM2_I2C_Write_Polled(0x76, &reg, 1);
    delay_loop_ms(2);
    SERCOM2_I2C_Read_Polled(0x76, calib_data, 24);
    
      // Parse temperature calibration data
    dig_T1 = (uint16_t)(calib_data[0] | (calib_data[1] << 8));
    dig_T2 = (int16_t)(calib_data[2] | (calib_data[3] << 8) );
    dig_T3 = (int16_t)(calib_data[4] | (calib_data[5] << 8));
    
    delay_loop_ms(2);
    
    // Now parse the pressure calibration data
    uint16_t dig_P1 = (uint16_t)(calib_data[6] | (calib_data[7] << 8));
    int16_t dig_P2 = (int16_t)(calib_data[8] | (calib_data[9] << 8));
    int16_t dig_P3 = (int16_t)(calib_data[10] | (calib_data[11] << 8));
    int16_t dig_P4 = (int16_t)(calib_data[12] | (calib_data[13] << 8));
    int16_t dig_P5 = (int16_t)(calib_data[14] | (calib_data[15] << 8));
    int16_t dig_P6 = (int16_t)(calib_data[16] | (calib_data[17] << 8));
    int16_t dig_P7 = (int16_t)(calib_data[18] | (calib_data[19] << 8));
    int16_t dig_P8 = (int16_t)(calib_data[20] | (calib_data[21] << 8));
    int16_t dig_P9 = (int16_t)(calib_data[22] | (calib_data[23] << 8));
    
    delay_loop_ms(2);
  
    uint8_t raw_data[8];
    bool read_success = true;

    // Clear buffers
    //memset(temp_data, 0, sizeof(temp_data));
    //memset(press_data, 0, sizeof(press_data));
    //memset(hum_data, 0, sizeof(hum_data));
    
    uint8_t hum_ctrl[2] = {0xF2, 0x01}; 
    SERCOM2_I2C_Write_Polled(0x76, hum_ctrl, 2);
    delay_loop_ms(2);
    
    forced_mode();
    
     wait_for_measurement();
    // Read Temperature (20-bit)
    SERCOM2_I2C_Write_Polled(0x76, 0xF7, 1);
    delay_loop_ms(2);
    SERCOM2_I2C_Read_Polled(0x76, raw_data, 8);
    
    delay_loop_ms(2);
    
    //adc_T = (uint32_t)((temp_raw[0] << 12) | (temp_raw[1] << 4) | (temp_raw[2] >> 4));

    pressure_raw = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);
    temperature_raw  = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
    humidity_raw   = (raw_data[6] << 8)  | raw_data[7];
    
    delay_loop_ms(2);
    
    //temperature_raw = 12312;
    temperature = compensate_temperature(temperature_raw, dig_T1, dig_T2, dig_T3);
    pressure = compensate_pressure(pressure_raw, dig_P1, dig_P2, dig_P3,
                            dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
    
    // Prepare output buffers only if read succeeded
    delay_loop_ms(2);
    char temp_buffer[32];
    char press_buffer[32];
    char hum_buffer[32];
    char alt_buffer[32];

    snprintf(temp_buffer, sizeof(temp_buffer), "Temperature: %ld.%02ld °C", temperature / 100, (temperature % 100));
    snprintf(press_buffer, sizeof(press_buffer), "Pressure:    %ld.%02ld hPa", pressure / 100, (pressure % 100));
    //snprintf(hum_buffer, sizeof(hum_buffer),    "Humidity:    %ld.%02ld %%", humidity / 1024, (humidity * 100 / 1024) % 100);
    //snprintf(alt_buffer, sizeof(alt_buffer),    "Altitude:    %.2f m", altitude);
    char debug[256];
    snprintf(debug, sizeof(debug),
        "\n------------------- DEBUG INFO -------------------\n"
        "Raw Temp:     %lu\n"
        "Raw Pressure: %lu\n"
        "Raw Humidity: %lu\n"
        "temp:       %ld\n\n"
        "dig_T1: %u\n"
        "dig_T2: %d\n"
        "dig_T3: %d\n"
        "...\n"
        "--------------------------------------------------\n",
        temperature_raw, pressure_raw, humidity_raw, temperature,
        dig_T1, dig_T2, dig_T3);
    
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
            ps->tx_desc[0].len = strlen(ESC_SEQ_KEYP_LINE);

            ps->tx_desc[1].buf = temp_buffer;
            ps->tx_desc[1].len = strlen(temp_buffer);

            ps->tx_desc[2].buf = "\n";
            ps->tx_desc[2].len = 1;

            ps->tx_desc[3].buf = press_buffer;
            ps->tx_desc[3].len = strlen(press_buffer);

            ps->tx_desc[4].buf = "\n";
            ps->tx_desc[4].len = 1;

            ps->tx_desc[5].buf = hum_buffer;
            ps->tx_desc[5].len = strlen(hum_buffer);

            ps->tx_desc[6].buf = "\n";
            ps->tx_desc[6].len = 1;

            ps->tx_desc[7].buf = alt_buffer;
            ps->tx_desc[7].len = strlen(alt_buffer);

            ps->tx_desc[8].buf = debug;
            ps->tx_desc[8].len = strlen(debug);
            
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
