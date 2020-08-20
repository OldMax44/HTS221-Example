/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

//
// Modified twi_sensor example main.c to use a HTS221 temperature and humidity sensor
//
// References:
// HTS221 Datasheet, ST DocID026333 Rev 4, 30-Aug-2016
// ST TN1218 Rev 4, 27-Aug-2018
//

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// TWI instance ID.
#define TWI_INSTANCE_ID     0

// Common address definitions for the HTS221 temperature and humidity sensor
#define HTS221_ADDR         (0xBEU >> 1)

#define HTS221_WHO_AM_I         0x0FU
#define HTS221_AV_CONF          0x10U
#define HTS221_CTRL_REG1        0x20U
#define HTS221_CTRL_REG2        0x21U
#define HTS221_CTRL_REG3        0x23U
#define HTS221_STATUS_REG       0x27U
#define HTS221_HUMIDITY_OUT_L   0x28U
#define HTS221_HUMIDITY_OUT_H   0x29U
#define HTS221_TEMP_OUT_L       0x2AU
#define HTS221_TEMP_OUT_H       0x2BU
//
//  Temperature and Humidity averaging settings
//  in the HTS221_AV_CONF register
//
//  Bits 5:3 AVGT2-AVGT1-AVGT0: Select the temperature internal average.
//
//      AVGT2 | AVGT1 | AVGT0 | Nr. Internal Average
//   ----------------------------------------------------
//       0    |   0   |   0   |    2
//       0    |   0   |   1   |    4
//       0    |   1   |   0   |    8
//       0    |   1   |   1   |    16
//       1    |   0   |   0   |    32
//       1    |   0   |   1   |    64
//       1    |   1   |   0   |    128
//       1    |   1   |   1   |    256
//
// Bits 2:0 AVGH2-AVGH1-AVGH0: Select humidity internal average.
//      AVGH2 | AVGH1 |  AVGH0 | Nr. Internal Average
//   ------------------------------------------------------
//       0    |   0   |   0   |    4
//       0    |   0   |   1   |    8
//       0    |   1   |   0   |    16
//       0    |   1   |   1   |    32
//       1    |   0   |   0   |    64
//       1    |   0   |   1   |    128
//       1    |   1   |   0   |    256
//       1    |   1   |   1   |    512
//
#define AVG_MODE 0b00111111 // Set maximum averaging Temp 256, Humidity 512

typedef enum
{
    HTS221_ODR_ONESHOT,
    HTS221_ODR_1HZ,
    HTS221_ODR_7HZ,
    HTS221_ODR_12_5HZ,
} hts221_odr_t;

typedef struct
{
    uint8_t  H0_rH_x2;
    uint8_t  H1_rH_x2;
    uint16_t T0_degC_x8;
    uint16_t T1_degC_x8;
    int16_t  H0_T0_OUT;
    int16_t  H1_T0_OUT;
    int16_t  T0_OUT;
    int16_t  T1_OUT;
} hts221_calib_t;

// Indicates if operation on TWI has ended.
static volatile bool m_xfer_done = false;

// TWI instance.
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Buffer for 1 byte read from hts221 sensor.
static uint8_t m_sample;

// Transmit data array
// Only one byte which is the register address
uint8_t reg[1] = {0};
//
// Raw Temperature array
// 2 bytes read from registers 0x2A (LSB) and 0x2B (MSB)
//
uint8_t raw_temp[2] = {0};
//
// 16 bit variable for the temperature read
// Combines LSB from register 0x2A and MSB from register 0x2B
//
int16_t T_Out = 0;
//
// Calculated temperature variables
//
float tempFloat = 0.0;
float tempF = 0.0;
//
// Raw Temperature array
// 2 bytes read from registers 0x2A (LSB) and 0x2B (MSB)
//
uint8_t raw_hum[2] = {0};
//
// 16 bit variable for the humidity read
// Combines LSB from register 0x28 and MSB from register 0x29
//
int16_t H_Out = 0;
//
// Calculated humidity variable
//
float humFloat = 0.0;
//
// Status flags
//
bool temperature_ready_flag = false;
bool humidity_ready_flag = false;
//
// Array for the raw calibration data
//
uint8_t calibration_regs[16];
// Declare the stucture for the calibration constants
hts221_calib_t m_hts221_cal;
// Declare an ODR type
hts221_odr_t m_odr;
//
// Function for setting Output Data Rate of the HTS221 sensor
// Note: 1 Hz, 7 Hz, and 12.5 Hz modes also set the BDU bit for asynchronous reads of temperature
// and humidity and set active mode for continuous conversions.
// The default is One Shot Mode 
// 
void set_ODR(uint8_t m_odr)
{
    ret_code_t err_code;
    // Set Default to One Shot mode
    // Would need to set the One Shot Enable to start a temperature and Humidity conversion
    uint8_t reg1[2] = {HTS221_CTRL_REG1, 0x80}; // Active in One Shot mode

    if (m_odr == HTS221_ODR_1HZ)
    {
      reg1[1]=0x85; // Active, BDU, 1 Hz ODR
    }
    if (m_odr == HTS221_ODR_7HZ)
    {
      reg1[1]=0x86; // Active, BDU, 7 Hz ODR
    }
    if (m_odr == HTS221_ODR_12_5HZ)
    {
      reg1[1]=0x87; // Active, BDU, 12.5 Hz ODR
    }

    m_xfer_done = false;
    // Writing to CTRL_REG1 to set active and Output Data Rate
    err_code = nrf_drv_twi_tx(&m_twi, HTS221_ADDR, reg1, sizeof(reg1), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    
}

//
// Function for setting averaging mode of the HTS221 sensor
//
void HTS221_set_mode(void)
{
    ret_code_t err_code;
    m_xfer_done = false;

    // Writing to HTS221_AV_CONF to set sensor averaging configuration
    // 2 byte transmit: Register addresss and data to the register
    uint8_t regm[2] = {HTS221_AV_CONF, AVG_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, HTS221_ADDR, regm, sizeof(regm), false);
    APP_ERROR_CHECK(err_code);
    // Wait for TWI transmit to complete
    while (m_xfer_done == false);


}
//
// Function for handling data read from HTS221 sensor WHO_AM_I register.
//
__STATIC_INLINE void id_handler(uint8_t temp)
{
    NRF_LOG_INFO("\nWho I Am: %0X \n", temp);
}

//
// Function for handling data read from HTS221 sensor CTRL_REG1.
//
__STATIC_INLINE void reg1_handler(uint8_t temp)
{
    NRF_LOG_INFO("\nCTRL_REG1: %0X \n", temp);
}
//
// Function for handling data read from HTS221 sensor temperature registers.
//
__STATIC_INLINE void temperature_handler(uint8_t temp[2])
{   
    T_Out = raw_temp[0] | (raw_temp[1] << 8);
    NRF_LOG_INFO("\n RAW_TEMP: %0X \n", T_Out);
}
//
// Function for handling data read from HTS221 sensor humidity registers.
//
__STATIC_INLINE void humidity_handler(uint8_t temp[2])
{   
    H_Out = raw_hum[0] | (raw_hum[1] << 8);
    NRF_LOG_INFO("\n RAW_HUMIDITY: %04X \n", H_Out);
}
//
// Function for handling calibration data read from HTS221 sensor
// 16 bytes returned and calibration data set up in hts221_calib_t structure
// for temperature and humidity interpolation
//
__STATIC_INLINE void calibration_handler(uint8_t calib_raw[16])
{

    m_hts221_cal.H0_rH_x2 = calib_raw[0];
    m_hts221_cal.H1_rH_x2 = calib_raw[1];
    m_hts221_cal.T0_degC_x8 = (uint16_t)calib_raw[2] + ((uint16_t)(calib_raw[5] & 0x03) << 8);
    m_hts221_cal.T1_degC_x8 = (uint16_t)calib_raw[3] + ((uint16_t)((calib_raw[5] >> 2) & 0x03) << 8);
    m_hts221_cal.H0_T0_OUT = (int16_t)calib_raw[6]  + ((int16_t)calib_raw[7]  << 8);
    m_hts221_cal.H1_T0_OUT = (int16_t)calib_raw[10] + ((int16_t)calib_raw[11] << 8);
    m_hts221_cal.T0_OUT = (int16_t)calib_raw[12] + ((int16_t)calib_raw[13] << 8);
    m_hts221_cal.T1_OUT = (int16_t)calib_raw[14] + ((int16_t)calib_raw[15] << 8);
}
//
// Function for handling data read from HTS221 sensor status register.
//
__STATIC_INLINE void status_handler(uint8_t temp)
{
    NRF_LOG_INFO("\nSTATUS: %0X \n", temp);
    
    temperature_ready_flag = false;
    humidity_ready_flag = false;

    if((temp & 0x01) != 0x00)
      temperature_ready_flag = true;
    if((temp & 0x02) != 0x00)
      humidity_ready_flag = true;
}
//
// TWI events handler.
//
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        // TWI event completed
        // If data was being read, jump to the data handler
        // Otherwise, assume it was a transmit and just indicate transfer is done
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) // Reading data
            {
              if (reg[0] == HTS221_WHO_AM_I) // Reading from HTS221 WHO_AM_I register
                id_handler(m_sample);
              if (reg[0] == HTS221_CTRL_REG1) // Reading CTRL_REG1
                reg1_handler(m_sample);
              if (reg[0] == 0xB0) // Reading the calibration data
                calibration_handler(calibration_regs);
              if (reg[0] == 0xAA) // Reading the temperature registers
                temperature_handler(raw_temp);
              if (reg[0] == HTS221_STATUS_REG) // Reading the status register
                status_handler(m_sample);
              if (reg[0] == 0xA8)  // Reading the humidity registers
                humidity_handler(raw_hum);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

//
// TWI initialization.
//
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_hts221_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
 
    err_code = nrf_drv_twi_init(&m_twi, &twi_hts221_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

    //
    // read calibration registers
    //
    reg[0]=0xB0; // 1st Register address of the calibration registers
    // Note bit 7 is set for register address incrementing. 0x30 | 0x80
    err_code = nrf_drv_twi_tx(&m_twi, HTS221_ADDR, reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);
    // Wait for transmit to complete
    while (m_xfer_done == false);
    // Reset TWI event complete flag
    m_xfer_done = false;
    // Read 16 bytes of raw calibration data from 0x30-3F
    // Data will be returned via twi event handler
    err_code = nrf_drv_twi_rx(&m_twi, HTS221_ADDR, calibration_regs, sizeof(calibration_regs));
    APP_ERROR_CHECK(err_code);


}
//
// Funtion to read the temperature registers
// 2 bytes read from registers 0x2A (LSB) and 0x2B (MSB)
//
static void read_temp()
{
    ret_code_t err_code;

    // Reset TWI event complete flag
    m_xfer_done = false;
    // Write register address for temperature read
    reg[0]=0xAA; // 1st Register address of the temperature registers
    // Note bit 7 is set for register address incrementing. 0x2A | 0x80
    err_code = nrf_drv_twi_tx(&m_twi, HTS221_ADDR, reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);
    // Wait for transmit to complete
    while (m_xfer_done == false);
    // Reset TWI event complete flag
    m_xfer_done = false;
    // Read 2 bytes of raw temperature data from 0x2A-2B
    // Data will be returned via twi event handler
    err_code = nrf_drv_twi_rx(&m_twi, HTS221_ADDR, raw_temp, sizeof(raw_temp));
    APP_ERROR_CHECK(err_code);
}

//
// Function to interpolate temperature
//
static void calc_temp()
{

    //
    // Compute Temperature
    // Note: From ST document TN1218
    //
    // Temp = ((temp_T1_DegC - temp_T0_DegC) * (T_Out - T0_Out)/(T1_Out - T0_Out))+temp_T0_DegC;
    //
    //
    // Set up calibration data
    // Set as floating point for a floating point interpolation calculation
    // This also retains the fractional remainder from the divide by 8
    //
    float temp_T0_DegC = (float)m_hts221_cal.T0_degC_x8 / 8.0;
    float temp_T1_DegC = (float)m_hts221_cal.T1_degC_x8 / 8.0;
    float T0_Out = (float)m_hts221_cal.T0_OUT;
    float T1_Out = (float)m_hts221_cal.T1_OUT;
    //
    // Cast any remaining integers to floating point for the interpolation calculation
    //
     tempFloat = (((temp_T1_DegC - temp_T0_DegC) * ((float)T_Out - T0_Out))/(T1_Out - T0_Out)) + (temp_T0_DegC);
     tempF = (tempFloat * 9.0/5.0) + 32.0;
    //
    NRF_LOG_INFO("Computed Temperature = " NRF_LOG_FLOAT_MARKER " C ", NRF_LOG_FLOAT(tempFloat));
    NRF_LOG_INFO("Computed Temperature = " NRF_LOG_FLOAT_MARKER " F \r\n", NRF_LOG_FLOAT(tempF));
}

//
// Funtion to read the humidity registers
// 2 bytes read from registers 0x28 (LSB) and 0x29 (MSB)
//
static void read_hum()
{
    ret_code_t err_code;

    // Reset TWI event complete flag
    m_xfer_done = false;
    // Write register address for humidity read
    reg[0]=0xA8; // 1st Register address of the humidity registers
    // Note bit 7 is set for register address incrementing. 0x28 | 0x80
    err_code = nrf_drv_twi_tx(&m_twi, HTS221_ADDR, reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);
    // Wait for transmit to complete
    while (m_xfer_done == false);
    // Reset TWI event complete flag
    m_xfer_done = false;
    // Read 2 bytes of raw humidity data from 0x28-29
    // Data will be returned via twi event handler
    err_code = nrf_drv_twi_rx(&m_twi, HTS221_ADDR, raw_hum, sizeof(raw_hum));
    APP_ERROR_CHECK(err_code);
}

//
// Function to interpolate humidity
//
static void calc_hum()
{
     //
     // Compute humidity
     // Note: From ST document TN1218
     //
     //HRH = (H1_rH - H0_rH)*(H_Out - H0_T0_Out)/(H1_T0_Out - H0_T0_Out) + H0_rH;
     //
     //
     //
     // Set up calibration data
     // Note:  Use of signed integers for some of the constants according to the tables in TN1218 document
     // Set up as floating point variables for use in the floating point interpolation calculation
     // This also retains any fractional remainder from the dive by 2
     //
     float H1_rH = (float)m_hts221_cal.H1_rH_x2/2.0;
     float H0_rH = (float)m_hts221_cal.H0_rH_x2/2.0;
     float H0_T0_Out = (float)m_hts221_cal.H0_T0_OUT;
     float H1_T0_Out = (float)m_hts221_cal.H1_T0_OUT;
     //
     // Cast remaining integers to floating point for the interpolation calculation
     //
     humFloat = (((H1_rH - H0_rH)*((float)H_Out - H0_T0_Out))/(H1_T0_Out - H0_T0_Out)) + H0_rH;
    //
    NRF_LOG_INFO("Computed Humidity = " NRF_LOG_FLOAT_MARKER " % \r\n", NRF_LOG_FLOAT(humFloat));

}
//
// Function for reading the device id from the HTS221 sensor.
//
static void read_sensor_id()
{
    ret_code_t err_code;
      
    m_xfer_done = false;
    //
    // Send the register address
    // Write with no stop
    //
    reg[0] = HTS221_WHO_AM_I;
    err_code = nrf_drv_twi_tx(&m_twi, HTS221_ADDR, reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    // Wait for transmit to complete
    m_xfer_done = false;
    // Read 1 byte
    // Data will be returned to twi event handler
    err_code = nrf_drv_twi_rx(&m_twi, HTS221_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}

//
// Function for reading ctrl_reg1 from the HTS221 sensor.
//
static void read_ctrl_reg1()
{
    ret_code_t err_code;
      
    m_xfer_done = false;
    //
    // Send the register address
    // Write with no stop
    //
    reg[0] = HTS221_CTRL_REG1;
    err_code = nrf_drv_twi_tx(&m_twi, HTS221_ADDR, reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    // Wait for transmit to complete
    m_xfer_done = false;
    // Read 1 byte
    // Data will be returned to twi event handler
    err_code = nrf_drv_twi_rx(&m_twi, HTS221_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}
//
// Function for reading the status register from the HTS221 sensor.
//
static void read_stat()
{
    ret_code_t err_code;
      
    m_xfer_done = false;
    //
    // Send the register address
    // Write with no stop
    //
    reg[0] = HTS221_STATUS_REG;
    err_code = nrf_drv_twi_tx(&m_twi, HTS221_ADDR, reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    // Wait for transmit to complete
    m_xfer_done = false;
    // Read 1 byte
    // Data will be returned to twi event handler
    err_code = nrf_drv_twi_rx(&m_twi, HTS221_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}


//
// Main application entry.
// 
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    // Initial delay to set up the comm port
    nrf_delay_ms(1000);
    //
    NRF_LOG_INFO("\r\nTWI HTS221 sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    // Wait for all the calibration data back from the hts221
    // and stored for interpolation calculations
    do
    {
        __WFE();
    }while (m_xfer_done == false);
    // Set the temperature and humidity averaging
    HTS221_set_mode();
    // Read the HTS221 id
    read_sensor_id();
    // Wait for response back from sensor
    do
    {
        __WFE();
    }while (m_xfer_done == false);
    // Check the default value in ctrl_reg1
    // Note: On power up this reads zero
    // Otherwise you see the value previously written to the register
    read_ctrl_reg1();
    // Wait for response back from sensor
    do
    {
        __WFE();
    }while (m_xfer_done == false);
    //  Set Output Data Rate in Control Register 1
    set_ODR(HTS221_ODR_7HZ);
    // Check the new value in ctrl_reg1
    read_ctrl_reg1();
    // Wait for response back from sensor
    do
    {
        __WFE();
    }while (m_xfer_done == false);
    //
    while (true)
    {
        //
        // Check to see if temperature data is ready
        //
        read_stat();  
        //
        // Wait for response back from twi interface
        // that teperature data has been returned
        // by the HTS221 sensor
        //
        do
        {
            __WFE();
        }while (m_xfer_done == false);
        //
        // If temperature conversion is complete, read temperature
        // and run the interpolation calculation
        //
        if (temperature_ready_flag)
        {
        //
        // Read Temperature registers
        //
        read_temp();
        //
        // Wait for response back from twi interface
        // that teperature data has been returned
        // by the HTS221 sensor
        //
        do
        {
            __WFE();
        }while (m_xfer_done == false);
        //
        // Check to see that the temperature ready bit is cleared
        // by the temperature read
        //
        read_stat();  
        //
        // Wait for response back from twi interface
        // that teperature data has been returned
        // by the HTS221 sensor
        //
        do
        {
            __WFE();
        }while (m_xfer_done == false);
        //
        // Run interpolation calculation
        //
        calc_temp();
        // Transmit log messages to display results
        NRF_LOG_FLUSH();
        }

        if(humidity_ready_flag)
        {
        read_hum();
        //
        // Wait for response back from twi interface
        // that humidity data has been returned
        // by the HTS221 sensor
        //
        do
        {
            __WFE();
        }while (m_xfer_done == false);
        //
        // Check to see that the humidity ready bit is cleared
        // by the humidity read
        //
        read_stat();  
        //
        // Wait for response back from twi interface
        // that teperature data has been returned
        // by the HTS221 sensor
        //
        do
        {
            __WFE();
        }while (m_xfer_done == false);
        //
        // Run interpolation calculation
        //
        calc_hum();
        // Transmit log messages to display results
        NRF_LOG_FLUSH();
        }

    // Delay 10 seconds
    nrf_delay_ms(10000);

    }


}

