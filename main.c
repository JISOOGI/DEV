/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
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
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "WT931.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define LM75B_ADDR          (0x90U >> 1)

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

/* Mode for LM75B. */
#define NORMAL_MODE 0U

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

static ret_code_t twi_rx(uint8_t address, uint8_t resister, uint8_t *p_data, uint8_t length)
{
    ret_code_t err_code;

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, address, &resister, sizeof(resister), true);
    APP_ERROR_CHECK(err_code);

    while (m_xfer_done == false);

    if (err_code == NRF_SUCCESS)
    {
        m_xfer_done = false;
        err_code = nrf_drv_twi_rx(&m_twi, address, p_data, length);
        APP_ERROR_CHECK(err_code);
        
        while (m_xfer_done == false);
    }

    return err_code;
}

static ret_code_t twi_tx(uint8_t address, uint8_t resister, uint8_t p_data, uint8_t length)
{
    ret_code_t err_code;
    uint8_t buffer[2];
    
    buffer[0] = resister;
    buffer[1] = p_data;

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, address, buffer, length, true);
    APP_ERROR_CHECK(err_code);

    while (m_xfer_done == false);

    return err_code;
}

static void wt931_init(void)
{
    ret_code_t err_code;
    uint8_t value;
    
    err_code = twi_rx(WT931_ADDR, IICADDR, &value, sizeof(value));
    APP_ERROR_CHECK(err_code);

    if (value == WT931_ADDR)
    {
        NRF_LOG_INFO("WT931 init success, id=0x%02x", value);
        nrf_delay_ms(100);
    }
    else
    {
        NRF_LOG_INFO("WT931 init fail, value=0x%02x", value);
    }

    NRF_LOG_FLUSH();
}

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void LM75B_set_mode(void)
{
    ret_code_t err_code;

    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to pointer byte. */
    reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    //NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_wt931_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_wt931_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
//static void read_sensor_data()
//{
//    m_xfer_done = false;

//    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
//    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
//    APP_ERROR_CHECK(err_code);
//}
static void read_sensor_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, WT931_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}


//Roll(X axis)Roll=((RollH<<8)|RollL)/32768*180(°)
//(float)JY901.stcAngle.Angle[0]/32768*180
//Pitch(Y axis)Pitch=((PitchH<<8)|PitchL)/32768*180(°)
//Yaw(Z axis)Yaw=((YawH<<8)|YawL)/32768*180(°)

void wt931_config(void){
    ret_code_t err_code;
    //uint8_t data[6];
    err_code = twi_rx(WT931_ADDR, Roll, &stcAngle, sizeof(stcAngle));
    APP_ERROR_CHECK(err_code);
    
    float roll = (float)(stcAngle.Angle[0])/32768*180;
    float pitch = (float)(stcAngle.Angle[1])/32768*180;
    float yaw = (float)(stcAngle.Angle[2])/32768*180;

    //float roll = (float)(stcAngle.Angle[1]<<8|stcAngle.Angle[0])/32768*180;
    //float pitch = (float)(stcAngle.Angle[3]<<8|stcAngle.Angle[2])/32768*180;
    //float yaw = (float)(stcAngle.Angle[5]<<8|stcAngle.Angle[4])/32768*180;

    NRF_LOG_INFO("Roll=%d Pitch=%d Yaw=%d", roll, pitch, yaw);
    //NRF_LOG_INFO("data=%d", data);
    NRF_LOG_FLUSH();
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    wt931_init();
    //wt931_config();    
    while (true)
    {
        nrf_delay_ms(500);

        do
        {
            __WFE();
        }while (m_xfer_done == false);

        //read_sensor_data();
        wt931_config();
        NRF_LOG_FLUSH();
    }
}



void getAngle(void){
    int32_t roll = 180 * (int32_t)stcAngle.Angle[0] / 32768;
    int32_t pitch = 180 * (int32_t)stcAngle.Angle[1] / 32768;
    int32_t yaw = 180 * (int32_t)stcAngle.Angle[2] / 32768;
}
/** @} */
