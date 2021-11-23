/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include "AD5933.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"

#include "app_timer.h"

#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "simple_ble.h"
#include "MAX14661.h"

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

APP_TIMER_DEF(impedance_timer);
int Terminal1,Terminal2,Gainres,Scannum;
long Freqstart,Freqstep;

int16_t real[60];
int16_t imag[60];

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
}
static void m_sample_timer_handler(void *p_context)
{

    if(AD5933frequencySweepAsync(real, imag, Scannum))
    {
        int16_t senddata[120];
					
        for(int i=0;i<60;i++){
            senddata[i] = real[i];
            senddata[i+60] = imag[i];
        }
        
        ble_data_send((uint8_t*)senddata, 240);
        
        NRF_LOG_INFO("Frequency sweep complete!\n");
        NRF_LOG_FLUSH();
        
        app_timer_stop(impedance_timer);
    
    }
        
}

static void timers_create(void)
{
    ret_code_t err_code;
	err_code = app_timer_create(&impedance_timer, APP_TIMER_MODE_REPEATED, m_sample_timer_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */



void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        char tempstr[200];
        memcpy(tempstr,p_evt->params.rx_data.p_data,p_evt->params.rx_data.length);
        tempstr[p_evt->params.rx_data.length] = 0;
    
        
        sscanf(tempstr,"%d,%d,%d,%ld,%ld,%d\n",&Terminal1,&Terminal2,&Gainres,&Freqstart,&Freqstep,&Scannum);
        
        if(Scannum > 60)
            Scannum = 60;
    
        NRF_LOG_INFO("%d,%d,%d,%ld,%ld,%d\n",Terminal1,Terminal2,Gainres,Freqstart,Freqstep,Scannum);
        NRF_LOG_FLUSH();
        
        SetTerminal(Terminal1,Terminal2);
        SetGainRes(Gainres);
        
        if (!(  AD5933reset() &&
                AD5933setInternalClock(true) &&
                AD5933setStartFrequency(Freqstart) &&
                AD5933setIncrementFrequency(Freqstep) &&
                AD5933setNumberIncrements(Scannum-1) &&
                AD5933setPGAGain(PGA_GAIN_X1)
            ))
        {
            NRF_LOG_INFO("FAILED in initialization!");
            return;
        }
		
        err_code = app_timer_start(impedance_timer, APP_TIMER_TICKS(10), NULL);  //2ms
        APP_ERROR_CHECK(err_code);
				
    }


}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}




/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
		
	//APP_ERROR_CHECK(ble_dfu_buttonless_async_svci_init());  //Enable in Release
    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    //NRF_LOG_INFO("START");
    AD5933_begin();
    initGainRes();
    timers_create();
        
	sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
	simple_ble_init();

    // Start execution.
    NRF_LOG_INFO("BLE Template Init.");
    advertising_start();
    

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
