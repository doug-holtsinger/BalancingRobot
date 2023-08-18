/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_twi.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "nrf_qdec.h"
#include "nrf_drv_qdec.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bsp.h"
#include "boards.h"

#include "app_config.h"
#include "imu.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"

#include "MotorDriver.h"
#include "board_init.h"
#include "AppDemux.h"

static volatile bool m_report_ready_flag = false;
static volatile bool m_first_report_flag = true;
static volatile uint32_t m_accdblread;
static volatile int32_t m_accread;

static void qdec_event_handler(nrf_drv_qdec_event_t event)
{
    if (event.type == NRF_QDEC_EVENT_REPORTRDY)
    {
        m_accdblread        = event.data.report.accdbl;
        m_accread           = event.data.report.acc;
        m_report_ready_flag = true;
        nrf_drv_qdec_disable();
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t cmd_get_cnt = 0;
    uint32_t err_code;
    float roll, pitch, unused1;
    int16_t roll_i, roll_driver_ratio; 
    pid_ctrl_t  driver0, unused2;
    IMU imu = IMU();
    MotorDriver md = MotorDriver();

    // Initialize.
    board_init();

    // Initialize QDEC hardware
    //nrfx_qdec_config_t qdec_cfg;
    //qdec_cfg.pselled = NRF_QDEC_LED_NOT_CONNECTED;
    err_code = nrf_drv_qdec_init(NULL, qdec_event_handler);
    APP_ERROR_CHECK(err_code);

    // Start IMU
    imu.init();

    // Add command handler for IMU
    appDemuxAddHandler(
        std::bind( &IMU::cmd, std::ref(imu), std::placeholders::_1),
        appDemuxCmdType(IMU_CMD_t::CMD_MAX) );

    // Start Motor Driver
    md.init();

    // Add command handler for Motor Driver
    appDemuxAddHandler(
        std::bind( &MotorDriver::cmd, std::ref(md), std::placeholders::_1),
        appDemuxCmdType(MOTOR_DRIVER_CMD_t::CMD_MAX) );

    // Add command handler for PID
    appDemuxAddHandler(
        std::bind( &MotorDriver::PIDCmd, std::ref(md), std::placeholders::_1),
        appDemuxCmdType(PID_CMD_t::CMD_MAX) );

    // register callback handler
    ble_svcs_register(&appDemuxExecHandler);

    board_post_init();

    // Start execution.
    NRF_LOG_INFO("Balancing Robot example started.");
 
    // Enter main loop.
    for (;; cmd_get_cnt++)
    {
        imu.update();

        imu.get_angles(roll, pitch, unused1);

        md.setRollAngle( roll );

        md.getValues(driver0, unused2);

	if (!ble_svcs_connected())
	{
	    // Send data over advertising channel when not connected.
	    if (roll != 0) 
	    {
	        roll_driver_ratio  = (int16_t)(roll / ((float)driver0/8192.0));
	    } else {
	        roll_driver_ratio  = 0;
	    }
	    roll_i = (int16_t)roll;
            ble_svcs_send_euler_angles(roll_i, driver0, roll_driver_ratio);
	} else {
            imu.send_all_client_data();
            md.send_all_client_data();
	}
    }
}

