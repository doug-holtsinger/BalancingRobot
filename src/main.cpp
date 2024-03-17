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

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bsp.h"
#include "boards.h"

#include "app_config.h"
#include "imu.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"

#include "PID.h"
#include "MotorDriver.h"
#include "board_init.h"
#include "AppDemux.h"
#include "QDEC.h"

constexpr float SPEED_PID_KP = 0.02;
constexpr float SPEED_PID_KI = 0.0;
constexpr float SPEED_PID_KD = 0.0;
constexpr float SPEED_PID_SP = 0.0;
constexpr float SPEED_PID_KP_INCR = 0.005;
constexpr float SPEED_PID_KI_INCR = 0.005;
constexpr float SPEED_PID_KD_INCR = 0.005;
constexpr float SPEED_PID_SP_INCR = 0.05;
constexpr float SPEED_PID_CTRL_MAX = 1.0;
constexpr bool SPEED_PID_REVERSE_OUTPUT = true;
constexpr bool SPEED_PID_LOW_PASS_FILTER = true;

extern int32_t wheel_encoder;

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t cmd_get_cnt = 0;
    float roll, pitch, yaw;
    int16_t roll_i, pitch_i, yaw_i;
    float speedControlSP = 0.0;

    IMU imu = IMU();
    MotorDriver md = MotorDriver();
#if 0
    int16_t wheel_position; 
    int16_t accdbl; 
#endif
    PID speedControlPID = PID({SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_SP}, 
           {SPEED_PID_KP_INCR, SPEED_PID_KI_INCR, SPEED_PID_KD_INCR, SPEED_PID_SP_INCR}, 
            SPEED_PID_CTRL_MAX, SPEED_PID_RECORD_KEY, SPEED_PID_NUM, 
	    SPEED_PID_REVERSE_OUTPUT, SPEED_PID_LOW_PASS_FILTER);

    // Initialize.
    board_init();

    // Start QDEC
    QDEC qdec = QDEC();

    // Start IMU
    imu.init();

    // Add command handler for IMU
    appDemuxAddHandler(
        std::bind( &IMU::cmd, std::ref(imu), std::placeholders::_1),
        appDemuxCmdType(IMU_CMD_t::CMD_MAX) );

    // Initialize speed control PID
    speedControlPID.init();
    // Start Motor Driver
    md.init();

    // Add command handler for Motor Driver
    appDemuxAddHandler(
        std::bind( &MotorDriver::cmd, std::ref(md), std::placeholders::_1),
        appDemuxCmdType(MOTOR_DRIVER_CMD_t::CMD_MAX) );

    // Add command handler for Motor Driver PID
    appDemuxAddHandler(
        std::bind( &MotorDriver::PIDCmd, std::ref(md), std::placeholders::_1),
        appDemuxCmdType(PID_CMD_t::CMD_MAX) );

    // Add command handler for Speed Control PID
    typedef void (PID<float>::*member_func_ptr)(const APP_CMD_t);
    member_func_ptr f = (member_func_ptr)&PID<float>::cmd;
    appDemuxAddHandler(
        std::bind( f,
		   std::ref(speedControlPID), 
		   std::placeholders::_1),
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

        imu.get_angles(roll, pitch, yaw);

        md.setActualRollAngle( roll );

        speedControlSP = speedControlPID.update(static_cast<float>(wheel_encoder));
        md.setDesiredRollAngle(speedControlSP);

#if 0
	if ((cmd_get_cnt & 0x3FF) == 0) 
	{
            NRF_LOG_INFO("alive %d", cmd_get_cnt);
            NRF_LOG_INFO("enc   %d", wheel_encoder);
            NRF_LOG_INFO("setP %f", speedControlSP);
            //NRF_LOG_INFO(" %u", nrf_gpio_pin_read(NRF_QDEC_C2));
            //NRF_LOG_INFO(" qdec en %u", nrf_qdec_enable_get());
#if 0
            NRF_LOG_INFO(" acc      %u", nrf_qdec_acc_get());
            NRF_LOG_INFO(" samplper %u", nrf_qdec_sampleper_reg_get());
            NRF_LOG_INFO(" sampl    %d", nrf_qdec_sample_get());
            NRF_LOG_INFO(" reportper %u", nrf_qdec_reportper_reg_get());
#endif
	}
#endif

#if 0
	if ((cmd_get_cnt & 0x3F) == 0) 
	{
	qdec.read_acc(&wheel_position, &accdbl);
	if (wheel_position != 0)
            NRF_LOG_INFO("NEW acc %hd", wheel_position);
	if (accdbl != 0)
            NRF_LOG_INFO("NEW accdbl %hd", accdbl);
	}
#endif

	if (!ble_svcs_connected())
	{
	    // Send data over advertising channel when not connected.
            roll_i = (int16_t)roll;
            pitch_i = (int16_t)pitch;
            yaw_i = (int16_t)yaw;
            ble_svcs_send_euler_angles(roll_i, pitch_i, yaw_i);
	} else {
            imu.send_all_client_data();
            md.send_all_client_data();
            speedControlPID.send_all_client_data();
	}
    }
}

