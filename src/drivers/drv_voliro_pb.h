/****************************************************************************
 *
 *   Copyright (C) 2021 Wyss zurich Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Voliro power board driver interface.
 */

#ifndef _DRV_VOLIRO_PB_H
#define _DRV_VOLIRO_PB_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define VOLIRO_PB_DEVICE_PATH	"/dev/voliro_pb"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_voliro_pb.h>

#define voliro_pb_report sensor_voliro_pb_s

struct voliro_pb_calibration_s {
	float _bias_cal_term_system_volt;
	float _SF_cal_term_system_volt;
	float _bias_cal_term_system_amp;
	float _SF_cal_term_system_amp;
	float _bias_cal_term_battery_volt;
	float _SF_cal_term_battery_volt;
	float _bias_cal_term_battery_amp;
	float _SF_cal_term_battery_amp;
	float _bias_cal_term_5v_digital_amp;
	float _SF_cal_term_5v_digital_amp;
	float _bias_cal_term_5v_analog_amp;
	float _SF_cal_term_5v_analog_amp;
	float _bias_cal_term_12v_digital_amp;
	float _SF_cal_term_12v_digital_amp;
	float _bias_cal_term_12v_analog_amp;
	float _SF_cal_term_12v_analog_amp;
};

/*
 * ioctl() definitions
 */

#define _VOLIRO_PBIOCBASE		(0x3400)
#define _VOLIRO_PBIOC(_n)		(_PX4_IOC(_VOLIRO_PBIOCBASE, _n))


#endif /* _DRV_VOLIRO_PB_H */
