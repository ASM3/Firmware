/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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


#include <lib/drivers/device/Device.hpp>
#include "PX4Voliro_pb.hpp"

PX4Voliro_pb::PX4Voliro_pb(uint32_t device_id) :
	CDev(nullptr),
	_sensor_voliro_pb_pub{ORB_ID(sensor_voliro_pb)}
{
	_class_device_instance = register_class_devname(VOLIRO_PB_DEVICE_PATH);
	_sensor_voliro_pb_pub.advertise();

	_sensor_voliro_pb_pub.get().device_id = device_id;
}

PX4Voliro_pb::~PX4Voliro_pb()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(VOLIRO_PB_DEVICE_PATH, _class_device_instance);
	}

	_sensor_voliro_pb_pub.unadvertise();
}

void PX4Voliro_pb::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _sensor_voliro_pb_pub.get().device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back to report
	_sensor_voliro_pb_pub.get().device_id = device_id.devid;
}

void PX4Voliro_pb::update(const hrt_abstime &timestamp_sample, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status,
			  uint8_t pwr_brd_blink_reg, uint8_t pwr_brd_led_1_pwr, uint8_t pwr_brd_led_2_pwr, uint8_t pwr_brd_led_3_pwr,
			  uint8_t pwr_brd_led_4_pwr, float pwr_brd_system_volt, float pwr_brd_system_amp, float pwr_brd_battery_volt,
			  float pwr_brd_battery_amp, float pwr_5v_analog_amp, float pwr_5v_digital_amp, float pwr_12v_analog_amp,
			  float pwr_12v_digital_amp)
{
	sensor_voliro_pb_s &report = _sensor_voliro_pb_pub.get();

	report.timestamp_sample = timestamp_sample;
	report.timestamp = hrt_absolute_time();
	report.pwr_brd_status = pwr_brd_status;
	report.pwr_brd_led_status = pwr_brd_led_status;
	report.pwr_brd_blink_reg = pwr_brd_blink_reg;
	report.pwr_brd_led_1_pwr = pwr_brd_led_1_pwr;
	report.pwr_brd_led_2_pwr = pwr_brd_led_2_pwr;
	report.pwr_brd_led_3_pwr = pwr_brd_led_3_pwr;
	report.pwr_brd_led_4_pwr = pwr_brd_led_4_pwr;
	report.pwr_brd_system_volt = pwr_brd_system_volt;
	report.pwr_brd_system_amp = pwr_brd_system_amp;
	report.pwr_brd_battery_volt = pwr_brd_battery_volt;
	report.pwr_brd_battery_amp = pwr_brd_battery_amp;
	report.pwr_5v_analog_amp = pwr_5v_analog_amp;
	report.pwr_5v_digital_amp = pwr_5v_digital_amp;
	report.pwr_12v_analog_amp = pwr_12v_analog_amp;
	report.pwr_12v_digital_amp = pwr_12v_digital_amp;

	_sensor_voliro_pb_pub.update();
}
