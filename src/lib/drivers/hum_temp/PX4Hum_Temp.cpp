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


#include "PX4Hum_Temp.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Hum_Temp::PX4Hum_Temp(uint32_t device_id) :
	CDev(nullptr),
	_sensor_hum_temp_pub{ORB_ID(sensor_hum_temp)}
{
	_class_device_instance = register_class_devname(HUM_TEMP_DEVICE_PATH);
	_sensor_hum_temp_pub.advertise();

	_sensor_hum_temp_pub.get().device_id = device_id;
}

PX4Hum_Temp::~PX4Hum_Temp()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(HUM_TEMP_DEVICE_PATH, _class_device_instance);
	}

	_sensor_hum_temp_pub.unadvertise();
}

void PX4Hum_Temp::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _sensor_hum_temp_pub.get().device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back to report
	_sensor_hum_temp_pub.get().device_id = device_id.devid;
}

void PX4Hum_Temp::update(const hrt_abstime &timestamp_sample, float humidity, float temperature)
{
	sensor_hum_temp_s &report = _sensor_hum_temp_pub.get();

	report.timestamp_sample = timestamp_sample;
	report.relative_humidity = humidity;
	report.ambient_temperature = temperature;
	report.timestamp = hrt_absolute_time();
	_sensor_hum_temp_pub.update();
}
