/****************************************************************************
 *
 *	Copyright (c) 2021 ETH Zurich - Wyss Zurich. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_voliro_pb.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/conversion/rotation.h>
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_voliro_pb.h>

class PX4Voliro_pb : public cdev::CDev
{

public:
	PX4Voliro_pb(uint32_t device_id);
	~PX4Voliro_pb() override;

	const sensor_voliro_pb_s &get() { return _sensor_voliro_pb_pub.get(); }

	void set_device_type(uint8_t devtype);
	void set_error_count(uint64_t error_count) { _sensor_voliro_pb_pub.get().error_count = error_count; }

	void update(const hrt_abstime &timestamp_sample, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status,
		    uint8_t pwr_brd_blink_reg, uint8_t pwr_brd_led_1_pwr, uint8_t pwr_brd_led_2_pwr, uint8_t pwr_brd_led_3_pwr,
		    uint8_t pwr_brd_led_4_pwr, float pwr_brd_system_volt, float pwr_brd_system_amp, float pwr_brd_battery_volt,
		    float pwr_brd_battery_amp, float pwr_5v_analog_amp, float pwr_5v_digital_amp, float pwr_12v_analog_amp,
		    float pwr_12v_digital_amp);
	int get_class_instance() { return _class_device_instance; };

private:

	uORB::PublicationMultiData<sensor_voliro_pb_s>	_sensor_voliro_pb_pub;

	int			_class_device_instance{-1};

};
