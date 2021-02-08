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

	void update(sensor_voliro_pb_s);

	int get_class_instance() { return _class_device_instance; };

private:

	uORB::PublicationMultiData<sensor_voliro_pb_s>	_sensor_voliro_pb_pub;

	int			_class_device_instance{-1};

};
