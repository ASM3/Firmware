/****************************************************************************
 *
 *   Copyright (c) 2020 WYSS Zurich Development Team. All rights reserved.
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

#ifndef VOLIRO_PB_HPP
#define VOLIRO_PB_HPP

#include <uORB/topics/sensor_voliro_pb.h>
#include <v2.0/common/mavlink_msg_sens_voliro_pb.h>

class MavlinkStreamVoliroPBData : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamVoliroPBData::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SENS_VOLIRO_PB";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SENS_VOLIRO_PB;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVoliroPBData(mavlink);
	}

	unsigned get_size() override
	{
		return _voliro_pb_sub.advertised() ? MAVLINK_MSG_ID_SENS_VOLIRO_PB_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _voliro_pb_sub{ORB_ID(sensor_voliro_pb)};

	/* do not allow top copying this class */
	MavlinkStreamVoliroPBData(MavlinkStreamVoliroPBData &) = delete;
	MavlinkStreamVoliroPBData &operator = (const MavlinkStreamVoliroPBData &) = delete;

protected:
	explicit MavlinkStreamVoliroPBData(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		sensor_voliro_pb_s voliro_pb_data;
		bool sent = false;

		while ((_mavlink->get_free_tx_buf() >= get_size()) && _voliro_pb_sub.update(&voliro_pb_data)) {
			mavlink_sens_voliro_pb_t msg = {};

			msg.timestamp = voliro_pb_data.timestamp;
			msg.pwr_brd_status = voliro_pb_data.pwr_brd_status;
			msg.pwr_brd_led_status  = voliro_pb_data.pwr_brd_led_status;
			msg.pwr_brd_blink_status  = voliro_pb_data.pwr_brd_blink_reg;
			msg.pwr_brd_power_led_1  = voliro_pb_data.pwr_brd_led_1_pwr;
			msg.pwr_brd_power_led_2  = voliro_pb_data.pwr_brd_led_2_pwr;
			msg.pwr_brd_power_led_3  = voliro_pb_data.pwr_brd_led_3_pwr;
			msg.pwr_brd_power_led_4  = voliro_pb_data.pwr_brd_led_4_pwr;
			msg.pwr_brd_system_volt = voliro_pb_data.pwr_brd_system_volt;
			msg.pwr_brd_battery_volt = voliro_pb_data.pwr_brd_battery_volt;
			msg.pwr_brd_system_current = voliro_pb_data.pwr_brd_system_amp;
			msg.pwr_brd_battery_current = voliro_pb_data.pwr_brd_battery_amp;
			msg.pwr_brd_5v_analog_current = voliro_pb_data.pwr_5v_analog_amp;
			msg.pwr_brd_5v_digital_current = voliro_pb_data.pwr_5v_digital_amp;
			msg.pwr_brd_12v_analog_current = voliro_pb_data.pwr_12v_analog_amp;
			msg.pwr_brd_12v_digital_current = voliro_pb_data.pwr_12v_digital_amp;

			mavlink_msg_sens_voliro_pb_send_struct(_mavlink->get_channel(), &msg);

			sent = true;
		}

		return sent;
	}
};

#endif // VOLIRO_PB_HPP
