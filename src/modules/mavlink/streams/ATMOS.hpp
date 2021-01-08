/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef ATMOS_HPP
#define ATMOS_HPP

#include <uORB/topics/sensor_hum_temp.h>
#include <v2.0/ASLUAV/mavlink_msg_sens_atmos.h>

class MavlinkStreamAtmosData : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAtmosData::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SENS_ATMOS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SENS_ATMOS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAtmosData(mavlink);
	}

	unsigned get_size() override
	{
		return _atmos_sub.advertised() ? MAVLINK_MSG_ID_SENS_ATMOS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _atmos_sub{ORB_ID(sensor_hum_temp)};

	/* do not allow top copying this class */
	MavlinkStreamAtmosData(MavlinkStreamAtmosData &) = delete;
	MavlinkStreamAtmosData &operator = (const MavlinkStreamAtmosData &) = delete;

protected:
	explicit MavlinkStreamAtmosData(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		sensor_hum_temp_s hum_temp_data;
		bool sent = false;

		while ((_mavlink->get_free_tx_buf() >= get_size()) && _atmos_sub.update(&hum_temp_data)) {
			mavlink_sens_atmos_t msg = {};

			//msg.atmos_timestamp = hum_temp_data.timestamp;
			msg.Humidity = hum_temp_data.relative_humidity;
			msg.TempAmbient = hum_temp_data.ambient_temperature;

			mavlink_msg_sens_atmos_send_struct(_mavlink->get_channel(), &msg);

			sent = true;
		}

		return sent;
	}
};

#endif // ATMOS_HPP
