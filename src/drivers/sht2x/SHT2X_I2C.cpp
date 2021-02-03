/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file SHT2X_I2C.cpp
 *
 * I2C interface for SHT2X
 */

#include <lib/drivers/device/i2c.h>

namespace sht2x
{

device::Device *SHT2X_I2C_interface(uint8_t bus, uint32_t address, int bus_frequency);

class SHT2X_I2C : public device::I2C
{
public:
	SHT2X_I2C(uint8_t bus, uint32_t address, int bus_frequency);
	virtual ~SHT2X_I2C() = default;

	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);
};

device::Device *
SHT2X_I2C_interface(uint8_t bus, uint32_t address, int bus_frequency)
{
	return new SHT2X_I2C(bus, address, bus_frequency);
}

SHT2X_I2C::SHT2X_I2C(uint8_t bus, uint32_t address, int bus_frequency) :
	I2C(DRV_HUM_TEMP_DEVTYPE_SHT2X, MODULE_NAME, bus, address, bus_frequency)
{
}

int
SHT2X_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	uint8_t cmd_size = 1;

	if (cmd == NULL) {
		cmd_size = 0;
	}

	return transfer(&cmd, cmd_size, (uint8_t *)data, count);
}

int
SHT2X_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}
} // namespace sht2x
