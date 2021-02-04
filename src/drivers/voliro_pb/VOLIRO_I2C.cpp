/****************************************************************************
 *
 *   Copyright (c) 2021 Wyss Zurich - ETH Zurich Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file VOLIRO_PB_I2C.cpp
 *
 * I2C interface for VOLIRO_PB
 */

#include <lib/drivers/device/i2c.h>

namespace voliro_pb
{

device::Device *VOLIRO_PB_I2C_interface(uint8_t bus, uint32_t address, int bus_frequency);

class VOLIRO_PB_I2C : public device::I2C
{
public:
	VOLIRO_PB_I2C(uint8_t bus, uint32_t address, int bus_frequency);
	virtual ~VOLIRO_PB_I2C() = default;

	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);
};

device::Device *
VOLIRO_PB_I2C_interface(uint8_t bus, uint32_t address, int bus_frequency)
{
	return new VOLIRO_PB_I2C(bus, address, bus_frequency);
}

VOLIRO_PB_I2C::VOLIRO_PB_I2C(uint8_t bus, uint32_t address, int bus_frequency) :
	I2C(DRV_PWR_DEVTYPE_VOLIRO_PB, MODULE_NAME, bus, address, bus_frequency)
{
}

int
VOLIRO_PB_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	uint8_t cmd_size = 1;

	if (cmd == NULL) {
		cmd_size = 0;
	}

	return transfer(&cmd, cmd_size, (uint8_t *)data, count);
}

int
VOLIRO_PB_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}
} // namespace voliro_pb
