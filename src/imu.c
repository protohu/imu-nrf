#include "imu.h"
#include "imu_driver.h"

static const ImuDriverOps *active_driver;

void imu_register_driver(const ImuDriverOps *ops)
{
	active_driver = ops;
}

void imu_init(void)
{
	if (active_driver && active_driver->init) {
		active_driver->init();
	}
}

bool imu_get_data(ImuData *out)
{
	if (!active_driver || !active_driver->get_data) {
		return false;
	}
	return active_driver->get_data(out);
}
