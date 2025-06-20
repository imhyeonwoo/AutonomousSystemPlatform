
#include "../inc/fc_sensor.h"

int fc_sensor_get_time_offset(void)
{
	return 0;
}

int fc_sensor_set_library_name(const char *name)
{
	return 0;
}

int fc_sensor_initialize(bool enable_debug_messages, fc_callbacks *callbacks)
{
	return 0;
}

int fc_sensor_advertise(const char *topic)
{
	return 0;
}

int fc_sensor_subscribe(const char *topic)
{
	return 0;
}

int fc_sensor_unsubscribe(const char *topic)
{
	return 0;
}

int fc_sensor_send_data(const char *topic,
			const uint8_t *data,
			uint32_t length_in_bytes)
{
	return 0;
}

void fc_sensor_kill_slpi(void)
{
	return;
}

