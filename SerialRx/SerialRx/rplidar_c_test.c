#include "inc/rptypes.h"
#include "inc/rplidar_protocol.h"
#include "inc/rplidar_cmd.h"
#include "rplidar_driver_impl.h"

#include <stdbool.h>
#include <stdio.h>

int millis() { return (20); }
bool OpenLpLidar();

bool testBasicCommandHealth()
{
	rplidar_response_device_health_t healthinfo;

	if (isConnected())
	{
		u_result result = reset(1000);
		if (result == 0)
		{
			result = getHealth(&healthinfo, 100);
			if (result == 0)
			{
				printf("NO ERROR!!!");
				return true;
			}
		}
	}
	return false;
}

bool testBasicCommandInfo()
{
	rplidar_response_device_info_t infoinfo;

	if (isConnected())
	{
		u_result result = reset(1000);
		if (result == 0)
		{
			result = getDeviceInfo(&infoinfo, 100);
			if (result == 0)
			{
				printf("NO ERROR!!!");
				return true;
			}
		}
	}
	return false;
}

bool runAllTests()
{
	bool result;
	result = rb_begin();
	if (result)
	{
		result = testBasicCommandHealth();
		result = testBasicCommandInfo();
		return result;
	}
	else
	{
		return false;
	}
}
