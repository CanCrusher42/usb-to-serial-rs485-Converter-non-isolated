#include "inc/rptypes.h"
#include "inc/rplidar_protocol.h"
#include "inc/rplidar_cmd.h"
#include "rplidar_driver_impl.h"

#include <stdbool.h>
#include <stdio.h>

int millis() { return (20); }
bool OpenLpLidar();

bool testBasicCommand()
{
	rplidar_response_device_health_t healthinfo;
	rb_begin();

	if (isConnected())
	{
		u_result result = reset(1000);
		if (result == 0)
		{
			result = getHealth(&healthinfo, 100);
			if (result == 0)
			{
				printf("NO ERROR!!!");
			}
		}
	}

}