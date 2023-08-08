#include "inc/rptypes.h"
#include "inc/rplidar_protocol.h"
#include "inc/rplidar_cmd.h"
#include "rplidar_driver_impl.h"

#include <stdbool.h>
#include <stdio.h>
int GetTickCount();
int millis() { return (20); }
bool OpenLpLidar();
extern rplidar_response_measurement_node_t finalLineData[1 * 180];

void DisplayLineDistance(uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight);

bool testBasicCommandHealth()
{
	rplidar_response_device_health_t healthinfo;

	if (isConnected())
	{
		stop(200);
		u_result result = 0;// = reset(1000);
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
extern uint16_t scans;
bool testExpressScanMode()
{
	int loopCount = 0;
	int screens = 0;

	u_result result = startScanExpress(false, RPLIDAR_CONF_SCAN_COMMAND_EXPRESS, 0, NULL, 200);
	int startTime, endTime;

	while (screens < 200)
	{
		
		startTime = GetTickCount();
		memset(finalLineData, 0, sizeof(finalLineData));
		scans = 0;
		while (loopCount < 100)
		{
			result = loopScanExpressData6();
			result = loopScanExpressData6();
			loopCount += 2;
			//printf("_cached_scan_node_hq_count = %d\n", loopCount);
		}
		endTime = GetTickCount();
		//printf("Delta Time = %d scans = %d\n", endTime - startTime, scans);
		//printf("ScanRate = %d\n", scans/((endTime-startTime)/1000));
		system("cls");
		//DisplayLineDistance(0, 180, 20, 50);
		//DisplayLineAngle(0, 180, 20, 50);
		DisplayLineToRoom(0, 180, 20, 50);
		
		lidarClear_serial();
		screens++;
		loopCount = 0;
	}
	return true;
}
bool runAllTests()
{
	bool result;
	result = rb_begin();
	if (result)
	{
		reset(100);
		printf("\nStarting Tests\n\n");
		result = testBasicCommandHealth();
		result = testBasicCommandInfo();
		result = testExpressScanMode();
		return result;
	}
	else
	{
		return false;
	}
}
