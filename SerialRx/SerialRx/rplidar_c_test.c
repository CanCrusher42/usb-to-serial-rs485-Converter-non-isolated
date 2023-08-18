#include "inc/rptypes.h"
#include "inc/rplidar_protocol.h"
#include "inc/rplidar_cmd.h"
#include "rplidar_driver_impl.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "serial.h"

int GetTickCount();
int millis() { return (20); }
bool OpenLpLidar();
extern rplidar_response_measurement_node_t finalLineData[1 * 180];
void ConvertDisplayLineToRoom(uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight);
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
		ConvertDisplayLineToRoom(0, 180, 20, 50);
		

		lidarClear_serial();
		screens++;
		loopCount = 0;
	}
	return true;
}



#define TEST_SERIAL
#ifdef TEST_SERIAL
extern bool serialTestMode;
extern int testBuffer[128];
extern int16_t testBufferIndex;
rplidar_response_capsule_measurement_nodes_t testNode;
u_result _waitCapsuledNodeRTOS(rplidar_response_capsule_measurement_nodes_t* node, bool  new);

bool testRtosSyncBit()
{
	u_result result;
	memset(&testNode, 0, sizeof(rplidar_response_capsule_measurement_nodes_t));
	serialTestMode = true;
	testBufferIndex = 0;
	testBuffer[0] = -1;
	testBuffer[1] = -1;
	testBuffer[2] = 0xA2;
	testBuffer[3] = 0x53;
	testBuffer[4] = -1;
	result = _waitCapsuledNodeRTOS(&testNode, true);
	result = _waitCapsuledNodeRTOS(&testNode, false);
	result = _waitCapsuledNodeRTOS(&testNode, false);
	result = _waitCapsuledNodeRTOS(&testNode, false);
	return ((testNode.s_checksum_1 == 0xA2) && (testNode.s_checksum_2 == 0x53));


}
bool testRtosWholePacket1()
{
	u_result result;
	_u8 data1[] = { 0xa1, 0x59, 0xc7, 0xd9, 0x03, 0x00, 0xd7, 0x15, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0c, 0xf3, 0x2a, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x2b, 0x03, 0x00, 0xcc, 0xf3, 0x29, 0xfb, 0x29, 0xcc, 0x67, 0x2a, 0xaf, 0x2a, 0xcc, 0x9b, 0x2a, 0xa3, 0x2a, 0xcc, 0xa3, 0x2a, 0xcb, 0x2a, 0xcc, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x2f, 0x2d, 0xd0, 0x03, 0x00, 0xdf, 0x2a, 0xcc, 0x6b, 0x2a, 0xaf, 0x2a, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 84 };

	uint16_t size = sizeof(data1);
	for (int i = 0; i < size; i++)
	{
		testBuffer[i] = data1[i];
	}
	memset(&testNode, 0, sizeof(rplidar_response_capsule_measurement_nodes_t));
	serialTestMode = true;
	testBufferIndex = 0;
	for (int i = 0; i < size; i++)
	{
		result = _waitCapsuledNodeRTOS(&testNode, (i==0));
	}
	return true;

}
bool runAllRtosTests()
{
	bool result = rb_begin();
	if (result)
	{

		reset(100);
		printf("\nStarting RTOS Tests\n\n");
		result = testRtosSyncBit();
		result = testRtosWholePacket1();
	}
	return result;
}
#endif

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
