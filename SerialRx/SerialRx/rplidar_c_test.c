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
extern int testBuffer[12000];
extern int16_t testBufferIndex;
extern int16_t maxTextBufferIndex;
rplidar_response_capsule_measurement_nodes_t testNode;
u_result _waitCapsuledNodeRTOS(rplidar_response_capsule_measurement_nodes_t* node, bool  new);

float getAngleFromPacket(rplidar_response_capsule_measurement_nodes_t* node)
{
	float angle = (float)(node->start_angle_sync_q6 & 0x7FFF);
	uint16_t angle64 = node->start_angle_sync_q6 & 0x7FFF;
	angle = angle/64;
	return angle;
}
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
	maxTextBufferIndex = 4;
	result = _waitCapsuledNodeRTOS(&testNode, true);
	result = _waitCapsuledNodeRTOS(&testNode, false);
	result = _waitCapsuledNodeRTOS(&testNode, false);
	result = _waitCapsuledNodeRTOS(&testNode, false);
	return ((testNode.s_checksum_1 == 0xA2) && (testNode.s_checksum_2 == 0x53));


}

bool testRtosWholePacket1()
{
	u_result result = RESULT_WAITING;
	_u8 data1[] = { 0xa1, 0x59, 0xc7, 0xd9, 0x03, 0x00, 0xd7, 0x15, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0c, 0xf3, 0x2a, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x2b, 0x03, 0x00, 0xcc, 0xf3, 0x29, 0xfb, 0x29, 0xcc, 0x67, 0x2a, 0xaf, 0x2a, 0xcc, 0x9b, 0x2a, 0xa3, 0x2a, 0xcc, 0xa3, 0x2a, 0xcb, 0x2a, 0xcc, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x2f, 0x2d, 0xd0, 0x03, 0x00, 0xdf, 0x2a, 0xcc, 0x6b, 0x2a, 0xaf, 0x2a, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 84 };

	uint16_t size = sizeof(data1);

	for (int i = 0; i < size; i++)
	{
		testBuffer[i] = data1[i];
	}
	memset(&testNode, 0, sizeof(rplidar_response_capsule_measurement_nodes_t));
	serialTestMode = true;
	testBufferIndex = 0;
	maxTextBufferIndex = size;

	int i;
	for ( i = 0; ((i < size) && (result != RESULT_OK)); i++)
	{
		result = _waitCapsuledNodeRTOS(&testNode, (i==0));
	}
	if (result != RESULT_OK)
	{
		printf("ERRROR %s  Result is not RESULT_OK, i = %d\n", __FUNCTION__, i);
	}
	float angle = getAngleFromPacket(&testNode);

	return (result == RESULT_OK);
}

bool testRtosBadGoodPacket1()
{
	u_result result = RESULT_WAITING;
	_u8 data1[] = { 0xa1, 0x59, 0xc7, 0xd9, 0x03, 0x00, 0xd7, 0x15, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x42, 0x00, 0x00, 0x0c, 0xf3, 0x2a, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x2b, 0x03, 0x00, 0xcc, 0xf3, 0x29, 0xfb, 0x29, 0xcc, 0x67, 0x2a, 0xaf, 0x2a, 0xcc, 0x9b, 0x2a, 0xa3, 0x2a, 0xcc, 0xa3, 0x2a, 0xcb, 0x2a, 0xcc, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x2f, 0x2d, 0xd0, 0x03, 0x00, 0xdf, 0x2a, 0xcc, 0x6b, 0x2a, 0xaf, 0x2a, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 84 ,
	                0xa1, 0x59, 0xc7, 0xd9, 0x03, 0x00, 0xd7, 0x15, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0c, 0xf3, 0x2a, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x2b, 0x03, 0x00, 0xcc, 0xf3, 0x29, 0xfb, 0x29, 0xcc, 0x67, 0x2a, 0xaf, 0x2a, 0xcc, 0x9b, 0x2a, 0xa3, 0x2a, 0xcc, 0xa3, 0x2a, 0xcb, 0x2a, 0xcc, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x2f, 0x2d, 0xd0, 0x03, 0x00, 0xdf, 0x2a, 0xcc, 0x6b, 0x2a, 0xaf, 0x2a, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 84 };

	uint16_t size = sizeof(data1);

	for (int i = 0; i < size; i++)
	{
		testBuffer[i] = data1[i];
	}
	memset(&testNode, 0, sizeof(rplidar_response_capsule_measurement_nodes_t));
	serialTestMode = true;
	testBufferIndex = 0;
	maxTextBufferIndex = size;

	int i;
	for (i = 0; ((i < size) && (result != RESULT_OK)); i++)
	{
		result = _waitCapsuledNodeRTOS(&testNode, (i == 0));
	}
	if (result != RESULT_OK)
	{
		printf("ERRROR %s  Result is not RESULT_OK, i = %d\n", __FUNCTION__, i);
	}
	float angle = getAngleFromPacket(&testNode);

	return (result == RESULT_OK);
}



bool testGetLineOfData()
{
	u_result result = RESULT_WAITING;
	_u8 data1[] = { 0xa1, 0x59, 0xc7, 0xd9, 0x03, 0x00, 0xd7, 0x15, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0c, 0xf3, 0x2a, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x2b, 0x03, 0x00, 0xcc, 0xf3, 0x29, 0xfb, 0x29, 0xcc, 0x67, 0x2a, 0xaf, 0x2a, 0xcc, 0x9b, 0x2a, 0xa3, 0x2a, 0xcc, 0xa3, 0x2a, 0xcb, 0x2a, 0xcc, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x2f, 0x2d, 0xd0, 0x03, 0x00, 0xdf, 0x2a, 0xcc, 0x6b, 0x2a, 0xaf, 0x2a, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	         	    0xa1, 0x51, 0x89, 0x05, 0x00, 0x00, 0x03, 0x00, 0x10, 0x03, 0x00, 0x9f, 0x0a, 0x00, 0x62, 0x0a, 0x36, 0x0a, 0xef, 0x22, 0x0a, 0x2a, 0x0a, 0xfe, 0x1a, 0x0a, 0x02, 0x0a, 0xff, 0xea, 0x09, 0x02, 0x00, 0xed, 0x5e, 0x09, 0x46, 0x09, 0xcd, 0x32, 0x09, 0x22, 0x09, 0xdc, 0x12, 0x09, 0x02, 0x09, 0xdc, 0xee, 0x08, 0xde, 0x08, 0xcc, 0xd2, 0x08, 0xca, 0x08, 0xdc, 0xc6, 0x08, 0xde, 0x08, 0xdc, 0x02, 0x00, 0x7a, 0x08, 0xac, 0x66, 0x08, 0x7a, 0x08, 0xcb, 0x02, 0x00, 0x02, 0x00, 0xaa, 0xba, 0x07, 0x02, 0x00, 0x89,
					0xa2, 0x54, 0x52, 0x0b, 0x5e, 0x07, 0x5e, 0x07, 0x87, 0x5e, 0x07, 0x72, 0x07, 0x78, 0x8e, 0x07, 0x8e, 0x07, 0x8a, 0x76, 0x07, 0x56, 0x07, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x90, 0x12, 0x08, 0x32, 0x08, 0xca, 0x5a, 0x08, 0x02, 0x00, 0xba, 0xd2, 0x08, 0x0a, 0x09, 0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xf3, 0x0b, 0x11, 0xfb, 0x0b, 0x13, 0x0c, 0x21, 0x3f, 0x0c, 0x5f, 0x0c, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xe0, 
		            0xad, 0x5c, 0x18, 0x11, 0x03, 0x00, 0x0f, 0x37, 0xdd, 0x2b, 0x36, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x93, 0x33, 0xdd, 0x7b, 0x33, 0x03, 0x00, 0xdd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd3, 0x31, 0xd0, 0xc3, 0x31, 0x07, 0x31, 0xdd, 0xc7, 0x30, 0xeb, 0x30, 0xdd, 
		            0xa7, 0x5e, 0xdb, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xa0, 0xeb, 0x1a, 0xe7, 0x1a, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xd0, 0x03, 0x00, 0x13, 0x1b, 0xaa, 0x2b, 0x1b, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x03, 0x00, 0x03, 0x00, 0xdd, 0x03, 0x00, 0x4b, 0x2e, 0xdd, 0x03, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 
		            0xa2, 0x5e, 0x9d, 0x1c, 0x03, 0x00, 0x27, 0x23, 0xcb, 0x4f, 0x23, 0xa3, 0x23, 0xcc, 0xf3, 0x23, 0x2f, 0x24, 0xcc, 0x4f, 0x24, 0x63, 0x24, 0xcc, 0x63, 0x24, 0x93, 0x24, 0xcc, 0xe7, 0x24, 0xdb, 0x24, 0xbc, 0xa7, 0x24, 0x77, 0x24, 0xbb, 0x67, 0x24, 0x87, 0x24, 0xcc, 0x7f, 0x24, 0x83, 0x24, 0xcb, 0xf3, 0x24, 0xe7, 0x25, 0xcc, 0x00, 0x00, 0x3f, 0x26, 0xc0, 0xbb, 0x25, 0x43, 0x25, 0xcc, 0xe3, 0x24, 0xaf, 0x24, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		            0xa4, 0x55, 0x5d, 0x22, 0x00, 0x00, 0x03, 0x00, 0xb0, 0x43, 0x20, 0x63, 0x20, 0xbb, 0x1b, 0x20, 0x00, 0x00, 0x0b, 0x03, 0x00, 0x03, 0x00, 0xbc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x1d, 0xab, 0xab, 0x1c, 0x43, 0x1c, 0xaa, 0xef, 0x1b, 0xaf, 0x1b, 0xaa, 0x7b, 0x1b, 0x00, 0x00, 0x0a, 0xa3, 0x1b, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x8f, 0x1a, 0x9a, 0xa3, 0x1a, 0x00, 0x00, 0x0a, 0xef, 0x1a, 0x00, 0x00, 0x0a, 0x03, 0x00, 0x00, 0x00, 0x0a, 
		            0xae, 0x5c, 0x1a, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x13, 0x19, 0x9a, 0xf3, 0x18, 0x07, 0x19, 0x99, 0x6f, 0x19, 0xc3, 0x19, 0x99, 0xe3, 0x19, 0x03, 0x00, 0xaa, 0x77, 0x1c, 0x03, 0x00, 0x9a, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd3, 0x18, 0x03, 0x00, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x77, 0x26, 0xcc, 0x03, 0x00, 0x2f, 0x1d, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xc0, 
		            0xaa, 0x5f, 0xe1, 0x2d, 0x3b, 0x26, 0x47, 0x26, 0xcc, 0x4b, 0x26, 0x4f, 0x26, 0xcc, 0x4b, 0x26, 0x4f, 0x26, 0xcc, 0x5f, 0x26, 0x67, 0x26, 0xcc, 0x67, 0x26, 0x73, 0x26, 0xcc, 0x83, 0x26, 0xa3, 0x26, 0xcc, 0xbf, 0x26, 0xd7, 0x26, 0xcc, 0xf7, 0x26, 0x0f, 0x27, 0xcc, 0x2b, 0x27, 0x3f, 0x27, 0xcc, 0x5b, 0x27, 0x7b, 0x27, 0xcc, 0x03, 0x00, 0x1b, 0x22, 0xbb, 0x37, 0x22, 0x7f, 0x22, 0xbb, 0x03, 0x00, 0xc7, 0x23, 0xbb, 0xfb, 0x23, 0x3f, 0x24, 0xcc, 0x67, 0x24, 0x03, 0x00, 0xcb, 0x03, 0x00, 0xaf, 0x29, 0xcc,	
	};

	uint16_t size = sizeof(data1);

	for (int i = 0; i < size; i++)
	{
		testBuffer[i] = data1[i];
	}

	memset(&testNode, 0, sizeof(rplidar_response_capsule_measurement_nodes_t));
	memset(&finalLineData, 0, sizeof(rplidar_response_measurement_node_t) * 180);
	serialTestMode = true;
	testBufferIndex = 0;
	maxTextBufferIndex = size;

	uint16_t goodSamples = 0;
	bool start = true;
	while (testBufferIndex<maxTextBufferIndex)
	{
		uint16_t count = 0;
		result= loopScanExpressAddDataRTOS(start, &count);
		goodSamples += count;
		start = false;

	}
	printf("\ngoodSamples = % d \n", goodSamples);
	if (goodSamples != 95)
	{
		printf("ERROR: %s  Expecting 95 good samples, only found %d\n", __FUNCTION__, goodSamples);
		return false;
	}
	return true;
}


bool testGetLineOfDataErrorInMiddle()
{
	u_result result = RESULT_WAITING;
	_u8 data1[] = { 0xa1, 0x59, 0xc7, 0xd9, 0x03, 0x00, 0xd7, 0x15, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0c, 0xf3, 0x2a, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x2b, 0x03, 0x00, 0xcc, 0xf3, 0x29, 0xfb, 0x29, 0xcc, 0x67, 0x2a, 0xaf, 0x2a, 0xcc, 0x9b, 0x2a, 0xa3, 0x2a, 0xcc, 0xa3, 0x2a, 0xcb, 0x2a, 0xcc, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x2f, 0x2d, 0xd0, 0x03, 0x00, 0xdf, 0x2a, 0xcc, 0x6b, 0x2a, 0xaf, 0x2a, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0xa1, 0x51, 0x89, 0x05, 0x00, 0x00, 0x03, 0x00, 0x10, 0x03, 0x00, 0x9f, 0x0a, 0x00, 0x62, 0x0a, 0x36, 0x0a, 0xef, 0x22, 0x0a, 0x2a, 0x0a, 0xfe, 0x1a, 0x0a, 0x02, 0x0a, 0xff, 0xea, 0x09, 0x02, 0x00, 0xed, 0x5e, 0x09, 0x46, 0x09, 0xcd, 0x32, 0x09, 0x22, 0x09, 0xdc, 0x12, 0x09, 0x02, 0x09, 0xdc, 0xee, 0x08, 0xde, 0x08, 0xcc, 0xd2, 0x08, 0xca, 0x08, 0xdc, 0xc6, 0x08, 0xde, 0x08, 0xdc, 0x02, 0x00, 0x7a, 0x08, 0xac, 0x66, 0x08, 0x7a, 0x08, 0xcb, 0x02, 0x00, 0x02, 0x00, 0xaa, 0xba, 0x07, 0x02, 0x00, 0x89,
/* corrupt CHECKSUM*/	0xa2, 0x54, /*0x52*/0x42, 0x0b, 0x5e, 0x07, 0x5e, 0x07, 0x87, 0x5e, 0x07, 0x72, 0x07, 0x78, 0x8e, 0x07, 0x8e, 0x07, 0x8a, 0x76, 0x07, 0x56, 0x07, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x90, 0x12, 0x08, 0x32, 0x08, 0xca, 0x5a, 0x08, 0x02, 0x00, 0xba, 0xd2, 0x08, 0x0a, 0x09, 0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xf3, 0x0b, 0x11, 0xfb, 0x0b, 0x13, 0x0c, 0x21, 0x3f, 0x0c, 0x5f, 0x0c, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xe0,
					0xad, 0x5c, 0x18, 0x11, 0x03, 0x00, 0x0f, 0x37, 0xdd, 0x2b, 0x36, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x93, 0x33, 0xdd, 0x7b, 0x33, 0x03, 0x00, 0xdd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd3, 0x31, 0xd0, 0xc3, 0x31, 0x07, 0x31, 0xdd, 0xc7, 0x30, 0xeb, 0x30, 0xdd,
					0xa7, 0x5e, 0xdb, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xa0, 0xeb, 0x1a, 0xe7, 0x1a, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xd0, 0x03, 0x00, 0x13, 0x1b, 0xaa, 0x2b, 0x1b, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x03, 0x00, 0x03, 0x00, 0xdd, 0x03, 0x00, 0x4b, 0x2e, 0xdd, 0x03, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
					0xa2, 0x5e, 0x9d, 0x1c, 0x03, 0x00, 0x27, 0x23, 0xcb, 0x4f, 0x23, 0xa3, 0x23, 0xcc, 0xf3, 0x23, 0x2f, 0x24, 0xcc, 0x4f, 0x24, 0x63, 0x24, 0xcc, 0x63, 0x24, 0x93, 0x24, 0xcc, 0xe7, 0x24, 0xdb, 0x24, 0xbc, 0xa7, 0x24, 0x77, 0x24, 0xbb, 0x67, 0x24, 0x87, 0x24, 0xcc, 0x7f, 0x24, 0x83, 0x24, 0xcb, 0xf3, 0x24, 0xe7, 0x25, 0xcc, 0x00, 0x00, 0x3f, 0x26, 0xc0, 0xbb, 0x25, 0x43, 0x25, 0xcc, 0xe3, 0x24, 0xaf, 0x24, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0xa4, 0x55, 0x5d, 0x22, 0x00, 0x00, 0x03, 0x00, 0xb0, 0x43, 0x20, 0x63, 0x20, 0xbb, 0x1b, 0x20, 0x00, 0x00, 0x0b, 0x03, 0x00, 0x03, 0x00, 0xbc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x1d, 0xab, 0xab, 0x1c, 0x43, 0x1c, 0xaa, 0xef, 0x1b, 0xaf, 0x1b, 0xaa, 0x7b, 0x1b, 0x00, 0x00, 0x0a, 0xa3, 0x1b, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x8f, 0x1a, 0x9a, 0xa3, 0x1a, 0x00, 0x00, 0x0a, 0xef, 0x1a, 0x00, 0x00, 0x0a, 0x03, 0x00, 0x00, 0x00, 0x0a,
					0xae, 0x5c, 0x1a, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x13, 0x19, 0x9a, 0xf3, 0x18, 0x07, 0x19, 0x99, 0x6f, 0x19, 0xc3, 0x19, 0x99, 0xe3, 0x19, 0x03, 0x00, 0xaa, 0x77, 0x1c, 0x03, 0x00, 0x9a, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd3, 0x18, 0x03, 0x00, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x77, 0x26, 0xcc, 0x03, 0x00, 0x2f, 0x1d, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0xc0,
					0xaa, 0x5f, 0xe1, 0x2d, 0x3b, 0x26, 0x47, 0x26, 0xcc, 0x4b, 0x26, 0x4f, 0x26, 0xcc, 0x4b, 0x26, 0x4f, 0x26, 0xcc, 0x5f, 0x26, 0x67, 0x26, 0xcc, 0x67, 0x26, 0x73, 0x26, 0xcc, 0x83, 0x26, 0xa3, 0x26, 0xcc, 0xbf, 0x26, 0xd7, 0x26, 0xcc, 0xf7, 0x26, 0x0f, 0x27, 0xcc, 0x2b, 0x27, 0x3f, 0x27, 0xcc, 0x5b, 0x27, 0x7b, 0x27, 0xcc, 0x03, 0x00, 0x1b, 0x22, 0xbb, 0x37, 0x22, 0x7f, 0x22, 0xbb, 0x03, 0x00, 0xc7, 0x23, 0xbb, 0xfb, 0x23, 0x3f, 0x24, 0xcc, 0x67, 0x24, 0x03, 0x00, 0xcb, 0x03, 0x00, 0xaf, 0x29, 0xcc,
	};


	
	uint16_t size = sizeof(data1);

	for (int i = 0; i < size; i++)
	{
		testBuffer[i] = data1[i];
	}

	memset(&testNode, 0, sizeof(rplidar_response_capsule_measurement_nodes_t));
	memset(&finalLineData, 0, sizeof(rplidar_response_measurement_node_t) * 180);
	serialTestMode = true;
	testBufferIndex = 0;
	maxTextBufferIndex = size;

	uint16_t goodSamples = 0;
	bool start = true;
	while (testBufferIndex < maxTextBufferIndex)
	{
		uint16_t count = 0;
		result = loopScanExpressAddDataRTOS(start, &count);
		goodSamples += count;
		start = false;

	}
	printf("goodSamples 2 = % d \n", goodSamples);
	if (goodSamples != 61)
	{
		printf("ERROR: %s  Expecting 61 good samples, only found %d\n", __FUNCTION__, goodSamples);
		return false;
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
		result &= testRtosSyncBit();
		result &= testRtosWholePacket1();
		result &= testRtosBadGoodPacket1();
		result &= testGetLineOfData();
		result &= testGetLineOfDataErrorInMiddle();
		if (result == false)
		{
			printf("\n\n\n------------------------- ERRORS DETECTED IN %s -----------------------\n\n\n",__FUNCTION__);
		}
		return result;
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
