#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "inc/rptypes.h"
#include "inc/rplidar_protocol.h"
#include "inc/rplidar_cmd.h"
#include "inc/rplidar_driver_impl.h"
#include "inc/lp_defines.h"
#include "serial.h"

#include <math.h>


uint32_t GetTickCount();;
int millis();
bool OpenLpLidar();
int lidarClear_serial();
// Class Variables
int maxValue = 0;
int16_t insideX_left = -3000;  // cm  100cm to the meter
int16_t insideX_right = 3000;   // cm
int16_t inside_up = 2070;   // 150 at home, Hall by desk is 9 foot steps in sandles  (30cm per)  270 cm at work



bool     _isConnected;
bool     _isScanning;
bool     _isTofLidar;

_u8                     _cached_express_flag;

rplidar_response_capsule_measurement_nodes_t _cached_previous_capsuledata;
bool                                         _is_previous_capsuledataRdy;
bool                                         _is_previous_HqdataRdy;
bool                                         _syncBit_is_finded;

rplidar_response_measurement_node_t finalLineData[SAMPLES_PER_DEGREE * 180];

uint16_t _magValue[180 * 2];
uint16_t _angValue[180 * 2];
uint16_t scans = 0;
#if !defined(_countof)
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

bool rb_begin()
{
    printf("\nOPEN\n");
    _isConnected = OpenLpLidar();
    uint32_t startTime = GetTickCount();
    while ((GetTickCount() - startTime) < 100);
    return _isConnected;
}


bool isConnected()
{
    return _isConnected;
}

bool isScanning()
{
    return _isScanning;
}

u_result reset(_u16 timeout)
{
 
//    printf("\nRESET\n");
    //{
    //    if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_RESET, NULL, 0))) {
    //        return ans;
    //    }
    //}
    uint32_t startTime = GetTickCount();
    while ((GetTickCount() - startTime) < 10) {};
    return RESULT_OK;
}

u_result _waitResponseHeader(rplidar_ans_header_t* header, _u32 timeout)
{
    _u8  recvPos = 0;
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8* headerbuf = (_u8*)header;
    char test[150] = { 0 };
    int index = 0;
    while ((remainingtime = millis() - currentTs) <= timeout) {

        int  currentbyte = lidarSerial_read();
        test[index++] = currentbyte;
        if (currentbyte < 0) continue;

        switch (recvPos) {
        case 0:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                recvPos = 0;
                continue;
            }
            break;
        }
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
            return RESULT_OK;
        }
    }

    printf("waitReponseHeader timeout");

    return RESULT_OPERATION_TIMEOUT;
}

u_result getHealth(rplidar_response_device_health_t * healthinfo, _u32 timeout)
{
    u_result  ans;

    if (!isConnected()) return RESULT_OPERATION_FAIL;

    //_disableDataGrabbing();

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }

        //  if (!_chanDev->waitfordata(header_size, timeout)) {
        //     return RESULT_OPERATION_TIMEOUT;
        // }
        // _chanDev->recvdata(reinterpret_cast<_u8 *>(&healthinfo), sizeof(healthinfo));
        _u8* infobuf = (_u8*)healthinfo;
        _u8  recvPos = 0;
        _u32 currentTs = millis();
        _u32 remainingtime;
        while ((remainingtime = millis() - currentTs) <= timeout) {
            int currentbyte = lidarSerial_read();
            if (currentbyte < 0) continue;

            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_health_t)) {
                return RESULT_OK;
            }
        }
    }
    return RESULT_OK;
}

u_result getDeviceInfo(rplidar_response_device_info_t* info, _u32 timeout)
{
    u_result  ans;

    if (!isConnected()) return RESULT_OPERATION_FAIL;

    _disableDataGrabbing();

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO,NULL, 0))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        // if (!_chanDev->waitfordata(header_size, timeout)) {
        //     return RESULT_OPERATION_TIMEOUT;
        // }
        // _chanDev->recvdata(reinterpret_cast<_u8 *>(&info), sizeof(info));
        _u8* infobuf = (_u8*)info;
        _u32 remainingtime;
        _u32 currentTs = millis();
        _u8  recvPos = 0;
        while ((remainingtime = millis() - currentTs) <= timeout) {
            int currentbyte = lidarSerial_read();
            if (currentbyte < 0) continue;
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
        if ((info->model >> 4) > RPLIDAR_TOF_MINUM_MAJOR_ID) {
            _isTofLidar = true;
        }
        else {
            _isTofLidar = false;
        }
    }
    return RESULT_OK;
}


u_result _waitCapsuledNodeRTOS(rplidar_response_capsule_measurement_nodes_t* node, bool  new)
{
	static _u8  recvPos = 0;

    _u8* nodeBuffer = (_u8*)node;
	int16_t currentbyte;
	_u8 currentByte;
    u_result result = RESULT_WAITING;

	if (new)
	{
		recvPos = 0;
	}


	while (1)
	{
		currentbyte = lidarSerial_read();
		if (currentbyte < 0)
			return RESULT_WAITING;


		currentByte = currentbyte & 0xFF;

		switch (recvPos) {
		case 0: // expect the sync bit 1
		{
			_u8 tmp = (currentByte >> 4);
			if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
				// pass
			}
			else {
                // Something happened to data stream, resync state machines.
                recvPos = 0;
				_is_previous_capsuledataRdy = false;
				result = RESULT_WAITING;
			}
		}
		break;
		case 1: // expect the sync bit 2
		{
			_u8 tmp = (currentByte >> 4);
			if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
				// pass
			}
			else {
                // Something happened to data stream, resync state machines.
				recvPos = 0;
				_is_previous_capsuledataRdy = false;
				result = RESULT_WAITING;
			}
		}
		break;
		}

		nodeBuffer[recvPos++] = currentByte;
		if (recvPos == sizeof(rplidar_response_capsule_measurement_nodes_t)) {
			// calc the checksum ...
			_u8 checksum = 0;
			_u8 recvChecksum = ((node->s_checksum_1 & 0xF) | (node->s_checksum_2 << 4));
			for (size_t cpos = 2;  // Not supported in C  offsetof(rplidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
				cpos < sizeof(rplidar_response_capsule_measurement_nodes_t); ++cpos)
			{
				checksum ^= nodeBuffer[cpos];
			}
            if (recvChecksum == checksum)
            {
                // only consider vaild if the checksum matches...
                if (node->start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
                {
                    // this is the first capsule frame in logic, discard the previous cached data...
                    _is_previous_capsuledataRdy = false;
                    recvPos = 0;
                    return RESULT_OK;
                }
                recvPos = 0;
                return RESULT_OK;
            }
            else
            {
                // Something happened to data stream, resync state machines.
                _is_previous_capsuledataRdy = false;
                recvPos = 0;
                result = RESULT_INVALID_DATA;
            }
		}
	}
	return RESULT_WAITING;
}

#pragma message("Move this to correct location")
rplidar_response_capsule_measurement_nodes_t gCapsule_node;


// Get Capsule data if needed and return the number added.

// 1)  Check to see if there is a valid capsule of data availible. - Capsule will be stored in gCapsule_node
// 2)  Convert raw capsule data to an array of 32 data points   nodes[]
// 3)  Scan each of the 33 nodes to check for valid data & non-duplicate data.  If so, Calculate cartisian coordiates and use this to determine if its in the active box. Increment sample count.
// 
u_result loopScanExpressAddDataRTOS(bool start, uint16_t *sampleCount, uint16_t rotate)
{
    //    static uint16_t recvNodeCount = 0;
    u_result ans;

    rplidar_response_measurement_node_t nodes[32];
    int16_t x, y;

    float ang;

    *sampleCount = 0;


    // Get new data and place it in gCapsule_node.  Notify when capsule is complete. 
    if  ((ans = _waitCapsuledNodeRTOS( &gCapsule_node, start)) == RESULT_WAITING) {
       // _isScanning = false;
        return RESULT_WAITING;
    }

    size_t count = 0;

    // Convert Capsule to line format IFF gCapsuleNode is valid and there is a previous cacheline
    _capsuleToNormal16(&gCapsule_node, nodes, &count,rotate);

    // If there is not a previous cacheline count will equal 0.
    for (size_t pos = 0; pos < count; ++pos) 
    {
        // If this sample > 0 and there is no previous sample at this location, then add it.
        if ((nodes[pos].distance_q2 > 0) && (finalLineData[nodes[pos].angle_q6_checkbit >> 6].distance_q2 == 0))
        {

#ifndef ROTATE_DATA
            int angle = nodes[pos].angle_q6_checkbit >> 6;
            if (angle < 180)
            {
                finalLineData[angle].distance_q2 = nodes[pos].distance_q2;
                finalLineData[angle].angle_q6_checkbit = nodes[pos].angle_q6_checkbit;
                ang = (float)(finalLineData[angle].angle_q6_checkbit >> 6);
                ang = ang * (float)0.0174533;
                // If this box is in the active area mark it.
                x = (int)trunc(-1.0 * cos(ang) * (float)(finalLineData[angle].distance_q2 >> 2));
                y = (int)trunc(sin(ang) * (float)(finalLineData[angle].distance_q2 >> 2));

                if ((insideX_left < x) && (x < insideX_right) && (y < inside_up))
                {
                    finalLineData[angle].sync_quality = 1;
                }
                *sampleCount = *sampleCount+1;
            }
#else
            // WE NEED TO ROTATE 270-90 to 0 - 180 (Rotate clockwise 90)
            // add 90 to value
            // do modulus 360 to get remainder
            // (280 + 90) = 370
            // 370 % 360 = 10
            // (10+90) = 100
            // 100 % 360 = 100

            int16_t angle = ( (nodes[pos].angle_q6_checkbit & 0x7FFF) + (((uint16_t) 90) << 6)) >> 6;
            angle = ang % 360;
            if (angle < 180)
            {
                finalLineData[angle].distance_q2 = nodes[pos].distance_q2;
                finalLineData[angle].angle_q6_checkbit = nodes[pos].angle_q6_checkbit;
                ang = (float)angle;
                //ang = (float)(finalLineData[angle].angle_q6_checkbit >> 6);
                ang = ang * (float)0.0174533;
                // If this box is in the active area mark it.
                x = (int)trunc(-1.0 * cos(ang) * (float)(finalLineData[angle].distance_q2 >> 2));
                y = (int)trunc(sin(ang) * (float)(finalLineData[angle].distance_q2 >> 2));

                if ((insideX_left < x) && (x < insideX_right) && (y < inside_up))
                {
                    finalLineData[angle].sync_quality = 1;
                }
                *sampleCount = *sampleCount + 1;
            }


#endif


        }
    }

    return RESULT_OK;
}



u_result _waitCapsuledNode(rplidar_response_capsule_measurement_nodes_t *node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = millis();
    _u8  recvBuffer[8];
    _u8* nodeBuffer = (_u8*)node;
    _u32 waitTime;

    while ((waitTime = millis() - startTs) <= timeout) {

        int currentbyte = lidarSerial_read();
        if (currentbyte < 0) continue;
        recvBuffer[0] = currentbyte;
        size_t recvSize = 1;

        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];

            switch (recvPos) {
            case 0: // expect the sync bit 1
            {
                _u8 tmp = (currentByte >> 4);
                if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
                    // pass
                }
                else {
                    _is_previous_capsuledataRdy = false;
                    continue;
                }

            }
            break;
            case 1: // expect the sync bit 2
            {
                _u8 tmp = (currentByte >> 4);
                if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                    // pass
                }
                else {
                    recvPos = 0;
                    _is_previous_capsuledataRdy = false;
                    continue;
                }
            }
            break;
            }
            nodeBuffer[recvPos++] = currentByte;
            if (recvPos == sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                // calc the checksum ...
                _u8 checksum = 0;
                _u8 recvChecksum = ((node->s_checksum_1 & 0xF) | (node->s_checksum_2 << 4));
                for (size_t cpos = 2;  // Not supported in C  offsetof(rplidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
                    cpos < sizeof(rplidar_response_capsule_measurement_nodes_t); ++cpos)
                {
                    checksum ^= nodeBuffer[cpos];
                }
                if (recvChecksum == checksum)
                {
                    // only consider vaild if the checksum matches...
                    if (node->start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
                    {
                        // this is the first capsule frame in logic, discard the previous cached data...
                        _is_previous_capsuledataRdy = false;
                        return RESULT_OK;
                    }
                    return RESULT_OK;
                }
                _is_previous_capsuledataRdy = false;
                return RESULT_INVALID_DATA;
            }
        }
    }
    _is_previous_capsuledataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}
 

// grab some express data and put it in the global structs _cached_scan_node_hq_buf
// Inside box.

u_result loopScanExpressData6()
{
//    static uint16_t recvNodeCount = 0;
    u_result ans;
    rplidar_response_capsule_measurement_nodes_t capsule_node;
    rplidar_response_measurement_node_t nodes[32];
    int16_t x, y;
    float ang;

    
    if (IS_FAIL(ans = _waitCapsuledNode(&capsule_node, DEFAULT_TIMEOUT))) {
        _isScanning = false;
        return RESULT_OPERATION_FAIL;
    }

    size_t count = 0;

    _capsuleToNormal16(&capsule_node, nodes, &count, 0);

    for (size_t pos = 0; pos < count; ++pos) {

        if ((nodes[pos].distance_q2 > 0) && (finalLineData[nodes[pos].angle_q6_checkbit >> 6].distance_q2 == 0))
        {
            int angle= nodes[pos].angle_q6_checkbit >> 6;
            if (angle < 180)
            {
                finalLineData[angle].distance_q2 = nodes[pos].distance_q2;
                finalLineData[angle].angle_q6_checkbit = nodes[pos].angle_q6_checkbit;
                ang = (float)(finalLineData[angle].angle_q6_checkbit >> 6);
                ang = ang * (float)0.0174533;
                // If this box is in the active area mark it.
                x = (int)trunc(-1.0 * cos(ang) * (float)(finalLineData[angle].distance_q2 >> 2));
                y = (int)trunc(sin(ang) * (float)(finalLineData[angle].distance_q2 >> 2));

                if ((insideX_left < x) && (x < insideX_right) && (y < inside_up))
                {
                    finalLineData[angle].sync_quality = 1;
                } 

      
            }
        }
    }
    return RESULT_OK;
}

u_result startScanNormal(bool force, _u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(100); //force the previous operation to stop

    {
        if (IS_FAIL(ans = _sendCommand(force ? RPLIDAR_CMD_FORCE_SCAN : RPLIDAR_CMD_SCAN, NULL, 0))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }

        _isScanning = true;
        // _cachethread = CLASS_THREAD(RPLidar, _cacheScanData);
        // if (_cachethread.getHandle() == 0) {
        //     return RESULT_OPERATION_FAIL;
        // }
    }
    return RESULT_OK;
}

// TODO not implemented
/*
int _getSyncBitByAngle(const int current_angle_q16, const int angleInc_q16)
{
    static int last_angleInc_q16 = 0;
    int current_angleInc_q16 = angleInc_q16;
    int syncBit_check_threshold = (int)((5 << 16) / angleInc_q16) + 1;//find syncBit in 0~3 degree
    int syncBit = 0;
    int predict_angle_q16 = (current_angle_q16 + angleInc_q16) % (360 << 16);

    if (predict_angle_q16 < 0) {
        predict_angle_q16 += (360 << 16);
    }
    if (!_syncBit_is_finded)
    {
        if (0 < predict_angle_q16 && predict_angle_q16 < (90 << 16))
            syncBit = 1;
        if (syncBit)
            _syncBit_is_finded = true;
    }
    else
    {
        if (predict_angle_q16 > (270 << 16))
            _syncBit_is_finded = false;
        //if (predict_angle_q16 > (syncBit_check_threshold * angleInc_q16)) {
        //    _is_previous_syncBit = false;
        //}
    }
    last_angleInc_q16 = current_angleInc_q16;
    return syncBit;
}
*/

// 
void _capsuleToNormal16(rplidar_response_capsule_measurement_nodes_t* capsule, rplidar_response_measurement_node_t* nodebuffer, size_t* nodeCount, uint16_t rotate)
{
    *nodeCount = 0;
    
    //_is_previous_capsuledataRdy is updated at the end of this function.  
    // If we make it, it then means we have gotten the first packet of good data.
    if (_is_previous_capsuledataRdy) {
        uint16_t diffAngle_q6;
//        int16_t currentStartAngle_q6 = ((capsule->start_angle_sync_q6 & 0x7FFF));
        uint16_t currentStartAngle_q6 ;

        uint16_t prevStartAngle_q6 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF));

        uint16_t testAngle = capsule->start_angle_sync_q6 >> 6;
        if (rotate)
        {
            currentStartAngle_q6 = ((capsule->start_angle_sync_q6 & 0x7FFF) + (rotate << 6)) % (((uint16_t)360) << 6);
        }
        else
        {
            currentStartAngle_q6 = ((capsule->start_angle_sync_q6 & 0x7FFF));
        }

        uint16_t testAngle2 = currentStartAngle_q6 >> 6;
        diffAngle_q6 = currentStartAngle_q6-prevStartAngle_q6;
        if (prevStartAngle_q6 > currentStartAngle_q6) {
            diffAngle_q6 += (360 << 6);
            printf("*");
            scans++;
        }

        int16_t angleInc_q6 = (diffAngle_q6 / 32);  // 5 bits

        int16_t currentAngle_raw_q6 = prevStartAngle_q6;
        for (size_t pos = 0; pos < 16; ++pos)
        {
            int16_t dist_q2[2];
            int16_t angle_q6[2];

//            typedef struct _rplidar_response_cabin_nodes_t {
//                _u16   distance_angle_1; // see [distance_sync flags]
//                _u16   distance_angle_2; // see [distance_sync flags]
//                _u8    offset_angles_q3;
//            } __attribute__((packed)) rplidar_response_cabin_nodes_t;


            dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
            dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

            int16_t angle_offset1_q3 = ((_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3) << 4));
            int16_t angle_offset2_q3 = ((_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3) << 4));
            // sign extend 
            if (angle_offset1_q3 & 0x20)
                angle_offset1_q3 |= 0xFFC0;
            if (angle_offset2_q3 & 0x20)
                angle_offset2_q3 |= 0xFFC0;


            angle_q6[0] = (currentAngle_raw_q6 - (angle_offset1_q3<<3 ));
            currentAngle_raw_q6 += angleInc_q6;

            angle_q6[1] = (currentAngle_raw_q6 - (angle_offset2_q3 << 3));
            currentAngle_raw_q6 += angleInc_q6;

            for (int8_t cpos = 0; cpos < 2; ++cpos) {

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                //rplidar_response_measurement_node_hq_t node;
                rplidar_response_measurement_node_t node;
                node.angle_q6_checkbit = (_u16)(angle_q6[cpos]);
                node.sync_quality = dist_q2[cpos] ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.distance_q2 = dist_q2[cpos];

                nodebuffer[*nodeCount] = node;
                *nodeCount = *nodeCount + 1;
            }
        }
    }

    // Copy data to the cached version
    _cached_previous_capsuledata = *capsule;
    _is_previous_capsuledataRdy = true;
}






u_result _sendCommand(_u8 cmd, void* payload, size_t payloadsize)
{
    _u8 pkt_header[10];
    rplidar_cmd_packet_t* header = (rplidar_cmd_packet_t * )pkt_header;
    _u8 checksum = 0;

    if (!_isConnected) return RESULT_OPERATION_FAIL;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    lidarSerial_write((uint8_t*)header, 2);

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (_u16 pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8*)payload)[pos];
        }
        // send size
        _u8 sizebyte = (_u8)payloadsize;
        lidarSerial_write((uint8_t*)&sizebyte, 1);

        // send payload
        lidarSerial_write((uint8_t*)payload, sizebyte);

        // send checksum
        lidarSerial_write((uint8_t*)&checksum, 1);
    }

    return RESULT_OK;
}

u_result startScanExpress(bool force, _u16 scanMode, _u32 options, struct RplidarScanMode* outUsedScanMode, _u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(50); //force the previous operation to stop

  //  if (scanMode == RPLIDAR_CONF_SCAN_COMMAND_STD)
  //  {
  //      return startScanNormal(force);
  //  }


    //get scan answer type to specify how to wait data
    _u8 scanAnsType;
    scanAnsType = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;

    {
        rplidar_payload_express_scan_t scanReq;
        memset(&scanReq, 0, sizeof(scanReq));
        if (scanMode != RPLIDAR_CONF_SCAN_COMMAND_STD && scanMode != RPLIDAR_CONF_SCAN_COMMAND_EXPRESS)
            scanReq.working_mode = (_u8)scanMode;
        scanReq.working_flags = options;

        char report[30];
        snprintf(report, sizeof(report), "%d", scanMode);
        printf(report);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq)))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != scanAnsType) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);

        if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED)
        {
            if (header_size < sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _cached_express_flag = 0;
            _isScanning = true;
            // _cachethread = CLASS_THREAD(RPLidar, _cacheCapsuledScanData);
        }
        else if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED)
        {
            if (header_size < sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _cached_express_flag = 1;
            _isScanning = true;
            // _cachethread = CLASS_THREAD(RPLidar, _cacheCapsuledScanData);
        }
        else if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_HQ) {
            if (header_size < sizeof(rplidar_response_hq_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _isScanning = true;
            // _cachethread = CLASS_THREAD(RPLidar, _cacheHqScanData);
        }
        else
        {
            if (header_size < sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _isScanning = true;
            return RESULT_INVALID_DATA;
            printf("Not implemented");
        }
    }
    return RESULT_OK;
}


u_result stop(_u32 timeout)
{
    u_result ans;
    _disableDataGrabbing();

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_STOP,NULL, 0))) {
            return ans;
        }
        lidarClear_serial();

    }


    return RESULT_OK;
}


void _disableDataGrabbing()
{
    _isScanning = false;
}
