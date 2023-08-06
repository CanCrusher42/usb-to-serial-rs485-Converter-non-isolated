#include "inc/rptypes.h"
#include "inc/rplidar_protocol.h"
#include "inc/rplidar_cmd.h"
#include "rplidar_driver_impl.h"
#include "lp_defines.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

int millis();
bool OpenLpLidar();
int lidarClear_serial();
// Class Variables
int maxValue = 0;

bool     _isConnected;
bool     _isSupportingMotorCtrl;
bool     _isScanning;
bool     _isTofLidar;

rplidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf[2048];
_u16                                   _cached_scan_node_hq_count;

_u16                    _cached_sampleduration_std;
_u16                    _cached_sampleduration_express;
_u8                     _cached_express_flag;
float                   _cached_current_us_per_sample;

rplidar_response_capsule_measurement_nodes_t _cached_previous_capsuledata;
rplidar_response_hq_capsule_measurement_nodes_t _cached_previous_Hqdata;
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

static void convert(rplidar_response_measurement_node_t * from, rplidar_response_measurement_node_hq_t * to)
{
    to->angle_z_q14 = (((from->angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;  //transfer to q14 Z-angle
    to->dist_mm_q2 = from->distance_q2;
    to->flag = (from->sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
    to->quality = (from->sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  //remove the last two bits and then make quality from 0-63 to 0-255
}

static void convertHQ(rplidar_response_measurement_node_hq_t* from, rplidar_response_measurement_node_t* to)
{
    to->sync_quality = (from->flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) | ((from->quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    to->angle_q6_checkbit = 1 | (((from->angle_z_q14 * 90) >> 8) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
    to->distance_q2 = from->dist_mm_q2 > 0xFFFF/*_u16(-1)*/ ? 0 /*_u16(0)*/ : from->dist_mm_q2 & 0xFFFF;
};

void lidarSerial_write(uint8_t* header, uint16_t length);

int lidarSerial_read();

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
    u_result ans;
    printf("\nRESET\n");
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

u_result _waitNode(rplidar_response_measurement_node_t* node, _u32 timeout)
{
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8* nodebuf = (_u8*)node;
    _u8 recvPos = 0;

    while ((remainingtime = millis() - currentTs) <= timeout) {
        int currentbyte = lidarSerial_read();
        if (currentbyte < 0) continue;

        switch (recvPos) {
        case 0: // expect the sync bit and its reverse in this byte          {
        {
            _u8 tmp = (currentbyte >> 1);
            if ((tmp ^ currentbyte) & 0x1) {
                // pass
            }
            else {
                continue;
            }

        }
        break;
        case 1: // expect the highest bit to be 1
        {
            if (currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                // pass
            }
            else {
                recvPos = 0;
                continue;
            }
        }
        break;
        }
        nodebuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_OK;
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}

u_result _waitCapsuledNode(rplidar_response_capsule_measurement_nodes_t *node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = millis();
    _u8  recvBuffer[sizeof(rplidar_response_capsule_measurement_nodes_t)];
    _u8* nodeBuffer = (_u8*)node;
    _u32 waitTime;

    while ((waitTime = millis() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_capsule_measurement_nodes_t) - recvPos;

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

u_result loopScanData()
{
    static uint16_t recvNodeCount = 0;
    u_result ans;
    rplidar_response_measurement_node_t node;
    rplidar_response_measurement_node_hq_t nodeHq;

    if (IS_FAIL(ans = _waitNode(&node, DEFAULT_TIMEOUT))) {
        _isScanning = false;
        return RESULT_OPERATION_FAIL;
    }

    convert(&node, &nodeHq);
    _cached_scan_node_hq_buf[_cached_scan_node_hq_count++] = nodeHq;
    if (_cached_scan_node_hq_count >= _countof(_cached_scan_node_hq_buf)) {
        _cached_scan_node_hq_count = 0;
    }

    return RESULT_OK;
}


// grab some express data and put it in the global structs _cached_scan_node_hq_buf
u_result loopScanExpressData()
{
    static uint16_t recvNodeCount = 0;
    u_result ans;
    rplidar_response_capsule_measurement_nodes_t capsule_node;
    rplidar_response_measurement_node_hq_t nodesHq[512];

    if (IS_FAIL(ans = _waitCapsuledNode(&capsule_node, DEFAULT_TIMEOUT))) {
        _isScanning = false;
        return RESULT_OPERATION_FAIL;
    }

    size_t count = 512;
    _capsuleToNormal(&capsule_node, nodesHq, &count);

    //_cached_scan_node_hq_buf is an array of the last XYZ hq buffers or samples recorded.
    // Basically, this code is decoding the last capture and storing it for later use.
    // If I detect that no data here is in the 0-180 degree, I could set a variable saying "collecting un usable data"
    // If I only want to store 0-180 degree data at any resoulition, this is where I would add it.
    // there are 2048 _cached_scan_node_hq_buf values currently.

    for (size_t pos = 0; pos < count; ++pos) {
        _cached_scan_node_hq_buf[_cached_scan_node_hq_count++] = nodesHq[pos];
        if ((nodesHq[pos].dist_mm_q2 > 0) && (finalLineData[nodesHq[pos].angle_z_q14 >> 6].distance_q2 ==0))
        {
            if ((nodesHq[pos].angle_z_q14 >> 6) < 180)
            {
                finalLineData[nodesHq[pos].angle_z_q14 >> 6].distance_q2 = nodesHq[pos].dist_mm_q2;
                finalLineData[nodesHq[pos].angle_z_q14 >> 6].angle_q6_checkbit = nodesHq[pos].angle_z_q14;
            }
        }
        if (_cached_scan_node_hq_count >= _countof(_cached_scan_node_hq_buf)) {
            _cached_scan_node_hq_count = 0;
        }
    }
    return RESULT_OK;
}

// grab some express data and put it in the global structs _cached_scan_node_hq_buf
u_result loopScanExpressData6()
{
    static uint16_t recvNodeCount = 0;
    u_result ans;
    rplidar_response_capsule_measurement_nodes_t capsule_node;
    rplidar_response_measurement_node_t nodes[32];

    if (IS_FAIL(ans = _waitCapsuledNode(&capsule_node, DEFAULT_TIMEOUT))) {
        _isScanning = false;
        return RESULT_OPERATION_FAIL;
    }

    size_t count = 0;

    _capsuleToNormal16(&capsule_node, nodes, &count);

    for (size_t pos = 0; pos < count; ++pos) {

        if ((nodes[pos].distance_q2 > 0) && (finalLineData[nodes[pos].angle_q6_checkbit >> 6].distance_q2 == 0))
        {
            int angle= nodes[pos].angle_q6_checkbit >> 6;
            if (angle < 180)
            {
                finalLineData[angle].distance_q2 = nodes[pos].distance_q2;
                finalLineData[angle].angle_q6_checkbit = nodes[pos].angle_q6_checkbit;
            }
        }
    }
    return RESULT_OK;
}



u_result grabScanData(rplidar_response_measurement_node_hq_t* nodebuffer, size_t * count, _u32 timeout)
{
    if (_cached_scan_node_hq_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout
    size_t size_to_copy = min(*count, _cached_scan_node_hq_count);
    memcpy(nodebuffer, _cached_scan_node_hq_buf, _cached_scan_node_hq_count * sizeof(rplidar_response_measurement_node_hq_t));
    *count = size_to_copy;
    _cached_scan_node_hq_count = 0;
    return RESULT_OK;
}
u_result grabScanExpressData(rplidar_response_measurement_node_hq_t* nodebuffer, size_t* count, _u32 timeout)
{
    return grabScanData(nodebuffer, count, timeout);
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



void _capsuleToNormal(rplidar_response_capsule_measurement_nodes_t* capsule, rplidar_response_measurement_node_hq_t* nodebuffer, size_t* nodeCount)
{
    *nodeCount = 0;

    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        // Convert angle to 1 degree = 256
        int currentStartAngle_q8 = ((capsule->start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        if (prevStartAngle_q8 > currentStartAngle_q8)
        {
            printf("*");
            scans++;
        }
        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3);  // each capsal is divided into 32 and this << 3 = a total of << 8
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos)
        {
            int dist_q2[2];
            int angle_q16[2];
            int syncBit[2] = { 0,0 };

            dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
            dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

            int angle_offset1_q3 = ((_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3) << 4));
            int angle_offset2_q3 = ((_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3) << 4));

            int syncBit_check_threshold = (int)((2 << 16) / angleInc_q16) + 1;//find syncBit in 0~1 degree

            angle_q16[0] = (currentAngle_raw_q16 - (angle_offset1_q3 << 13));
            syncBit[0] = _getSyncBitByAngle(currentAngle_raw_q16, angleInc_q16);
            currentAngle_raw_q16 += angleInc_q16;

            angle_q16[1] = (currentAngle_raw_q16 - (angle_offset2_q3 << 13));
            syncBit[1] = _getSyncBitByAngle(currentAngle_raw_q16, angleInc_q16);
            currentAngle_raw_q16 += angleInc_q16;

            for (int cpos = 0; cpos < 2; ++cpos) {

                if (angle_q16[cpos] < 0) angle_q16[cpos] += (360 << 16);
                if (angle_q16[cpos] >= (360 << 16)) angle_q16[cpos] -= (360 << 16);

                rplidar_response_measurement_node_hq_t node;

                node.angle_z_q14 = (_u16)((angle_q16[cpos] >> 2) / 90);
                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[*nodeCount] = node;
                *nodeCount = *nodeCount + 1;
                if ((node.angle_z_q14>>9) > maxValue)
                    maxValue = node.angle_z_q14;

            }

        }
    }

    _cached_previous_capsuledata = *capsule;
    _is_previous_capsuledataRdy = true;
}


void _capsuleToNormal16(rplidar_response_capsule_measurement_nodes_t* capsule, rplidar_response_measurement_node_t* nodebuffer, size_t* nodeCount)
{
    *nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int16_t diffAngle_q6;
        int16_t currentStartAngle_q6 = ((capsule->start_angle_sync_q6 & 0x7FFF) );
        int16_t prevStartAngle_q6 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF));

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
