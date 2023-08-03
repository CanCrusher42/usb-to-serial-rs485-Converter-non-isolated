#include "inc/rptypes.h"
#include "inc/rplidar_protocol.h"
#include "inc/rplidar_cmd.h"
#include "rplidar_driver_impl.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

int millis();
bool OpenLpLidar();


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

uint8_t lidarSerial_read();

bool rb_begin()
{
    _isConnected = OpenLpLidar();
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

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_RESET, NULL, 0))) {
            return ans;
        }
    }
    uint32_t startTime = GetTickCount();
    while ((GetTickCount() - startTime) < 20) {};
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

        uint8_t currentbyte = lidarSerial_read();
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

    //_disableDataGrabbing();

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
    _u8* nodeBuffer = (_u8*)&node;
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

    for (size_t pos = 0; pos < count; ++pos) {
        _cached_scan_node_hq_buf[_cached_scan_node_hq_count++] = nodesHq[pos];
        if (_cached_scan_node_hq_count >= _countof(_cached_scan_node_hq_buf)) {
            _cached_scan_node_hq_count = 0;
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
        int currentStartAngle_q8 = ((capsule->start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3);
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

                nodebuffer[*nodeCount++] = node;
            }

        }
    }

    _cached_previous_capsuledata = *capsule;
    _is_previous_capsuledataRdy = true;
}

/* Not Used*/
void _dense_capsuleToNormal(const rplidar_response_capsule_measurement_nodes_t* capsule, rplidar_response_measurement_node_hq_t* nodebuffer, size_t* nodeCount)
{
    const rplidar_response_dense_capsule_measurement_nodes_t* dense_capsule = capsule;
    *nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((dense_capsule->start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_dense_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 8) / 40;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_dense_capsuledata.cabins); ++pos)
        {
            int dist_q2;
            int angle_q6;
            int syncBit;
            const int dist = (const int)_cached_previous_dense_capsuledata.cabins[pos].distance;
            dist_q2 = dist << 2;
            angle_q6 = (currentAngle_raw_q16 >> 10);
            syncBit = _getSyncBitByAngle(currentAngle_raw_q16, angleInc_q16);
            currentAngle_raw_q16 += angleInc_q16;

            if (angle_q6 < 0) angle_q6 += (360 << 6);
            if (angle_q6 >= (360 << 6)) angle_q6 -= (360 << 6);



            rplidar_response_measurement_node_hq_t node;

            node.angle_z_q14 = (_u16)((angle_q6 << 8) / 90);
            node.flag = (syncBit | ((!syncBit) << 1));
            node.quality = dist_q2 ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
            node.dist_mm_q2 = dist_q2;

            nodebuffer[*nodeCount++] = node;


        }
    }

    _cached_previous_dense_capsuledata = *dense_capsule;
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


u_result stop(_u32 timeout)
{
    u_result ans;
    _disableDataGrabbing();

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_STOP,NULL, 0))) {
            return ans;
        }
    }
    return RESULT_OK;
}


void _disableDataGrabbing()
{
    _isScanning = false;
}
