#include "inc/rptypes.h"
#include "inc/rplidar_protocol.h"
#include "inc/rplidar_cmd.h"
#include "rplidar_driver_impl.h"

#include <stdbool.h>
#include <stdio.h>

int millis();
bool OpenLpLidar();

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


