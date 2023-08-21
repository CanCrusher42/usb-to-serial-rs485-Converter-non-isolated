/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once
#ifndef RPLIDAR_DRIVER_H
#define RPLIDAR_DRIVER_H
#include <stdbool.h>
#include "rptypes.h"
#include "rplidar_cmd.h"

#if defined (_WIN32)
#define NULL (void*)0
#endif

struct RplidarScanMode {
    _u16    id;
    float   us_per_sample;   // microseconds per sample
    float   max_distance;    // max distance
    _u8     ans_type;         // the answer type of the scam mode, its value should be RPLIDAR_ANS_TYPE_MEASUREMENT*
    char    scan_mode[64];    // name of scan mode, max 63 characters
};



    enum {
        RPLIDAR_TOF_MINUM_MAJOR_ID = 5,
    };
    enum {
        RPLIDAR_SERIAL_BAUDRATE = 115200,  
        DEFAULT_TIMEOUT = 500,
    };
    enum {
        MAX_SCAN_NODES = 8192,
    };

    bool rb_begin();     
    bool isScanning();     
    bool isConnected();     
    u_result reset(_u16);

    u_result getHealth(rplidar_response_device_health_t* health, _u32 Timeout);
    u_result getDeviceInfo(rplidar_response_device_info_t* info, _u32 Timeout);

    u_result startScanExpress(bool force, _u16 scanMode, _u32 options, struct RplidarScanMode* outUsedScanMode , _u32 timeout );
    u_result startScanNormal(bool force, _u32 timeout );
    u_result stop(_u32 timeout);
    u_result loopScanExpressData6();
    u_result loopScanExpressAddDataRTOS(bool start, uint16_t* sampleCount);

    u_result _sendCommand(_u8 cmd, void * payload , size_t payloadsize );

    void     _disableDataGrabbing();
    u_result _waitCapsuledNode(rplidar_response_capsule_measurement_nodes_t * node, _u32 timeout);
    void     _capsuleToNormal16(rplidar_response_capsule_measurement_nodes_t* capsule, rplidar_response_measurement_node_t* nodebuffer, size_t* nodeCount);


#endif