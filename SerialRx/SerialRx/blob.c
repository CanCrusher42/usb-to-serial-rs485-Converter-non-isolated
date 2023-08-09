

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "lp_defines.h"
#include "inc/rplidar_cmd.h"
#include "string.h"

#include "blob.h"

extern rplidar_response_measurement_node_t finalLineData[1 * 180];


blobStruct_t blobs[BLOBS_IN_LIST];

void ClearBlobs()
{
	for (int i = 0; i < BLOBS_IN_LIST; i++)
	{
		memset(&blobs[i], 0, sizeof(blobStruct_t));
	}
}

void ClearFinalLineData()
{
	for (int i = 0; i < 180; i++)
	{
		memset(&finalLineData[i], 0, sizeof(blobStruct_t));
	}

}
