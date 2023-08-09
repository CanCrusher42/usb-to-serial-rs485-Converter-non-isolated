

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "lp_defines.h"
#include "inc/rplidar_cmd.h"
#include "string.h"

#include "blob.h"


//Y starts high and goes downward to 0
//X starts neg at left and goes uppward to right.

extern rplidar_response_measurement_node_t finalLineData[1 * 180];


blobStruct_t blobList[BLOBS_IN_LIST];

void ClearBlobNumber(uint16_t i)
{
	memset(&blobList[i], 0, sizeof(blobStruct_t));
}

void ClearBlobs()
{
	for (int i = 0; i < BLOBS_IN_LIST; i++)
	{
		memset(&blobList[i], 0, sizeof(blobStruct_t));
	}
}

void ClearFinalLineData()
{
	for (int i = 0; i < 180; i++)
	{
		memset(&finalLineData[i], 0, sizeof(blobStruct_t));
	}

}

void PrintBlobList()
{

}

uint16_t GetBlobCount()
{
	uint16_t count = 0;
	for (int i = 0; i < BLOBS_IN_LIST; i++)
	{
		if (blobList[i].numSamples > 0)
			count++;
	}
	return count;
}

bool IsPointInBlob(uint16_t blobNumber, int16_t x, int16_t y)
{
	if ((x >= (blobList[blobNumber].xLeft - 1)) && (x <= (blobList[blobNumber].xRight + 1)) &&
		(y >= (blobList[blobNumber].yLower - 1)) && (y <= (blobList[blobNumber].yUpper + 1)))
		return true;
	return false;

}
int16_t addPointToBlobList(int16_t x, int16_t y)
{
	bool found = false;
	uint16_t firstBlob = 99;
	for (int i = 0; i < BLOBS_IN_LIST; i++)
	{
		if (blobList[i].numSamples > 0)
		{
			if ((x >= (blobList[i].xLeft - 1)) && (x <= (blobList[i].xRight + 1)) &&
				(y >= (blobList[i].yLower - 1)) && (y <= (blobList[i].yUpper + 1)))
			{
				// Was this the first blob, this was found in?  If so, just increase the size of the blob to include this point.
				if (found == false)
				{
					if (x < blobList[i].xLeft)
						blobList[i].xLeft = x;
					else if (x > blobList[i].xRight)
						blobList[i].xRight = x;

					if (y < blobList[i].yLower)
						blobList[i].yLower = y;
					else if (y > blobList[i].yUpper)
						blobList[i].yUpper = x;
					found = true;
					firstBlob = i;
					blobList[i].numSamples++;
				}
				else   // This point is part of 2 or more blobs, merge this one into the first one found
				{
					MergeSecondBlobIntoFirst(firstBlob, i);
					ClearBlobNumber(i);
				}
			}

		}
	}

	if (found == false)
	{
		for (int i = 0; i < BLOBS_IN_LIST; i++)
		{
			if (blobList[i].numSamples == 0)
			{
				blobList[i].xLeft = blobList[i].xRight = x;
				blobList[i].yUpper = blobList[i].yLower = y;
				blobList[i].numSamples = 1;
				return i;
			}
		}
		return -1;  //Ran out of space to add it.
	} 

	return firstBlob;
}

// The new blob will combine the two 
void MergeSecondBlobIntoFirst(uint16_t firstBlob, uint16_t secondBlob)
{
	blobList[firstBlob].xLeft  = min(blobList[firstBlob].xLeft, blobList[secondBlob].xLeft);
	blobList[firstBlob].xRight = max(blobList[firstBlob].xRight, blobList[secondBlob].xRight);
	blobList[firstBlob].yUpper = max(blobList[firstBlob].yUpper, blobList[secondBlob].yUpper);
	blobList[firstBlob].yLower = min(blobList[firstBlob].yLower, blobList[secondBlob].yLower);
	ClearBlobNumber(secondBlob);
}
