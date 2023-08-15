

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

void ClearBlobNumber(uint8_t i)
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
	uint8_t count = 0;
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
	{
		if (blobList[i].numSamples > 0)
			count++;
	}
	return count;
}

bool IsPointInBlob(uint8_t blobNumber, int8_t x, int8_t y)
{
	if ((x >= (blobList[blobNumber].xLeft - 1)) && (x <= (blobList[blobNumber].xRight + 1)) &&
		(y >= (blobList[blobNumber].yLower - 1)) && (y <= (blobList[blobNumber].yUpper + 1)))
		return true;
	return false;

}
int8_t addPointToBlobList(int8_t x, int8_t y)
{
	bool found = false;
	uint8_t firstBlob = 99;
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
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
		for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
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
void MergeSecondBlobIntoFirst(uint8_t firstBlob, uint8_t secondBlob)
{
	blobList[firstBlob].xLeft  = min(blobList[firstBlob].xLeft, blobList[secondBlob].xLeft);
	blobList[firstBlob].xRight = max(blobList[firstBlob].xRight, blobList[secondBlob].xRight);
	blobList[firstBlob].yUpper = max(blobList[firstBlob].yUpper, blobList[secondBlob].yUpper);
	blobList[firstBlob].yLower = min(blobList[firstBlob].yLower, blobList[secondBlob].yLower);
	ClearBlobNumber(secondBlob);
}


 void GetBlobCenter(uint8_t blob, int16_t* x, int16_t* y)
{
	if (blobList[blob].numSamples > 0)
	{
		*x = (blobList[blob].xLeft + blobList[blob].xRight) / 2;
		*y = (blobList[blob].yUpper + blobList[blob].yLower) / 2;
	}

}



 // get angle to blob in radians
float GetAngleToBlob(int8_t blob)
{ 
	int16_t centerX, centerY;

	// Compute Center of Blob
	GetBlobCenter(blob, &centerX, &centerY);
	float a = (float)centerY / (float)centerX;
	a = atan(a);
	return a;


}

uint8_t GetLargestBlob()
{
	uint32_t largestSize=0;
	
	uint8_t largestIndex=0xFF;
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
	{
		if (blobList[i].numSamples > 0)
		{
			uint32_t tempSize = (blobList[i].xRight - blobList[i].xLeft) + 1;
			tempSize *= (blobList[i].yUpper- blobList[i].yLower +1 );
			if (tempSize > largestSize)
			{
				largestSize = tempSize;
				largestIndex = i;
			}
		}
	}
	return largestIndex;
}

int GetCmToBlobCenter()
{
	return 0;
}