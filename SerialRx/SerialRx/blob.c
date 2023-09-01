

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "inc/lp_defines.h"
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
	printf("B - ULx , ULy    LRx , LRy  Size\n");
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
	{

		if (blobList[i].numSamples > 0)
		{
			printf("%d - %3d , %3d    %3d , %3d  %4d \n", i, blobList[i].xLeft, blobList[i].yUpper, blobList[i].xRight, blobList[i].yLower, GetBlobSize(i));
		}
	}
	uint8_t largestBlob = GetLargestBlob();
	printf("Angle to largest Blob = %3f degrees  Distance = %d size = %d\n", GetAngleToBlob(largestBlob)* 57.2958F, GetDistanceToBlobCenter(largestBlob), GetBlobSize(largestBlob));



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
		*x = (blobList[blob].xLeft  + blobList[blob].xRight) / 2;
		*y = (blobList[blob].yUpper + blobList[blob].yLower) / 2;
	}

}

 
 uint16_t GetDistanceToBlobCenter(uint8_t blob)
 {
	 int16_t x = 0, y = 0;
	 GetBlobCenter(blob, &x, &y);
	 int16_t distance = (int16_t)round(sqrt(x * x + y * y));
	 return distance;
 }

 // get angle to blob in radians
float GetAngleToBlob(int8_t blob)
{ 
	int16_t centerX, centerY;

	// Compute Center of Blob
	GetBlobCenter(blob, &centerX, &centerY);
	float a = (float)centerY / (float)centerX;
	a = (float)atan(a);
	return a;


}

uint32_t GetBlobSize(uint8_t blob)
{
	uint32_t tempSize = (blobList[blob].xRight - blobList[blob].xLeft) + 1;
	tempSize *= (blobList[blob].yUpper - blobList[blob].yLower + 1);
	return tempSize;
}


uint8_t GetLargestBlob()
{
	uint32_t largestSize=0;
	
	uint8_t largestIndex=0xFF;
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
	{
		if (blobList[i].numSamples > 0)
		{
			
			uint32_t tempSize = GetBlobSize(i);
			if (tempSize > largestSize)
			{
				largestSize = tempSize;
				largestIndex = i;
			}
		}
	}
	return largestIndex;
}

void GetLargestBlobData(float *angle, uint16_t *distance)
{

	uint8_t index = GetLargestBlob();
	*angle = GetAngleToBlob(index);
	*distance = GetDistanceToBlobCenter(index);

}

#define NUM_COLS  180 
#define NUM_ROWS  50
// to calculate what xPerColumn should be, figure out the max x distance in one direction in mm.  Then devide this by (NUM_COLS/2)
// to calculate what yPerRow should be, figure out the max Y distance in mm.  Then devide this by (NUM_ROWS/2)
int CreateBlobsFromFinalLineData(uint16_t xPerColumn, uint16_t yPerRow)
{
	int16_t divX, divY;
	int16_t xValue[182] = { 0 };
	int16_t yValue[182] = { 0 };
	int16_t minX = 1000;

	float ang;

	minX = (NUM_COLS / -2) * xPerColumn;

	// Compute cartisian values
	for (uint16_t angle = 0; angle < 180; angle++)
	{
		if ((finalLineData[angle].angle_q6_checkbit >> 6) != 0)
		{
			ang = (float)(finalLineData[angle].angle_q6_checkbit >> 6);
			ang = ang * 0.0174533F;
			xValue[angle] = (int)trunc(-1.0 * cos(ang) * (float)(finalLineData[angle].distance_q2 >> 2));
			yValue[angle] = (int)trunc(       sin(ang) * (float)(finalLineData[angle].distance_q2 >> 2));
		}
		else
		{
			xValue[angle] = 0;
			yValue[angle] = 0;
		}

	}


	
	ClearBlobs();
	for (int16_t row = NUM_ROWS; row > 0; row--)
	{
		for (uint16_t col = 0; col < 180; col++)
		{
			//divX = (xValue[col] - colStart) / xPerColumn;
			divX = (xValue[col] - minX) / xPerColumn;
			divY = (yValue[col] / yPerRow);
			if (divY == row)
			{
				if ((divX < 0) || (divX > 180))
					printf("ERROR Bad DIV X %d\n\n\n\n", divX);
				else
				{
					if (finalLineData[col].sync_quality != 0)
					{
						addPointToBlobList((int8_t)divX, (int8_t)divY);
					}
				}
			}
		}
	}

	return 0;

}