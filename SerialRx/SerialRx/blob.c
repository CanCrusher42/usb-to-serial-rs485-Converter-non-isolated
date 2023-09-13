

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "inc/lp_defines.h"
#include "inc/rplidar_cmd.h"
#include "string.h"
#ifdef __XC16__
#include "min_max.h"
#endif

#include "blob.h"

#include "lidar_conf.h"


extern     int16_t yPerRow, xPerColumn, absBounds;
//Y starts high and goes downward to 0
//X starts neg at left and goes uppward to right.

extern rplidar_response_measurement_node_xy_t finalLineData[1 * 180];


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
	printf("B -  ULx ,         ULy   LRx ,       LRy  Size\n");
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
	{

		if (blobList[i].numSamples > 0)
		{
			printf("%d - %3d(%3d) ,   %3d   %3d(%3d) ,    %3d  %4d \n", i, blobList[i].xLeft, blobList[i].xLeft-90, blobList[i].yUpper, blobList[i].xRight, blobList[i].xRight-90,  blobList[i].yLower, GetBlobSize(i));
		}
	}
	uint8_t largestBlob = GetLargestBlob();
	printf("Angle to largest Blob = %3f (%3f) degrees  Distance = %d(%d mm) size = %d\n", (double)(GetAngleToBlob(largestBlob) * 57.2958F), (double)(GetRealAngleToBlob(largestBlob) * 57.2958F), GetDistanceToBlobCenter(largestBlob), GetRealDistanceToBlobCenter(largestBlob), GetBlobSize(largestBlob));
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
 //yPerRow, xPerColumn
 uint16_t GetRealDistanceToBlobCenter(uint8_t blob)
 {
	 int16_t x = 0, y = 0;
	 int16_t distance;
	 GetBlobCenter(blob, &x, &y);
	 if ((yPerRow != 0) && (xPerColumn != 0))
	 {
		 int32_t x1 = ((uint32_t)(x-90) * (uint32_t)xPerColumn);
		 int32_t y1 = ((uint32_t)y * (uint32_t)yPerRow);
		 distance = (int16_t)round(sqrt(x1 * x1 + y1 * y1));
	 }
	 else
	 {
		 distance = GetDistanceToBlobCenter(blob);
	 }
	 return distance;
 }

 // get angle to blob in radians
float GetAngleToBlob(int8_t blob)
{ 
	int16_t centerX, centerY;

	// Compute Center of Blob
	GetBlobCenter(blob, &centerX, &centerY);
	float a = (float)centerY / (float)(centerX);
	a = (float)atan(a);
	return a;
}

// get angle to blob in radians
float GetRealAngleToBlob(int8_t blob)
{
	int16_t centerX, centerY;

	if ((yPerRow != 0) && (xPerColumn != 0))
	{
		// Compute Center of Blob
		GetBlobCenter(blob, &centerX, &centerY);

		float a = (float)(centerY*yPerRow) / (float)((centerX - 90) * xPerColumn);
		a = (float)atan(a);
		return a;
	}
	else
		return GetAngleToBlob(blob);
}


uint16_t GetBlobSize(uint8_t blob)
{
	uint16_t tempSize = (blobList[blob].xRight - blobList[blob].xLeft) + 1;
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
//#define LIDAR_BLOB_ROWS        20U
//#define LIDAR_BLOB_COLUMNS     180U


// to calculate what xPerColumn should be, figure out the max x distance in one direction in mm.  Then devide this by (NUM_COLS/2)
// to calculate what yPerRow should be, figure out the max Y distance in mm.  Then devide this by (NUM_ROWS/2)

int CreateBlobsFromFinalLineData(uint16_t xPerColumn, uint16_t yPerRow)
{
    int16_t divX;
    uint16_t divY;
    int16_t minX = 1000;

    minX = ((int)LIDAR_BLOB_COLUMNS /  -2) * (int)xPerColumn;

    ClearBlobs();
    for (int16_t row = LIDAR_BLOB_ROWS; row > 0; row--)
    {
        for (uint16_t col = 0; col < 180; col++)
        {
            // If its not in the blob area, ignore it.
            if (finalLineData[col].sync_quality != 0)  
            {
                divY = (finalLineData[col].y / yPerRow);
                if (divY == row)
                {
                    divX = (finalLineData[col].x - minX) / xPerColumn;
                    if ((divX >= 0) && (divX <= 180))
                    {
                            addPointToBlobList((int8_t) divX, (int8_t) divY);
                    } else
                    {
                        //printf("ERROR Bad DIV X %d\n\n\n\n", divX);
                    }

                }
            }
        }
    }

    return 0;
}