

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

extern LidarDefinesStruct LidarDefines;
extern  int16_t yPerRow, xPerColumn, absBounds;

//Y starts high and goes downward to 0
//X starts neg at left and goes uppward to right.

extern rplidar_response_measurement_node_xy_t finalLineData[1 * 180];
int16_t minX;


blobStruct_t blobList[BLOBS_IN_LIST];
blobDetailStruct_t blobDetails[BLOBS_IN_LIST];

void ClearBlobNumber(uint8_t i)
{
	memset(&blobList[i], 0, sizeof(blobStruct_t));
	// To make the search go faster, we need a active blob count
	// Then also pack the arrary each time we remove the link.
    
}

void ClearBlobs()
{
    memset(&blobList[0],0, sizeof(blobStruct_t)* BLOBS_IN_LIST);
    
}

void ClearFinalLineData()
{
    memset(&finalLineData[0],0,sizeof(blobStruct_t)*180 );
}

void PrintBlobList()
{
	printf("B -  ULx ,      ULy      Size\n");
//    printf("B -  ULx ,      ULy   LRx ,       LRy    Size\n");
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
	{

		if (blobList[i].numSamples > 0)
		{
            printf("%d - %3d(%3d) ,   %3d   %4d   %4.1f   %d \n", i, blobList[i].xLeft, blobList[i].xLeft-90, blobList[i].yUpper,  GetBlobSize(i), GetRealAngleToBlob(i)* 57.2958F,  GetRealDistanceToBlobCenter(i));
//			printf("%d - %3d(%3d) ,   %3d   %3d(%3d) ,    %3d  %4d \n", i, blobList[i].xLeft, blobList[i].xLeft-90, blobList[i].yUpper, blobList[i].xRight, blobList[i].xRight-90,  blobList[i].yLower, GetBlobSize(i));
		}
	}
	uint8_t largestBlob = GetLargestBlob();
    
	printf("Angle to largest Blob = %3f degrees  Distance = (%d mm) size = %d\n", (double)(GetRealAngleToBlob(largestBlob) * 57.2958F), GetRealDistanceToBlobCenter(largestBlob), GetBlobSize(largestBlob));
    //printf("Angle to largest Blob = %3f (%3f) degrees  Distance = %d(%d mm) size = %d\n", (double)(GetAngleToBlob(largestBlob) * 57.2958F), (double)(GetRealAngleToBlob(largestBlob) * 57.2958F), GetDistanceToBlobCenter(largestBlob), GetRealDistanceToBlobCenter(largestBlob), GetBlobSize(largestBlob));    
}

int16_t GetRealX(int16_t shiftedX)
{
    return (shiftedX-90)* LidarDefines.BlobXPerColumn;
    return shiftedX*LidarDefines.BlobXPerColumn+minX;
}

int16_t ShiftRealX(int16_t realX)
{
    int16_t minX = -90* LidarDefines.BlobXPerColumn;
    int16_t maxX = 90* LidarDefines.BlobXPerColumn;
    if ( (realX >minX) && (realX <maxX) ) 
        return (realX/LidarDefines.BlobXPerColumn)+90;
    else if  (realX <=minX)
        return 0;
    else 
        return 179;
    
    return ((realX - minX) / LidarDefines.BlobXPerColumn); 
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
    printf("\n[x%i(%i) y%i]\n",x,x-90,y);
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
	{
        // if we blobs in the list, first see if it part of one already.
		if (blobList[i].numSamples > 0)
		{
           
			if ((x >= (blobList[i].xLeft - 1)) && (x <= (blobList[i].xRight + 1)) &&
				(y >= (blobList[i].yLower - 1)) && (y <= (blobList[i].yUpper + 1)))
			{
				// Was this the first blob, this point was found in?  If so, just increase the size of the blob to include this point.
                // There are several cases here.  First case is one point is is just part of one blob.
                // Then as we continue to search, and  if we find this same point in another blob (found == true), . 
    //             printf("&i%i x%i y%i - ",i,x,y);
                //printf("&%i",i);
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
				{      // This point was already added into firstBlob so that means we can just merge this second blob into firstBlob and then delete this blob from the list. 
                       // This will continue if this point was part of a third or fourth one as well.
                    //printf("@%i",i);
      //              printf("@i%i x%i y%i - ",i,x,y);
					MergeSecondBlobIntoFirst(firstBlob, i);
				//	ClearBlobNumber(i);
				}
			}

		}
	}

    // If it was not found, create a new blob and put this point in it.
	if (found == false)
	{
		for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
		{
			if (blobList[i].numSamples == 0)
			{
                //printf("^%i",i);
   //             printf("^i%i x%i y%i - ",i,x,y);
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

int8_t addDetailedPointToBlobList(int8_t x, int8_t y, int16_t realX, int16_t realY)
{
	bool found = false;
	uint8_t firstBlob = 99;
	for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
	{
        // if we blobs in the list, first see if it part of one already.
		if (blobList[i].numSamples > 0)
		{
			if ((x >= (blobList[i].xLeft - 1)) && (x <= (blobList[i].xRight + 1)) &&
				(y >= (blobList[i].yLower - 1)) && (y <= (blobList[i].yUpper + 1)))
			{
				// Was this the first blob, this point was found in?  If so, just increase the size of the blob to include this point.
                // There are several cases here.  First case is one point is is just part of one blob.
                // Then as we continue to search, and  if we find this same point in another blob (found == true), .  
				if (found == false)
				{
					// do blob values
					if (x < blobList[i].xLeft)
						blobList[i].xLeft = x;
					else if (x > blobList[i].xRight)
						blobList[i].xRight = x;

					if (y < blobList[i].yLower)
						blobList[i].yLower = y;
					else if (y > blobList[i].yUpper)
						blobList[i].yUpper = x;

					// do precise Values
					if (realX < blobDetails[i].minX)
						blobDetails[i].minX = realX;
					else if (realX > blobDetails[i].maxX)
						blobDetails[i].maxX = realX;

					if (realY < blobDetails[i].minY)
						blobDetails[i].minY = realY;
					else if (realY > blobDetails[i].maxY)
						blobDetails[i].maxY = realY;


					found = true;
					firstBlob = i;
					blobList[i].numSamples++;
				}
				else   // This point is part of 2 or more blobs, merge this one into the first one found
				{      // This point was already added into firstBlob so that means we can just merge this second blob into firstBlob and then delete this blob from the list. 
                       // This will continue if this point was part of a third or fourth one as well.
					MergeSecondBlobIntoFirst(firstBlob, i);
					ClearBlobNumber(i);
				}
			}

		}
	}

    // If it was not found, create a new blob and put this point in it.
	if (found == false)
	{
		for (uint8_t i = 0; i < BLOBS_IN_LIST; i++)
		{
			if (blobList[i].numSamples == 0)
			{
				blobList[i].xLeft = blobList[i].xRight = x;
				blobList[i].yUpper = blobList[i].yLower = y;
				blobDetails[i].minX = blobDetails[i].maxX = realX;
				blobDetails[i].minY = blobDetails[i].maxY = realY;
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
	blobList[firstBlob].numSamples += blobList[secondBlob].numSamples;
	ClearBlobNumber(secondBlob);
    printf("M%d %d\n",firstBlob,secondBlob);
}

// The new blob will combine the two 
void MergeDetailedSecondBlobIntoFirst(uint8_t firstBlob, uint8_t secondBlob)
{
	blobList[firstBlob].xLeft = min(blobList[firstBlob].xLeft, blobList[secondBlob].xLeft);
	blobList[firstBlob].xRight = max(blobList[firstBlob].xRight, blobList[secondBlob].xRight);
	blobList[firstBlob].yUpper = max(blobList[firstBlob].yUpper, blobList[secondBlob].yUpper);
	blobList[firstBlob].yLower = min(blobList[firstBlob].yLower, blobList[secondBlob].yLower);
	blobDetails[firstBlob].minX = min(blobDetails[firstBlob].minX, blobDetails[secondBlob].minX);
	blobDetails[firstBlob].maxX = max(blobDetails[firstBlob].maxX, blobDetails[secondBlob].maxX);
	blobDetails[firstBlob].minY = min(blobDetails[firstBlob].minY, blobDetails[secondBlob].minY);
	blobDetails[firstBlob].maxY = max(blobDetails[firstBlob].maxY, blobDetails[secondBlob].maxY);

	blobList[firstBlob].numSamples += blobList[secondBlob].numSamples;
	ClearBlobNumber(secondBlob);
	printf("M%d %d\n", firstBlob, secondBlob);
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
 // angle a = arctan(y/x);
 // y/sin(a) = hyp/sin 90
 // y/sin(a) = hyp;
 
 uint16_t GetRealDistanceToBlobCenter(uint8_t blob)
 {
	 int16_t x = 0, y = 0;
	 uint16_t distance;
	 GetBlobCenter(blob, &x, &y);
	 //if ((yPerRow != 0) && (xPerColumn != 0))
     if ((LidarDefines.BlobYPerRow != 0) && (LidarDefines.BlobXPerColumn != 0))         
	 {
        int32_t x1 = (int32_t)GetRealX(x);
  		int32_t y1 = ((uint32_t)y * (uint32_t)LidarDefines.BlobYPerRow);
         
         distance = (uint16_t)round(sqrt(x1 * x1 + y1 * y1)); 
        // printf("(%i %i %3.1f %3.1f %3.1f, %i)\n",LidarDefines.BlobXPerColumn ,GetRealX(x),a, x1,y1, distance);
         //printf("(%i %i %li %li %d)\n",LidarDefines.BlobXPerColumn ,GetRealX(x),x1,y1, distance);
		
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
	float a = (float)centerY / (float)(centerX-90);
	a = (float)atan(a);
	return a;
}

// get angle to blob in radians
// Should return 0 to 180(0-1.5708).  0 Being left most angle.
float GetRealAngleToBlob(int8_t blob)
{
	int16_t centerX, centerY;
    int16_t realX;
    
    if ((LidarDefines.BlobYPerRow != 0) && (LidarDefines.BlobXPerColumn != 0))                 
	{
		// Compute Center of Blob
		GetBlobCenter(blob, &centerX, &centerY);
        realX = GetRealX(centerX);
        
		float a = (float)(centerY*LidarDefines.BlobYPerRow) / (float)(GetRealX(centerX));
        
        a = 0-(float)atan(a);
        if (a<0)
            a=0.0-a;
        else
            a = 3.1419-a;
       // printf("%d %d, %d %d, f=%3.1f \n",centerX,centerY,realX,(centerY*LidarDefines.BlobYPerRow) ,a*57.1);
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
	*angle = GetRealAngleToBlob(index);
	*distance = GetDistanceToBlobCenter(index);

}

//#define NUM_COLS  LidarDefines.BlobNumColumns 
//#define NUM_ROWS  LidarDefines.BlobNumRows

// to calculate what xPerColumn should be, figure out the max x distance in one direction in mm.  Then devide this by (NUM_COLS/2)
// to calculate what yPerRow should be, figure out the max Y distance in mm.  Then devide this by (NUM_ROWS/2)
//CreateBlobsFromFinalLineData(LidarDefines.BlobXPerColumn, LidarDefines.BlobYPerRow);

int CreateBlobsFromFinalLineData(uint16_t xPerColumnAx, uint16_t yPerRowAx)
{
    int16_t divX;
    uint16_t divY;
    //int16_t minX = 1000;

    // Will shift data so it fits in array from 0-180 and not -90 to 90
    // What is the
    // LidarDefines.BlobNumColumns = 180
    // LidarDefines.BlobXPerColumn .  Anything bigger than LidarDefines.BlobXPerColumn Needs to be limited
    
    minX = (LidarDefines.BlobNumColumns / -2) * LidarDefines.BlobXPerColumn;
    
    ClearBlobs();
    printf("\nLidarDefines.BlobXPerColumn = %i, LidarDefines.BlobYPerRow =%i\n  ",LidarDefines.BlobXPerColumn,LidarDefines.BlobYPerRow );
    for (int16_t row = LidarDefines.BlobNumRows; row > 0; row--)
    {
        for (uint16_t col = 0; col < 180; col++)
        {
            // If its not in the blob area, ignore it.
            if (finalLineData[col].sync_quality != 0)  
            {
                divY = (finalLineData[col].y / LidarDefines.BlobYPerRow);
                if (divY == row)
                {
                    divX = ShiftRealX(finalLineData[col].x);
                    //divX = (finalLineData[col].x - minX) / LidarDefines.BlobXPerColumn;
                    if ((divX >= 0) && (divX <= 180))
                    {
                           printf("%i<%i> %i<%i> %i %i divx=%d(%d)((%d)), divy=%d\n",
                                   finalLineData[col].angle_q6_checkbit,finalLineData[col].angle_q6_checkbit>>6,finalLineData[col].distance_q2,finalLineData[col].distance_q2>>2,finalLineData[col].x,finalLineData[col].y, 
                                   divX,divX-90, divX*LidarDefines.BlobXPerColumn+minX, divY); 
                           //printf("  divx=%d(%d)((%d)), divy=%d\n",divX,divX-90, divX*LidarDefines.BlobXPerColumn+minX, divY); 
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